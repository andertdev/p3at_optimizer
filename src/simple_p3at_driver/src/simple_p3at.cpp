#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "Aria.h"

using namespace std::chrono_literals;

class SimpleP3AT : public rclcpp::Node
{
public:
  SimpleP3AT() : Node("simple_p3at_driver")
  {
    // --- PARÂMETROS DE REDE ---
    this->declare_parameter("robot_ip", "192.168.0.1");
    this->declare_parameter("robot_port", 8101); // Porta padrão ARIA/Pioneer
    this->declare_parameter("cmd_timeout_s", 1.5); // Deadman menos agressivo para Wi-Fi real
    this->declare_parameter("use_wheel_mix", true); // Bypass setRotVel em links instaveis
    this->declare_parameter("axle_track_m", 0.42); // Distancia lateral aproximada entre rodas
    this->declare_parameter("connect_retries", 5); // Tentativas de conexão TCP/ARIA
    this->declare_parameter("connect_retry_s", 1.0); // Espera entre tentativas
    this->declare_parameter("battery_log_delta_v", 0.10); // Loga so quando variacao de tensao for relevante
    this->declare_parameter("battery_log_delta_soc", 1.0); // Loga so quando variacao de SoC for relevante

    std::string ip = this->get_parameter("robot_ip").as_string();
    int port = this->get_parameter("robot_port").as_int();
    int connect_retries = this->get_parameter("connect_retries").as_int();
    double connect_retry_s = this->get_parameter("connect_retry_s").as_double();
    battery_log_delta_v_ = this->get_parameter("battery_log_delta_v").as_double();
    battery_log_delta_soc_ = this->get_parameter("battery_log_delta_soc").as_double();
    cmd_timeout_s_ = this->get_parameter("cmd_timeout_s").as_double();
    use_wheel_mix_ = this->get_parameter("use_wheel_mix").as_bool();
    axle_track_m_ = this->get_parameter("axle_track_m").as_double();

    RCLCPP_INFO(this->get_logger(), "Tentando conectar em %s:%d ...", ip.c_str(), port);

    // Inicialização da ARIA
    Aria::init();

    // MUDANÇA: Usamos ArTcpConnection em vez de ArSerialConnection
    conn = new ArTcpConnection;
    robot = new ArRobot;
    sonar = new ArSonarDevice;

    // Configura o IP e Porta
    conn->setPort(ip.c_str(), port);

    robot->setDeviceConnection(conn);
    robot->addRangeDevice(sonar);

    // Tenta conectar (bloqueante), com retries para links Wi-Fi instáveis.
    bool connected = false;
    for (int attempt = 1; attempt <= connect_retries; ++attempt) {
      if (robot->blockingConnect()) {
        connected = true;
        break;
      }
      RCLCPP_WARN(this->get_logger(),
        "Falha ao conectar (%d/%d) em %s:%d. Nova tentativa em %.1fs...",
        attempt, connect_retries, ip.c_str(), port, connect_retry_s);
      std::this_thread::sleep_for(std::chrono::duration<double>(connect_retry_s));
    }
    if (!connected) {
      RCLCPP_FATAL(this->get_logger(), "FALHA DE CONEXÃO: Não foi possível conectar ao robô em %s:%d", ip.c_str(), port);
      RCLCPP_FATAL(this->get_logger(), "Verifique Wi-Fi, IP/porta e estabilidade do enlace.");
      exit(1);
    }

    // Liga os motores e sonares
    robot->enableMotors();
    robot->runAsync(true);
    // Zera velocidades logo após conectar para evitar qualquer comando residual.
    robot->lock();
    robot->setVel2(0.0, 0.0);
    robot->unlock();

    // Publishers e Subscribers
    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", cmd_qos, std::bind(&SimpleP3AT::cmd_callback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);

    // Timer de leitura (20Hz)
    timer_ = this->create_wall_timer(50ms, std::bind(&SimpleP3AT::update_loop, this));
    last_cmd_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "🚀 P3AT Conectado via WI-FI! Sonares Ativos.");
    RCLCPP_INFO(this->get_logger(), "Driver mode: use_wheel_mix=%s axle_track_m=%.3f cmd_timeout_s=%.2f",
      use_wheel_mix_ ? "true" : "false", axle_track_m_, cmd_timeout_s_);
  }

  ~SimpleP3AT() {
    robot->stopRunning();
    robot->disableMotors();
    Aria::shutdown();
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vel_mm_s = msg->linear.x * 1000.0;
    const double rot_deg_s = msg->angular.z * (180.0 / M_PI);
    const double half_track_m = axle_track_m_ * 0.5;
    const double left_mm_s = (msg->linear.x - msg->angular.z * half_track_m) * 1000.0;
    const double right_mm_s = (msg->linear.x + msg->angular.z * half_track_m) * 1000.0;

    robot->lock();
    if (use_wheel_mix_) {
      robot->setVel2(left_mm_s, right_mm_s);
    } else {
      robot->setVel(vel_mm_s);
      robot->setRotVel(rot_deg_s);
    }
    robot->unlock();
    last_cmd_time_ = this->now();
    deadman_stopped_ = false;

    // Keep cmd logs quiet during long runs; use debug only if needed.
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "cmd_vel: linear=%.3f angular=%.3f",
      msg->linear.x, msg->angular.z
    );
  }

  void update_loop()
  {
    if (!robot->isConnected()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Driver desconectado do robô (isConnected=false)."
      );
      return;
    }

    // Deadman: se parar de chegar cmd_vel, forca parada no robô.
    if ((this->now() - last_cmd_time_).seconds() > cmd_timeout_s_ && !deadman_stopped_) {
      robot->lock();
      if (use_wheel_mix_) {
        robot->setVel2(0.0, 0.0);
      } else {
        robot->setVel(0.0);
        robot->setRotVel(0.0);
      }
      robot->unlock();
      deadman_stopped_ = true;
      RCLCPP_WARN(this->get_logger(), "Deadman stop acionado (sem cmd_vel recente).");
    }

    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = "base_link";

    // Configuração simulada de Laser (360 graus)
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = (2.0 * M_PI) / 16.0;
    scan_msg.range_min = 0.1;
    scan_msg.range_max = 5.0;
    scan_msg.ranges.resize(16);

    robot->lock();
    for(int i=0; i<16; i++) {
        double range_m = robot->getSonarReading(i)->getRange() / 1000.0;
        if (range_m > 4.9) range_m = std::numeric_limits<float>::infinity();
        scan_msg.ranges[i] = range_m;
    }
    robot->unlock();

    scan_pub_->publish(scan_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    robot->lock();
    const double x_m = robot->getX() / 1000.0;
    const double y_m = robot->getY() / 1000.0;
    const double yaw_rad = robot->getTh() * (M_PI / 180.0);
    const double vel_x_m_s = robot->getVel() / 1000.0;
    const double vel_yaw_rad_s = robot->getRotVel() * (M_PI / 180.0);
    robot->unlock();

    odom_msg.pose.pose.position.x = x_m;
    odom_msg.pose.pose.position.y = y_m;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(yaw_rad * 0.5);
    odom_msg.pose.pose.orientation.w = std::cos(yaw_rad * 0.5);
    odom_msg.twist.twist.linear.x = vel_x_m_s;
    odom_msg.twist.twist.angular.z = vel_yaw_rad_s;
    odom_pub_->publish(odom_msg);

    auto batt_msg = sensor_msgs::msg::BatteryState();
    batt_msg.header.stamp = this->now();
    batt_msg.voltage = static_cast<float>(robot->getRealBatteryVoltageNow());
    batt_msg.percentage = static_cast<float>(robot->getStateOfCharge());
    batt_msg.present = true;
    battery_pub_->publish(batt_msg);

    const bool first_batt_log = !battery_log_initialized_;
    const bool voltage_changed = std::fabs(static_cast<double>(batt_msg.voltage) - last_battery_voltage_) >= battery_log_delta_v_;
    const bool soc_changed = std::fabs(static_cast<double>(batt_msg.percentage) - last_battery_soc_) >= battery_log_delta_soc_;
    if (first_batt_log || voltage_changed || soc_changed) {
      RCLCPP_INFO(
        this->get_logger(),
        "Battery: %.2f V (SoC: %.1f%%)",
        batt_msg.voltage,
        batt_msg.percentage
      );
      last_battery_voltage_ = static_cast<double>(batt_msg.voltage);
      last_battery_soc_ = static_cast<double>(batt_msg.percentage);
      battery_log_initialized_ = true;
    }
  }

  // MUDANÇA: Ponteiro TCP
  ArTcpConnection *conn;
  ArRobot *robot;
  ArSonarDevice *sonar;
  double cmd_timeout_s_{0.6};
  bool use_wheel_mix_{true};
  double axle_track_m_{0.42};
  double battery_log_delta_v_{0.10};
  double battery_log_delta_soc_{1.0};
  bool battery_log_initialized_{false};
  double last_battery_voltage_{0.0};
  double last_battery_soc_{0.0};
  rclcpp::Time last_cmd_time_;
  bool deadman_stopped_{true};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleP3AT>());
  rclcpp::shutdown();
  return 0;
}

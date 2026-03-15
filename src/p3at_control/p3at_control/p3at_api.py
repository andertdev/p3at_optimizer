import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from flask import Flask, request, jsonify
import threading
import json
import os
from datetime import datetime

from .cap_parser import parse_cap

app = Flask(__name__)
ros_node = None

@app.route('/')
def index():
    return "🤖 P3AT API Ready!", 200

@app.route('/command', methods=['POST'])
def handle_command():
    if not request.is_json:
        return jsonify({"status": "error", "message": "Body must be JSON"}), 400
    if not ros_node:
        return jsonify({"status": "error", "message": "ROS node not ready"}), 503

    data = request.json
    ok, out, msg = ros_node.publish_command(data)
    if not ok:
        return jsonify({"status": "error", "message": msg}), 400
    return jsonify({"status": "received", "data": out}), 200


class API_Node(Node):
    def __init__(self):
        super().__init__('p3at_api_node')

        self.declare_parameter('mode', 'real')
        self.declare_parameter('use_executor', True)
        self.declare_parameter('parser_mode', 'llm')  # llm|cap|dual
        self.declare_parameter('dual_route', 'llm_fallback_cap')  # llm_fallback_cap|cap_fallback_llm
        self.declare_parameter('parser_log_dir', '/tmp/p3at_parser_logs')
        self.mode = (self.get_parameter('mode').value or 'real').strip().lower()
        self.use_executor = bool(self.get_parameter('use_executor').value)
        param_parser_mode = str(self.get_parameter('parser_mode').value or 'llm').strip().lower()
        self.parser_mode = os.getenv('PARSER_MODE', param_parser_mode).strip().lower()
        self.dual_route = str(self.get_parameter('dual_route').value or 'llm_fallback_cap').strip().lower()
        self.parser_log_dir = str(self.get_parameter('parser_log_dir').value or '/tmp/p3at_parser_logs').strip()
        os.makedirs(self.parser_log_dir, exist_ok=True)

        # Real mode topics
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        # Sim+Real task channel
        self.pub_json = self.create_publisher(String, '/p3at_target', 10)

        self.get_logger().info(
            f"API Node started (/command). mode={self.mode} use_executor={self.use_executor} "
            f"parser_mode={self.parser_mode} dual_route={self.dual_route}"
        )

    def _append_dual_log(self, payload_in, llm_out, cap_out, routed_to):
        rec = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "payload_in": payload_in,
            "llm_out": llm_out,
            "cap_out": cap_out,
            "routed_to": routed_to,
        }
        path = os.path.join(self.parser_log_dir, "parser_dual_compare.jsonl")
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    @staticmethod
    def _has_structured_command(d):
        return isinstance(d, dict) and (
            "command" in d or "linear" in d or "angular" in d
        )

    def _extract_llm_candidate(self, payload):
        if not isinstance(payload, dict):
            return {}
        if self._has_structured_command(payload):
            return payload
        for k in ("llm_json", "command_json", "json"):
            v = payload.get(k)
            if isinstance(v, str):
                try:
                    v = json.loads(v)
                except Exception:
                    v = None
            if self._has_structured_command(v):
                return v
        return {}

    def _apply_parser(self, payload):
        if self.parser_mode == "cap":
            cmd, meta = parse_cap(payload)
            return True, cmd, {"parser_mode": "cap", "meta": meta}

        if self.parser_mode == "llm":
            llm_candidate = self._extract_llm_candidate(payload)
            if not llm_candidate:
                return False, None, {"error": "parser_mode=llm requires structured command JSON"}
            cmd, meta = parse_cap(llm_candidate)
            meta["source"] = "llm_structured"
            return True, cmd, {"parser_mode": "llm", "meta": meta}

        if self.parser_mode == "dual":
            cap_cmd, cap_meta = parse_cap(payload)
            llm_candidate = self._extract_llm_candidate(payload)
            llm_ok = bool(llm_candidate)
            llm_cmd = None
            llm_meta = {"source": "llm_structured", "input_type": "missing"}
            if llm_ok:
                llm_cmd, llm_meta = parse_cap(llm_candidate)
                llm_meta["source"] = "llm_structured"

            if self.dual_route == "cap_fallback_llm":
                routed = "cap" if cap_cmd else "llm"
                chosen = cap_cmd if cap_cmd else llm_cmd
            else:
                routed = "llm" if llm_ok and llm_cmd else "cap"
                chosen = llm_cmd if llm_ok and llm_cmd else cap_cmd

            self._append_dual_log(
                payload_in=payload,
                llm_out={"ok": llm_ok, "cmd": llm_cmd, "meta": llm_meta},
                cap_out={"ok": True, "cmd": cap_cmd, "meta": cap_meta},
                routed_to=routed,
            )
            return True, chosen, {
                "parser_mode": "dual",
                "routed_to": routed,
                "llm_ok": llm_ok,
                "cap_ok": True,
            }

        return False, None, {"error": f"Invalid parser_mode={self.parser_mode}"}

    def publish_command(self, payload):
        ok, cmd_dict, parse_info = self._apply_parser(payload)
        if not ok:
            return False, None, parse_info.get("error", "Parser error")

        # Em SIM: tudo vira JSON e vai para /p3at_target
        if self.mode == 'sim':
            msg = String()
            msg.data = json.dumps(cmd_dict)
            self.pub_json.publish(msg)
            return True, {"parsed": cmd_dict, "parse_info": parse_info}, "ok"

        if self.use_executor:
            msg = String()
            msg.data = json.dumps(cmd_dict)
            self.pub_json.publish(msg)
            return True, {"parsed": cmd_dict, "parse_info": parse_info}, "ok"

        # Em REAL sem executor: aceita apenas comandos diretos em Twist
        if 'linear' in cmd_dict or 'angular' in cmd_dict:
            msg = Twist()
            msg.linear.x = float(cmd_dict.get('linear', 0.0))
            msg.angular.z = float(cmd_dict.get('angular', 0.0))
            self.pub_twist.publish(msg)
            return True, {"parsed": cmd_dict, "parse_info": parse_info}, "ok"

        if self.mode == 'real':
            self.get_logger().warn("JSON command without linear/angular ignored (use_executor=False).")
            return False, None, "Command ignored (use_executor=False requires linear/angular)"

        return False, None, "Invalid mode"


def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = API_Node()

    t = threading.Thread(target=run_flask, daemon=True)
    t.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

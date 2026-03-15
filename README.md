# p3at_optimizer
Modular P3TA Sim-to-Real Generalization

# P3AT Control Stack

ROS 2 stack for Pioneer 3-AT (P3AT) navigation in simulation and on the real robot, with:

- a unified command interface on `/p3at_target`
- an HTTP API on `/command`
- reactive exploration mode in the executor
- optional integration with n8n + Groq/Llama 3
- automated validation, parsing benchmark, and multi-seed optimization

The workspace covers the full pipeline:

`command -> parser -> executor -> backend (Webots or real robot) -> validator -> reports`

## Architecture

Main layers:

1. **Command input**
   - local structured command
   - CAP baseline
   - LLM via n8n/Groq/Llama 3
2. **ROS/HTTP API**
   - Flask endpoint at `http://<host>:5000/command`
   - publishes tasks to `/p3at_target`
3. **Executor**
   - translates task JSON into motion
   - supports `explore_mode`
   - uses closed-loop odometry control for `move_distance` and `turn_angle`
4. **Backend**
   - simulation: ROS -> UDP -> Webots controller
   - real robot: ROS -> `/cmd_vel` -> `simple_p3at_driver` -> P3AT
5. **Evaluation**
   - `p3at_validator.py`
   - `real_robot_validation.py`
   - `run_optimizer_squad.py`

## Workspace packages

- `src/p3at_control`: API, executor, UDP bridge, launch files, optimizers, and validators
- `src/simple_p3at_driver`: real robot driver via ARIA/AriaCoda, publishing `/scan`, `/odom`, and battery state

## Requirements

- Ubuntu + ROS 2 Humble
- `colcon`
- Python 3
- Webots
- ARIA/AriaCoda installed for the real robot driver
- Python runtime dependencies:
  - `flask`
  - `numpy`
  - `matplotlib`
- optional:
  - n8n
  - Groq account / Llama 3 model access

## Build

```bash
cd ~/drivers_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select simple_p3at_driver p3at_control
source install/setup.bash
```

## Clean build artifacts

Remove `colcon` artifacts for a full rebuild:

```bash
cd ~/drivers_ws
rm -rf build install log
```

## Execution modes

### 1) Simulation

Terminal 1:

```bash
cd ~/drivers_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch p3at_control p3at.launch.py mode:=sim
```

This launches:

- `bus_udp`: ROS 2 -> UDP bridge for the Webots controller
- `api`: `/command` endpoint + `/p3at_target` publisher

In Webots, open the world and press `Play`.

### 2) Real robot

Terminal 1:

```bash
cd ~/drivers_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch p3at_control p3at.launch.py mode:=real
```

With network override:

```bash
ros2 launch p3at_control p3at.launch.py mode:=real robot_ip:=192.168.0.1 robot_port:=20001
```

This launches:

- `simple_p3at_driver`
- `executor`
- `api`

## Operational commands

### Explore mode

Enable:

```bash
ros2 topic pub --once /p3at_target std_msgs/msg/String '{data: "{\"command\":\"explore_mode\",\"value\":true}"}'
```

Stop:

```bash
ros2 topic pub --once /p3at_target std_msgs/msg/String '{data: "{\"command\":\"stop\",\"value\":0}"}'
```

### Direct structured commands

Move forward 1 m:

```bash
ros2 topic pub --once /p3at_target std_msgs/msg/String '{data: "{\"command\":\"move_distance\",\"value\":1.0}"}'
```

Move forward 3 m:

```bash
ros2 topic pub --once /p3at_target std_msgs/msg/String '{data: "{\"command\":\"move_distance\",\"value\":3.0}"}'
```

Move backward 1 m:

```bash
ros2 topic pub --once /p3at_target std_msgs/msg/String '{data: "{\"command\":\"move_distance\",\"value\":-1.0}"}'
```

Rotate 90 degrees:

```bash
ros2 topic pub --once /p3at_target std_msgs/msg/String '{data: "{\"command\":\"turn_angle\",\"value\":90.0}"}'
```

Rotate 180 degrees:

```bash
ros2 topic pub --once /p3at_target std_msgs/msg/String '{data: "{\"command\":\"turn_angle\",\"value\":180.0}"}'
```

### Local HTTP API

Structured motion command:

```bash
curl -X POST http://127.0.0.1:5000/command \
  -H "Content-Type: application/json" \
  -d '{"command":"move_distance","value":3.0}'
```

With CAP parser:

```bash
ros2 launch p3at_control p3at.launch.py mode:=sim parser_mode:=cap
```

With LLM structured parser:

```bash
ros2 launch p3at_control p3at.launch.py mode:=sim parser_mode:=llm
```

With dual parser mode:

```bash
ros2 launch p3at_control p3at.launch.py mode:=sim parser_mode:=dual dual_route:=llm_fallback_cap
```

## n8n and LLM layers

Expected flow:

`user -> n8n webhook -> Groq/Llama 3 -> JSON normalization -> POST /command -> /p3at_target -> executor -> backend`

Example call to the remote webhook configured in this environment:

```bash
curl -X POST http://192.168.18.121:5000/command \
  -H "Content-Type: application/json" \
  -d '{"text":"move forward 3 meters"}'
```

In `run_optimizer_squad.py`, the default LLM webhook is:

`http://192.168.18.121:5000/command`

## Optimization and generalization

The optimization pipeline runs on the same JSON validation contract and currently integrates:

- **MOEA/D + I-RecEd**
- **PyGAD**
- **NSGA-II** (experimental profile)

In the study, generalization is evaluated through:

- before/after comparison on the same task suite
- CAP vs LLM comparison
- Tail-Shielding ON/OFF ablation
- environment shift
- real-robot validation

### `run_optimizer_squad.py` summary

Typical sequence:

1. before validator
2. gene search
3. after validator
4. parser-aware scenario execution
5. reports and comparison figures

Paper defaults:

- `pop=16`
- `gens=20`
- `repeats=5`
- `seeds=3`

### Recommended execution order

1. start simulation
2. run LLM for one optimizer
3. run CAP for the same optimizer
4. run `--compare`
5. repeat for the other optimizer
6. run tail ablation
7. run the parsing benchmark
8. if needed, run real-robot validation

### Optimizer squad startup

Terminal 1:

```bash
cd ~/drivers_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch p3at_control p3at.launch.py mode:=sim
```

Terminal 2:

```bash
cd ~/drivers_ws/src/p3at_control/optimizer
```

LLM + PyGAD:

```bash
python3 run_optimizer_squad.py --pygad --llm
```

CAP + PyGAD:

```bash
python3 run_optimizer_squad.py --pygad --cap
```

Compare PyGAD:

```bash
python3 run_optimizer_squad.py --pygad --compare
```

LLM + MOEA/D:

```bash
python3 run_optimizer_squad.py --moead --llm
```

CAP + MOEA/D:

```bash
python3 run_optimizer_squad.py --moead --cap
```

Compare MOEA/D:

```bash
python3 run_optimizer_squad.py --moead --compare
```

### Parameterized executions

Examples with explicit seeds, resume, and higher validator timeout:

```bash
python3 run_optimizer_squad.py --pygad --cap \
  --seeds 3 --seed-base 43 --gens 7 --pop 6 --repeats 5 \
  --resume --validator-timeout 1200
```

```bash
python3 run_optimizer_squad.py --pygad --llm \
  --seeds 3 --seed-base 43 --gens 7 --pop 6 --repeats 5 \
  --resume --validator-timeout 1200
```

## Tail ablation

Question answered: does Tail-Shielding help beyond the effect of the optimized genes?

MOEA/D:

```bash
cd ~/drivers_ws/src/p3at_control/optimizer
python3 run_optimizer_squad.py --moead --tail-ablation --tail-parser cap --seeds 4 --seed-base 43 --repeats 5 --validator-timeout 1200
python3 run_optimizer_squad.py --moead --tail-ablation --tail-parser llm --seeds 4 --seed-base 43 --repeats 5 --validator-timeout 1200
```

Typical outputs:

- `tail_ablation_cap_moead_sXX.json`
- `tail_ablation_llm_moead_sXX.json`
- `moead_tail_ablation_aggregate.png`

## Parsing benchmark

Base dataset:

- `src/p3at_control/optimizer/llm_parsing_dataset_v1_120.json`

Execution:

```bash
cd ~/drivers_ws
python3 src/p3at_control/optimizer/llm_parsing_benchmark.py \
  --dataset src/p3at_control/optimizer/llm_parsing_dataset_v1_120.json \
  --outdir src/p3at_control/optimizer/scenario_results/llm_parsing \
  --llm-webhook http://192.168.18.121:5000/command \
  --delay-ms 300 \
  --max-retries 3 \
  --retry-backoff-ms 300
```

Outputs:

- `benchmark_results.json`
- `benchmark_cases.csv`
- `benchmark_summary.csv`
- `benchmark_summary.md`

## Real-robot validation

The real validator:

- applies genes to the `executor`
- runs a fixed task suite
- measures linear and angular error through odometry
- collects proximity data from `/scan`
- records manual aborts and partial progress
- writes JSON outputs under `scenario_results/real`

### Baseline

```bash
cd ~/drivers_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/p3at_control/optimizer/real_robot_validation.py \
  --label real_default \
  --repeats 5 \
  --env p3at_real_lab \
  --outdir src/p3at_control/optimizer/scenario_results/real
```

### Comparison with optimized genes

MOEA/D seed 44:

```bash
cd ~/drivers_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/p3at_control/optimizer/real_robot_validation.py \
  --label real_moead_s44 \
  --genes-file src/p3at_control/optimizer/scenario_results/moead/gen_cap/seed_44/best_genes_moead_s44.json \
  --repeats 5 \
  --env p3at_real_lab \
  --outdir src/p3at_control/optimizer/scenario_results/real
```

PyGAD seed 44:

```bash
cd ~/drivers_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/p3at_control/optimizer/real_robot_validation.py \
  --label real_pygad_s44 \
  --genes-file src/p3at_control/optimizer/scenario_results/pygad/gen_cap/seed_44/best_genes_pygad_s44.json \
  --repeats 5 \
  --env p3at_real_lab \
  --outdir src/p3at_control/optimizer/scenario_results/real
```

Generated files:

- `validation_real_default.json`
- `validation_real_moead_s44.json`
- `validation_real_pygad_s44.json`

## Outputs and artifacts

### Main directories

- `src/p3at_control/optimizer/scenario_results/pygad`
- `src/p3at_control/optimizer/scenario_results/moead`
- `src/p3at_control/optimizer/scenario_results/llm_parsing`
- `src/p3at_control/optimizer/scenario_results/real`

### Generated figures and reports

Classical comparison:

- `robustness_heatmap.png`
- `avg_vs_worst.png`
- `resets_duration.png`
- `worst_case_error.png`
- `time_vs_error.png`
- `learning_curve.png`

Parser-aware / paper summary:

- `cap_vs_llm_table.png`
- `cap_vs_llm_avg_vs_worst.png`
- `moead_baseline_cap_vs_llm.png`
- `moead_tail_ablation_aggregate.png`
- `moead_env_shift_aggregate.png`
- `modular_pipeline_overview.png`
- `system_architecture_diagram.png`
- `tail_shielding_mechanism.png`

## Operational notes

- in `sim` mode, Webots must be open and running
- in `real` mode, the robot must be connected and properly network-configured
- the executor uses closed-loop odometry control for distance and rotation tasks
- calibration and operational limit parameters are defined in `src/p3at_control/config/params.yaml`
- for a clean rebuild, remove `build/`, `install/`, and `log/`

# CROSS 2026 – LLM Agentica (Notas rápidas)
- Controller agora inicia em `0.0.0.0:20001` e sinaliza “conexão UDP estabelecida” na primeira mensagem.
- Before/After rodam com `validation_mode=True` (sem desvio durante task); otimização roda com `validation_mode=False` (desvio ativo).
- Stuck/reset mais rígido: thresholds maiores e reset em 3 se não sair do lugar.

Gráficos relevantes (gerados pelo compare):
- `robustness_heatmap.png`: success ↑, collisions/reset por tentativa ↓ (pares=Before, ímpares=After).
- `avg_vs_worst.png`: erro médio e pior caso por teste (menor é melhor).
- `resets_duration.png`: resets (barras) e duração média (linhas) por teste.
- `worst_case_error.png`: pior erro por teste (menor é melhor).
- `time_vs_error.png`: tempo médio vs erro (After ideal fica para baixo/esquerda).
- `learning_curve.png`: fitness do melhor indivíduo por geração.


Guia de Execução (SIM, CAP vs LLM, otimização e comparação)

Pré-requisitos:

Webots instalado.
ROS2 + colcon.
Workspace em: /home/iartes/drivers_ws
Repositório: /home/iartes/drivers_ws/src/p3at_control
Webhook LLM padrão no squad: http://192.168.18.121:5000/command
1) Terminal 1: preparar ambiente e subir stack ROS (simulação)

cd /home/iartes/drivers_ws
colcon build --packages-select p3at_control
source install/setup.bash
ros2 launch p3at_control p3at.launch.py mode:=sim
O que isso sobe:

bus_udp (ponte ROS2 -> UDP Webots)
api (/command), com parser configurável (cap|llm|dual)
Importante:

Abra o Webots com sua arena e dê Play.
2) Terminal 2: rodar experimento com LLM (webhook padrão)

cd /home/iartes/drivers_ws/src/p3at_control/optimizer
python3 run_optimizer_squad.py --pygad --llm
ou

python3 run_optimizer_squad.py --moead --llm
O que acontece:

Validator before (clássico)
Otimização (PyGAD ou MOEA/D)
Validator after (clássico)
Compare clássico (gráficos/relatório)
Scenario runner parser-aware em modo llm
(usa POST no webhook padrão para os comandos de teste NL)
3) Terminal 2: rodar experimento com baseline CAP

python3 run_optimizer_squad.py --pygad --cap
ou

python3 run_optimizer_squad.py --moead --cap
Mesma sequência, mas parser-aware em modo cap.

4) Terminal 2: comparar CAP vs LLM (offline, sem webhook)

Depois de gerar os JSON de CAP e LLM para o mesmo otimizador:

python3 run_optimizer_squad.py --pygad --compare
ou

python3 run_optimizer_squad.py --moead --compare
Esse comando:

não roda otimização
não chama webhook
compara apenas os JSON já gerados (cap_* vs llm_*)
gera resumo + tabela + figura
Saídas principais

Clássico (por otimizador):

/home/iartes/drivers_ws/src/p3at_control/optimizer/pygad/
/home/iartes/drivers_ws/src/p3at_control/optimizer/moead/
Parser-aware (por otimizador e seed):

/home/iartes/drivers_ws/src/p3at_control/optimizer/scenario_results/pygad/
/home/iartes/drivers_ws/src/p3at_control/optimizer/scenario_results/moead/
Arquivos relevantes:

cap_default.json, cap_pygad.json / cap_moead.json
llm_default.json, llm_pygad.json / llm_moead.json
cap_vs_llm_summary.json
cap_vs_llm_table.csv
cap_vs_llm_table.md
cap_vs_llm_table.png
cap_vs_llm_avg_vs_worst.png
Configuração padrão paper já aplicada

pop=16
gens=20
repeats=5
seeds=3
Você pode sobrescrever no comando, se precisar:

python3 run_optimizer_squad.py --pygad --llm --gens 20 --pop 16 --repeats 5 --seeds 3
Webhook LLM padrão (onde alterar)

Arquivo:

run_optimizer_squad.py
Constante:

DEFAULT_LLM_WEBHOOK = "http://192.168.18.121:5000/command"
Se mudar IP/porta do n8n, altere só essa linha.

Fluxo recomendado de execução completa

Subir sim (Terminal 1).
Rodar LLM com PyGAD (Terminal 2).
Rodar CAP com PyGAD (Terminal 2).
Rodar --compare com PyGAD (Terminal 2).
Repetir os passos 2–4 para MOEA/D.






# ---------------------------------------------------------
# FILE: params.yaml
# Base unificada para SIMULAÇÃO e ROBÔ REAL
# ---------------------------------------------------------

executor:
  ros__parameters:
    # Proteção contra deadlock
    linear_timeout_per_meter: 12.0
    angular_timeout_per_90deg: 8.0

    # Tolerâncias
    linear_epsilon: 0.02
    angular_epsilon_deg: 2.0
    # Exploratório (sem Brain)
    dist_stop: 0.70
    dist_avoid: 1.75
    turn_force: 1.0
    explore_speed: 0.20
    tail_cooldown: 0.4
    turn_duration: 0.7
    scan_timeout: 0.7
    lateral_avoid_factor: 1.25
    lateral_turn_hold: 0.9
    lateral_turn_cooldown: 1.2
    lateral_block_time: 0.8

p3at_real_driver:
  ros__parameters:
    # Rede do robô real
    robot_ip: 192.168.0.1
    robot_port: 20001

----------------------------------------------------------
curl -X POST http://192.168.18.121:5000/command \
  -H "Content-Type: application/json" \
  -d '{"text":"andar 3 metros"}'
----------------------------------------------------------  

Perfeito. Aqui vai um passo a passo pronto para README.

Tail-Shielding Ablation (ON/OFF) after optimization

Objective: compare tail_shielding=on vs off using already learned best genes.
Output: per-parser JSON/MD + consolidated CAP vs LLM figure for each optimizer.
Prerequisites

Webots running with your P3AT world.
ROS2 stack running in simulation mode.
Best genes already generated for seed 42:
best_genes_pygad_s42.json
best_genes_pygad_s42.json
best_genes_moead_s42.json
best_genes_moead_s42.json
n8n: not required for tail ablation (validator sends structured JSON directly).


Terminal 1 — Launch robot stack

cd /home/iartes/drivers_ws
source /opt/ros/<your_distro>/setup.bash
source install/setup.bash
ros2 launch p3at_control p3at.launch.py mode:=sim

Terminal 2 — Run Tail Ablation (PyGAD)

cd /home/iartes/drivers_ws/src/p3at_control/optimizer

# CAP
python3 run_optimizer_squad.py --pygad --tail-ablation --tail-parser cap --seeds 1 --seed-base 42 --repeats 5

# LLM
python3 run_optimizer_squad.py --pygad --tail-ablation --tail-parser llm --seeds 1 --seed-base 42 --repeats 5


Terminal 2 — Run Tail Ablation (MOEA/D)

cd /home/iartes/drivers_ws/src/p3at_control/optimizer

# CAP
python3 run_optimizer_squad.py --moead --tail-ablation --tail-parser cap --seeds 1 --seed-base 42 --repeats 5

# LLM
python3 run_optimizer_squad.py --moead --tail-ablation --tail-parser llm --seeds 1 --seed-base 42 --repeats 5
What the script does

Loads best genes for the selected optimizer/parser.
Runs validator twice:
tail_shielding=on
tail_shielding=off
Saves comparison report.
When both parsers (CAP and LLM) are available for same method/seed, generates consolidated figure automatically.
Genes lookup order

For --tail-parser llm:
best_genes_<method>_s42.json (or equivalent)
For --tail-parser cap:
best_genes_<method>_s42.json (or equivalent)
Outputs

PyGAD:

validation_cap_pygad_tail_on_s42.json
validation_cap_pygad_tail_off_s42.json
validation_llm_pygad_tail_on_s42.json
validation_llm_pygad_tail_off_s42.json
tail_ablation_cap_pygad_s42.json
tail_ablation_llm_pygad_s42.json
tail_ablation_pygad_cap_vs_llm_on_off.png



MOEA/D:

validation_cap_moead_tail_on_s42.json
validation_cap_moead_tail_off_s42.json
validation_llm_moead_tail_on_s42.json
validation_llm_moead_tail_off_s42.json
tail_ablation_cap_moead_s42.json
tail_ablation_llm_moead_s42.json
tail_ablation_moead_cap_vs_llm_on_off.png
Interpretation rule

In tail_ablation_*.json:
improvement_pct_on_vs_off > 0 means ON is better.
Metric direction:
lower is better: distance, duration, collisions, resets
higher is better: min_front_avg_m


---------------------------------------------
Passo a passo atualizado

Múltiplas seeds (gerar genes s42..s46)
(sem isso, não tem N suficiente para Wilcoxon/IC forte)
cd ~/drivers_ws/src/p3at_control/optimizer

# CAP -> gera best_genes_moead_s42..s46 em scenario_results/moead/gen_cap
python3 run_optimizer_squad.py --moead --cap --seeds 5 --seed-base 42 --repeats 5

# LLM -> gera best_genes_moead_s42..s46 em scenario_results/moead/gen_llm
python3 run_optimizer_squad.py --moead --llm --seeds 5 --seed-base 42 --repeats 5
Checagem:

ls src/p3at_control/optimizer/scenario_results/moead/gen_cap/best_genes_moead_s*.json
ls src/p3at_control/optimizer/scenario_results/moead/gen_llm/best_genes_moead_s*.json
Tail ablation ON/OFF para todas as seeds
cd ~/drivers_ws/src/p3at_control/optimizer

python3 run_optimizer_squad.py --moead --tail-ablation --tail-parser cap --seeds 5 --seed-base 42 --repeats 5
python3 run_optimizer_squad.py --moead --tail-ablation --tail-parser llm --seeds 5 --seed-base 42 --repeats 5
Separar treino/validação/teste (sem mudar código)
Treino: seeds usadas no otimizador (42–46)
Validação: ON/OFF nessas seeds
Teste não visto: novo cenário/arranjo/real, com --env como rótulo
Exemplo (já dá para fazer hoje com s42):

cd ~/drivers_ws/src/p3at_control/optimizer

python3 p3at_validator.py --label test_unseen_on_s42 --env webots_unseen_layout --repeats 5 --tail-shielding on  --genes-file scenario_results/moead/gen_cap/best_genes_moead_s42.json
python3 p3at_validator.py --label test_unseen_off_s42 --env webots_unseen_layout --repeats 5 --tail-shielding off --genes-file scenario_results/moead/gen_cap/best_genes_moead_s42.json
Estatística (mean/std/IC95 + Wilcoxon)
(use os tail_ablation_* após completar seeds)
python3 - <<'PY'
import json, glob, math, statistics
files = sorted(glob.glob("src/p3at_control/optimizer/scenario_results/moead/tail_ablation/seed_*/tail_ablation_cap_moead_s*.json"))
metric = "distance_avg_linear_m"
vals = [json.load(open(f))["improvement_pct_on_vs_off"][metric] for f in files]
n=len(vals); mean=statistics.mean(vals); std=statistics.stdev(vals) if n>1 else 0.0
ci=1.96*std/math.sqrt(n) if n>1 else 0.0
print(f"n={n} mean={mean:.4f}% std={std:.4f} IC95=[{mean-ci:.4f}, {mean+ci:.4f}]")
PY
python3 - <<'PY'
import json, glob
from scipy.stats import wilcoxon
files = sorted(glob.glob("src/p3at_control/optimizer/scenario_results/moead/tail_ablation/seed_*/tail_ablation_cap_moead_s*.json"))
metric="distance_avg_linear_m"
docs=[json.load(open(f)) for f in files]
off=[d["off"][metric] for d in docs]
on =[d["on"][metric]  for d in docs]
stat,p = wilcoxon(off, on, alternative="greater")
print(f"metric={metric} stat={stat} p={p}")
PY



cd ~/drivers_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch p3at_control p3at.launch.py mode:=sim

cd ~/drivers_ws/src/p3at_control/optimizer
python3 run_optimizer_squad.py --moead --tail-ablation --tail-parser cap --seeds 4 --seed-base 43 --repeats 5 --validator-timeout 1200
python3 run_optimizer_squad.py --moead --tail-ablation --tail-parser llm --seeds 4 --seed-base 43 --repeats 5 --validator-timeout 1200


Tail ablation é um teste para responder uma pergunta muito específica:

o Tail-Shielding realmente ajuda?
ou o robô melhora só por causa dos genes otimizados?
Para leigo, pense assim:

você tem um robô já ajustado
existe uma proteção extra chamada Tail-Shielding
essa proteção serve para evitar que o robô “se confunda” com leituras ruins na parte traseira durante curvas e manobras mais fechadas
o tail ablation é o experimento de ligar e desligar essa proteção e observar o que muda
O que é “ablation”
Em pesquisa, ablation significa:

pegar um sistema
remover uma parte dele
medir o quanto ele piora ou melhora
No seu caso:

sistema completo = genes otimizados + Tail-Shielding
sistema ablacionado = mesmos genes, mas sem Tail-Shielding
Então ele mede o efeito da peça isoladamente.

Como funciona no seu experimento
Para cada seed:

ele pega o best_genes já encontrado
roda a validação com:
tail-shielding on
roda a validação de novo com:
tail-shielding off
compara os dois resultados
O ponto mais importante é:

os genes são os mesmos
o parser é o mesmo
o cenário é o mesmo
a suíte de testes é a mesma
Então a única coisa que mudou foi:

Tail ON vs Tail OFF
Isso faz o teste ser justo.

O que ele mede
Ele compara métricas como:

erro médio linear
duração média
colisões
resets
distância mínima frontal média
Na prática:

se Tail ON reduz colisões ou resets, isso é muito relevante
se Tail ON melhora estabilidade sem piorar muito o tempo, melhor ainda
se Tail ON não muda nada, então talvez a proteção não seja tão importante quanto parecia
Como ele valida
Ele usa o mesmo p3at_validator.py.

Ou seja:

roda a mesma suíte de tarefas
mesmas repetições (repeats=5)
salva JSON
calcula o resultado agregado
depois gera um relatório comparando ON e OFF
Então ele não “inventa” uma validação nova.
Ele reutiliza o mesmo validador compartilhado.

Como ele compara
Ele faz algo como:

OFF:
erro = X
tempo = Y
colisões = Z
ON:
erro = X2
tempo = Y2
colisões = Z2
Depois calcula a melhora percentual de ON em relação a OFF.

Exemplo:

OFF teve mais colisões
ON teve menos
então a melhora é positiva para a camada de segurança
Por que isso é importante no experimento
Porque sem esse teste, alguém pode ler o paper e pensar:

“talvez tudo tenha melhorado só porque os genes ficaram bons”
“essa camada Tail-Shielding talvez nem faça diferença”
O tail ablation responde exatamente isso.

Ele mostra se:

a camada de segurança tem efeito real
esse efeito aparece de forma consistente
o ganho vem do mecanismo, e não só do otimizador
Por que isso fortalece o paper
Ele transforma o trabalho de:

“temos um sistema que funciona”
para:

“sabemos qual parte do sistema está contribuindo”
Isso é muito mais forte cientificamente.

Resumo simples

otimização encontra os melhores genes
tail ablation testa a proteção ligada e desligada
a comparação mostra se essa proteção realmente ajuda
isso prova causalidade da camada de segurança
Se quiser, eu também posso te explicar agora a diferença entre:

CAP vs LLM
Tail ablation
Env-shift
como três perguntas científicas diferentes do mesmo paper.


Essas três partes respondem três perguntas científicas diferentes.

1. CAP vs LLM
Pergunta:

o jeito de interpretar o comando em alto nível muda o desempenho do sistema?
O que compara:

parser CAP
parser LLM
O que fica igual:

mesmo executor
mesmo controlador
mesmo validador
mesmo robô / mesma simulação
O que isso mede:

impacto da camada de interpretação de comando
Em termos simples:

se eu der a mesma intenção ao sistema, CAP e LLM entregam resultados igualmente bons ou um deles é mais confiável?
2. Tail ablation
Pergunta:

a camada de segurança Tail-Shielding realmente ajuda?
O que compara:

Tail ON
Tail OFF
O que fica igual:

mesmos genes
mesmo parser
mesmo cenário
mesma suíte de testes
O que isso mede:

impacto causal da camada de segurança
Em termos simples:

com tudo igual, só ligar/desligar essa proteção muda o comportamento do robô?
3. Env-shift
Pergunta:

os genes aprendidos continuam funcionando quando o ambiente muda?
O que compara:

cenário baseline
cenário novo / não visto
O que fica igual:

mesmo gene set
mesmo método
mesma validação
O que isso mede:

generalização
Em termos simples:

o robô aprendeu algo útil de verdade ou só ficou bom naquele mundo específico?
Como as três se encaixam no paper
Cada uma cobre um risco diferente:

CAP vs LLM
risco: melhora depender só do parser
Tail ablation
risco: melhora depender só da heurística de segurança
Env-shift
risco: melhora ser overfitting ao cenário
Então, juntas, elas respondem:

quem interpreta melhor?
o mecanismo de segurança realmente contribui?
o que foi aprendido generaliza?
Resumo científico em uma linha

CAP vs LLM = efeito do parser
Tail ablation = efeito da camada de segurança
Env-shift = efeito de generalização
Por que isso é forte
Porque o paper deixa de ser:

“otimizei e funcionou”
e passa a ser:

“separei os fatores do sistema, medi cada um e mostrei o que realmente explica o desempenho”
Isso é exatamente o tipo de desenho experimental que melhora muito a qualidade da submissão.

Se quiser, eu posso transformar isso em um parágrafo pronto para colocar no artigo na seção Multi-Layer Experimental Protocol.


python3 run_optimizer_squad.py --pygad --cap --seeds 3 --seed-base 43 --gens 7 --pop 6 --repeats 5 --resume --validator-timeout 1200
python3 run_optimizer_squad.py --pygad --llm --seeds 3 --seed-base 43 --gens 7 --pop 6 --repeats 5 --resume --validator-timeout 1200




Implemented

Criei o validador para o robô real em real_robot_validation.py (line 1), e ajustei o stack para que os genes realmente afetem a execução no hardware.

Os pontos alterados foram:

real_robot_validation.py (line 68): executa a suíte no real, aplica genes via serviço de parâmetros do ROS 2, publica comandos estruturados em /p3at_target e salva o JSON no mesmo espírito do validador do simulador.
p3at_executor.py (line 28): agora usa explore_speed como base_speed, usa turn_speed nas rotações e incorpora avoid_gain nas decisões reativas do modo de exploração.
simple_p3at.cpp (line 88): agora publica /odom, que era o dado que faltava para medir erro linear e angular com rigor.
CMakeLists.txt (line 1) e package.xml (line 1): incluí nav_msgs para suportar a odometria.
Validação feita:

python3 -m py_compile nos arquivos Python alterados
colcon build --packages-select simple_p3at_driver
colcon build --packages-select p3at_control
Como ele mede

O validador no real captura:

erro linear: por projeção da odometria no heading inicial
erro angular: por delta de yaw em /odom
duração: janela entre início e fim de /executor_active
distância mínima frontal/lateral: de /scan
stopped_time_frac: proxy por velocidade linear/angular próxima de zero
avoid_time_frac: proxy por proximidade abaixo de dist_avoid
colisões: proxy por scan abaixo de collision_threshold
Isso é cientificamente aceitável para o ambiente atual, com duas ressalvas:

colisão é uma proxy por proximidade, não contato físico real
reset ainda não é observável no real como no Webots; hoje tende a ficar 0 a menos que vocês adicionem um canal explícito de falha/intervenção
Se quiser métrica de colisão forte para artigo final, o próximo passo é adicionar bumper, contato físico, ou um evento explícito do driver/executor.

Como rodar

Baseline com parâmetros atuais do executor:

python3 /home/iartes/drivers_ws/src/p3at_control/optimizer/real_robot_validation.py \
  --label real_default \
  --repeats 5 \
  --env p3at_real_lab \
  --outdir /home/iartes/drivers_ws/src/p3at_control/optimizer/scenario_results/real
Perfil otimizado:

python3 /home/iartes/drivers_ws/src/p3at_control/optimizer/real_robot_validation.py \
  --label real_moead_s44 \
  --genes-file /home/iartes/drivers_ws/src/p3at_control/optimizer/scenario_results/moead/gen_cap/seed_44/best_genes_moead_s44.json \
  --repeats 5 \
  --env p3at_real_lab \
  --outdir /home/iartes/drivers_ws/src/p3at_control/optimizer/scenario_results/real
O desenho experimental que eu recomendo agora é:

default
top-3 perfis otimizados
mesma suíte
repeats=5
mesma pose inicial e mesmo ambiente
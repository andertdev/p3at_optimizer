## Visão geral do repositório

- Monorepo ROS2 com dois pacotes principais:
  - `p3at_control` (Python, `ament_python`) — pilha de alto nível: `brain`, `gatekeeper`, `executor`, `api`, otimizadores e pontes para simulação.
  - `simple_p3at_driver` (C++, `ament_cmake`) — driver de hardware que faz link com a libAria; gera o executável `simple_p3at_node`.

## Arquitetura / fluxo de dados

- O `p3at.launch.py` (em `p3at_control/launch`) orquestra dois modos de execução:
  - `sim`: sobe `api` + `bus_udp` (conecta ao controlador Webots via UDP). O Webots é executado externamente; o controlador está em `controllers/p3at_webots_controller`.
  - `real`: sobe `simple_p3at_node` (driver C++) + `brain`, `gatekeeper`, `executor`, `api` (pipeline ROS completo).
- Comunicação: nós usam tópicos/serviços/parameters do ROS; o controlador do Webots comunica com o ROS via UDP (veja `p3at_webots_controller.py`).

## Build & execução (fluxos de desenvolvedor)

- Build do workspace (a partir da raiz):

  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```

- Executar simulação (Webots executado separadamente):

  ```bash
  # inicie o Webots manualmente, depois no workspace
  ros2 launch p3at_control p3at.launch.py mode:=sim
  ```

- Executar pipeline real (hardware):

  ```bash
  # garanta que libAria esteja instalada e disponível em /usr/local
  ros2 launch p3at_control p3at.launch.py mode:=real
  ```

- Executar nós individualmente (entradas em `setup.py`):

  ```bash
  ros2 run p3at_control api
  ros2 run p3at_control brain
  ros2 run p3at_control bus_udp
  ros2 run p3at_control webots_bus
  ros2 run simple_p3at_driver simple_p3at_node
  ```

## Padrões do projeto e armadilhas conhecidas

- `p3at_control` é um pacote `ament_python`; os pontos de entrada de tempo de execução estão em `setup.py` sob `console_scripts` — prefira `ros2 run`/`ros2 launch` em vez de chamar `main()` diretamente.
- Parâmetros estão centralizados em `p3at_control/config/params.yaml`. Os launch files passam esse arquivo para os nós (veja `p3at.launch.py`).
- O controlador do Webots usa UDP para enviar comandos e status para processos ROS — inspecione `controllers/p3at_webots_controller/p3at_webots_controller.py` para o formato das mensagens e portas (`20001` comandos, `20002` status).
- O driver C++ faz link com a lib `Aria` (veja `simple_p3at_driver/CMakeLists.txt`). Em máquinas sem `Aria` em `/usr/local`, o build C++ falhará; instale ou crie um symlink da biblioteca conforme necessário.

## Testes e verificações estáticas

- `package.xml` lista dependências de teste (`ament_flake8`, `pytest`), porém não há testes de nível superior detectados automaticamente. Use `colcon test` após o build para executar testes disponíveis.

## Arquivos-chave para inspeção

- Orquestração de launch: `p3at_control/launch/p3at.launch.py` ([p3at.launch.py](src/p3at_control/launch/p3at.launch.py#L1-L200)).
- Controlador de simulação (protocolo UDP): `controllers/p3at_webots_controller/p3at_webots_controller.py` ([p3at_webots_controller.py](src/p3at_control/controllers/p3at_webots_controller/p3at_webots_controller.py#L1-L400)).
- Entrypoints e scripts de runtime: `p3at_control/setup.py` ([setup.py](src/p3at_control/setup.py#L1-L200)).
- Build do driver e bibliotecas do sistema: `simple_p3at_driver/CMakeLists.txt` ([CMakeLists.txt](src/simple_p3at_driver/CMakeLists.txt#L1-L200)).
- Parâmetros centrais: `p3at_control/config/params.yaml` ([params.yaml](src/p3at_control/config/params.yaml#L1-L200)).

## Exemplos úteis (copy/paste)

- Subir stack de simulação (após build e source):

  ```bash
  ros2 launch p3at_control p3at.launch.py mode:=sim
  ```

- Fluxo de edição rápido para o controlador Webots:

  - Edite `controllers/p3at_webots_controller/p3at_webots_controller.py` no editor.
  - Reinicie o Webots e execute novamente `ros2 launch p3at_control p3at.launch.py mode:=sim` (o controlador do Webots é um processo separado).

## Onde agentes de IA devem ter cuidado

- Evite alterar portas UDP ou formatos de mensagem sem atualizar tanto o controlador do Webots quanto a ponte UDP do ROS (`bus_udp`/`webots_bus`).
- Ao modificar configurações de link no `CMakeLists.txt`, verifique os caminhos da `Aria` nas máquinas alvo; prefira documentar passos adicionais de instalação.

---

Se alguma seção estiver pouco clara ou você quiser mais exemplos (logs de runtime, comandos de debug comuns ou dicas de CI/build), diga o que quer que eu amplie.

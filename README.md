# ROS2Gazebo

# Link para o video no YouTube
  https://youtube.com/shorts/-p8D8h1KyQc?si=eQsCDkq8eS3xrrCg
  
# TurtleBot3 Waypoint Navigator (ROS 2)

Este projeto implementa um sistema de navegação autônoma para o TurtleBot3 no simulador Gazebo, utilizando o ROS 2 (Python). O robô é capaz de visitar quatro alvos (representando blocos coloridos) e retornar à posição inicial após cada visita, contornando obstáculos por meio de rotas planejadas com waypoints.

## 🚀 Funcionalidades

- Navegação baseada em waypoints até múltiplos objetivos
- Retorno automático à origem após alcançar cada alvo
- Controle de posição e orientação via odometria (`/odom`)
- Trajetórias planejadas manualmente para evitar obstáculos fixos
- Encerramento automático ao fim da missão

## 📂 Estrutura do Projeto

```
turtlebot3_control_ros2/
├── navigator_node.py      # Nó principal de navegação
├── package.xml
└── README.md              # Este arquivo
```

## 🔧 Execução

1. Certifique-se de ter o ROS 2 instalado (foxy, humble, iron etc.).
2. Clone o repositório no seu workspace ROS 2:

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/seu-usuario/turtlebot3_control_ros2.git
```

3. Compile o workspace:

```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

4. Inicie o simulador Gazebo com o mundo desejado.

5. Execute o nó de navegação:

```bash
ros2 run turtlebot3_control_ros2 navigator_node
```

## 📌 Estratégia de Navegação

A navegação é feita por caminhos predefinidos (`forward_paths`) compostos por waypoints intermediários cuidadosamente posicionados. O robô orienta-se até cada ponto usando cálculos de ângulo e distância com base na odometria.

Após alcançar o objetivo, o robô retorna à origem pelo caminho inverso. Esse processo é repetido para todos os alvos.

## 🚧 Desvios de Obstáculos

Este projeto **não utiliza sensores como LIDAR**. Os obstáculos são evitados através do planejamento manual das rotas. Portanto, obstáculos dinâmicos ou não previstos no mapa não serão evitados automaticamente.

## ⚠️ Limitações

- Não reage a obstáculos inesperados
- Sensível a ruído na odometria
- Caminhos fixos, sem replanejamento dinâmico

## ✅ Melhorias Futuras

- Integração com `/scan` para desvio dinâmico de obstáculos
- Planejamento de rotas com A*/Dijkstra/RRT
- Controle refinado com PID
- Visualização dos waypoints no Gazebo

## 🧠 Requisitos

- ROS 2 (testado em Ubuntu 22.04 com Humble)
- Gazebo
- TurtleBot3 (model Burger)

---

Desenvolvido por Rodrigo Dias Fagundes

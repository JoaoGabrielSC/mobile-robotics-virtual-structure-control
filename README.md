# robotica-sim

Simulador de estrutura virtual para o robô terrestre **LIMO** e o quadrirrotor **Parrot Bebop 2**, desenvolvido no contexto da disciplina *Robótica Móvel* (UFES, 2026/1).

Este repositório implementa em Python a arquitetura **Inner–Outer Loop** descrita no enunciado: controlador cinemático da formação (laço externo), compensadores dinâmicos dos dois robôs (laço interno), desvio de obstáculo em espaço nulo e integração numérica da planta.

---

# Especificação do Trabalho – Robótica Móvel (2026/1)

## Objetivo

Projetar e implementar um sistema de controle para uma **estrutura virtual** composta por:

- 1 robô terrestre diferencial **LIMO**;
- 1 quadrirrotor **Parrot Bebop 2**.

O controlador deverá utilizar a estratégia de **Estrutura Virtual (Virtual Structure)**, composta por:

- controlador cinemático da formação (laço externo);
- compensador dinâmico para o LIMO (laço interno);
- compensador dinâmico para o Bebop 2 (laço interno).

Em outras palavras, o sistema deverá possuir uma arquitetura **Inner-Outer Loop**.

---

## Estrutura da formação

O ponto de controle da formação é definido no **ponto de controle do robô LIMO**, deslocado:

- 10 cm do centro de gravidade;
- ao longo do eixo X do robô;
- ângulo de deslocamento igual a 0°.

Ou seja,

- a = 10 cm
- direção = eixo X do robô

---

## Modelos utilizados

### LIMO

Utilizar o modelo dinâmico fornecido pelo professor, juntamente com os parâmetros disponibilizados no enunciado.

### Bebop 2

O modelo dinâmico é dado por

$$
\dot{\mathbf v}=f_1\mathbf u-f_2\mathbf v
$$

onde os parâmetros fornecidos correspondem às seguintes limitações:

- arfagem (θ) limitada a 5°
- rolagem (φ) limitada a 5°
- velocidade vertical máxima:

$$
v_z = 1\;\mathrm{m/s}
$$

- velocidade angular máxima

$$
\dot\psi =100\;\mathrm{rad/s}
$$

---

## Trajetória desejada

A formação deverá seguir uma **lemniscata de Bernoulli** no plano XY.

A referência é

$$
q_d=
[x_f,\;
y_f,\;
z_f,\;
\rho_f,\;
\alpha_f,\;
\beta_f]^T
$$

onde

- altura da formação:

$$
\rho_f=1.5\;\mathrm{m}
$$

- orientação:

$$
\alpha_f=0^\circ
$$

$$
\beta_f=90^\circ
$$

As coordenadas da trajetória são

$$
x_d=0.75\sin\left(\frac{2\pi t}{40}\right)
$$

$$
y_d=0.75\sin\left(\frac{4\pi t}{40}\right)
$$

---

## Frequência do controlador

O loop de controle deverá executar a

- 30 Hz

Logo,

$$
T=\frac1{30}\;\mathrm{s}
$$

---

## Condições iniciais

### LIMO

Posição inicial

```
(0.40 , -0.25 , 0.00) m
```

Orientação inicial

```
alinhado ao eixo X global
```

### Drone

Inicialmente

- aproximadamente 30 cm à esquerda do LIMO;
- alinhado com o mesmo eixo X.

---

## Obstáculo

Considerar um obstáculo cilíndrico fixo.

Centro da base

```
(-0.20 , 0.425 , 0.00) m
```

Raio

```
0.15 m
```

Representação física sugerida:

- um balde do laboratório.

---

## Desvio de obstáculo

O trabalho exige utilizar

**Controle baseado em Espaço Nulo (Null Space Control)**.

A prioridade deverá ser

1. evitar obstáculo;
2. controlar a formação.

A manobra evasiva somente deverá ser ativada quando o robô entrar na região de influência.

Raio da região de influência

```
0.50 m
```

Centro

```
centro do obstáculo
```

Fora dessa região, apenas o controlador da formação deverá atuar.

---

## Código MATLAB

O professor disponibilizou exemplos de integração com ROS.

Os principais recursos são:

### Inicialização

```matlab
rosshutdown;
rosinit('192.168.0.100');
```

### Publicadores ROS

Criar publicadores para

- cmd_vel
- takeoff
- land

Exemplo

```matlab
pub_cmdvel = rospublisher('/NAMESPACE/cmd_vel','geometry_msgs/Twist');
```

### Subscriber

Leitura da pose pelo OptiTrack

```matlab
pose = rossubscriber('vrpn_client_node/NAMESPACE/pose');
```

### Joystick

```matlab
J = vrjoystick(1);
```

### Durante o loop

Enviar comandos de velocidade

```matlab
send(pub_cmdvel,msg_cmdvel)
```

### Drone

Decolagem

```matlab
send(pub_takeoff,msg_takeoff);
```

Pouso

```matlab
send(pub_land,msg_land);
```

### Leitura da pose

Ler

- posição
- orientação (quaternion)
- converter quaternion para ângulos de Euler

Exemplo

```matlab
quat = [w x y z];
EulZYX = quat2eul(quat);
```

### Encerramento

Ao finalizar o programa

```matlab
rosshutdown;
```

---

## Entregáveis esperados

O projeto deverá conter:

- controlador cinemático da estrutura virtual;
- compensador dinâmico do LIMO;
- compensador dinâmico do Bebop 2;
- arquitetura em laço interno e laço externo;
- seguimento da trajetória em forma de lemniscata;
- manutenção da altura do drone em 1,5 m;
- controle da formação entre os dois robôs;
- desvio de obstáculo utilizando Controle em Espaço Nulo;
- implementação em MATLAB utilizando ROS;
- leitura da pose via OptiTrack;
- envio de comandos aos robôs via tópicos ROS.

---

# Simulador Python (este repositório)

## Execução em hardware (MATLAB + ROS)

O arquivo `main.m` na raiz do projeto implementa o mesmo controlador conectado aos robôs reais via ROS e OptiTrack, seguindo o anexo MATLAB do PDF.

```matlab
main
```

Antes de executar, ajuste em `main.m`:

- `cfg.ros_ip` — IP do servidor ROS (padrão `192.168.0.100`)
- `cfg.limo_namespace` — namespace do LIMO no launch (ex.: `L1`)
- `cfg.bebop_namespace` — namespace do Bebop 2 no launch (ex.: `B1`)

O script publica `cmd_vel`, envia `takeoff`/`land` ao drone, lê poses em `vrpn_client_node/<NAMESPACE>/pose` e usa o joystick para parada de emergência.

## Pré-requisitos

- [uv](https://docs.astral.sh/uv/) package manager

## Instalação

```bash
uv sync
```

## Execução

```bash
uv run python src/main.py --output-dir results --no-show
```

Ver opções disponíveis:

```bash
uv run python src/main.py --help
```

Animar os dois robôs:

```bash
uv run python src/main.py --t_final 40 --anim
```

## Parâmetros da CLI

| Flag | Padrão | Descrição |
|------|--------|-----------|
| `--t_final` | `100.0` | Tempo total de simulação (s) |
| `--dt` | `1/30` | Período de amostragem T (30 Hz) |
| `--kq` | `1.2` | Ganho proporcional do controlador da formação |
| `--lq` | `0.8` | Limite de saturação da tangente hiperbólica |
| `--kd_limo` | `4.0` | Ganho do compensador dinâmico do LIMO |
| `--anim` | off | Anima a movimentação do LIMO e do Bebop 2 |
| `--output-dir` | `output` | Diretório onde os gráficos são salvos |
| `--no-show` | off | Salva os gráficos sem abrir janelas interativas |

## Saída

Por padrão, os gráficos são salvos em `output/`:

- `robots.png` — trajetórias 3D e vista superior, com obstáculo
- `formation_errors.png` — referência vs. estado da formação e erros

## Estrutura do projeto

```
src/
├── main.py       # Ponto de entrada
├── config.py     # Dataclass de configuração da simulação
├── cli.py        # Parsing de argumentos da CLI
├── simulator.py  # Loop de simulação, controladores e integração da planta
└── plotting.py   # Visualização e exportação dos gráficos
```

## Arquitetura de controle implementada

Estado da formação `q = [xf, yf, zf, ρ, α, β]`, com PoI no ponto de controle do LIMO:

1. **Laço externo** — controlador cinemático da formação com saturação por tangente hiperbólica
2. **Espaço nulo** — desvio de obstáculo (prioridade) com rastreamento da formação no espaço nulo
3. **Jacobiano inverso** — mapeia velocidades da formação para comandos do LIMO e do Bebop
4. **Laço interno** — compensador dinâmico do LIMO (regressão) e compensador simplificado do Bebop, com regulador de altitude
5. **Integração** — método de Euler para atualizar as poses dos robôs

# Loop de controle — `matlab/limo_bebop_final.m`

## Antes do loop

Aguarda poses do LIMO e do Bebop (OptiTrack) e o Botão A. Ao pressionar A: envia `takeoff` real, espera `takeoff_wait_s` (5 s), reseta todo o estado (tempo, `dvd_B`, velocidades anteriores) e libera o loop.

## A cada ciclo (30 Hz), enquanto voando

### 1. Sensores

- Lê pose real do LIMO: `x1, y1, z1, ψ1`
- Lê pose real do Bebop: `x2, y2, z2, ψ2`
- Watchdog: se alguma pose não atualizou há mais de 0,5 s → aborta e pousa.

### 2. Ponto de controle do LIMO

```
poi = [x1 + a1·cos(ψ1);  y1 + a1·sin(ψ1)]
```

Não é o centro do robô, é um ponto 10 cm à frente — usado em tudo daqui pra frente como "posição do LIMO".

### 3. Parede virtual (segurança)

Compara a posição **real medida** do Bebop (`p2 = [x2;y2;z2]`) com os limites configurados. Se fora → aborta e pousa. Independente de qualquer cálculo de controle.

### 4. Referência e controle do LIMO

**Em preparação** (`t < preparation_time_s`): LIMO fica parado.

- REF: `ref_xy = poi` (referência = própria posição, erro zero por definição)
- ENVIADO: `cmdL = [0; 0]`

**Fase ativa**:

- REF: lemniscata `xd(t), yd(t)` (+ derivadas `ẋd, ẏd`) — a referência de posição/velocidade que o LIMO deve seguir.
- Calcula erro `err_xy = ref_xy − poi` → lei cinemática saturada (`tanh`) → `vel_poi` (velocidade desejada do PoI no mundo).
- Se o PoI estiver perto do obstáculo: `vel_poi` é redirecionado pela evasão em espaço nulo (prioridade máxima sobre seguir a lemniscata).
- Converte `vel_poi` (mundo) para `[v; ω]` do robô (inversão cinemática `A1inv`) → `v_d`.
- Compensador dinâmico (`limo_inner_loop`, modelo identificado `theta_limo`) → **ENVIADO**: `cmdL = [v; ω]`.

**`/L1/cmd_vel` enviado**: `Linear.X = cmdL(1)` (v), `Angular.Z = cmdL(2)` (ω), `Linear.Y = Linear.Z = 0` (LIMO não se move de lado nem verticalmente).

### 5. Referência e controle do Bebop

**REF (alvo de posição)**:

- Se `TRAJ=1` (padrão): `p2d = [poi; z1] + offset_f` — Bebop segue a formação em cima/ao lado do LIMO, seja em preparação ou na fase ativa.
- Se `TRAJ=0` fora da preparação: `p2d = cfg.p2d_teste` (alvo fixo).

**Cálculo**:

- Erro `p2d − p2` (`p2` = posição **real** medida do Bebop) → lei cinemática saturada (`tanh`) somada ao feedforward `vel_poi_ff` (a velocidade que o LIMO está tentando seguir, pra o Bebop acompanhar o movimento, não só a posição) → `dx2` (velocidade desejada no mundo).
- Rotaciona `dx2` pro corpo do Bebop (`A2inv`).
- Yaw: `w_d_B = k_yaw·wrap_π(yaw_d − ψ2)`, saturado — mantém o Bebop alinhado ao eixo X global.
- `vd_B = [A2inv·dx2; w_d_B]` — velocidade desejada no corpo (4 componentes: vx, vy, vz, ψ̇).
- `vB_meas` — velocidade **real medida** do Bebop (diferença finita de `p2`/`ψ2`; aqui, diferente do LIMO, é feedback de verdade).
- `dvd_B` — aceleração desejada (diferença finita de `vd_B`; zerada no 1º ciclo de voo e na transição preparação→ativa).
- Compensador dinâmico (`f1`, `f2`, `KD_B`) → `cmdB_raw` → satura em `cmdB_max` → **ENVIADO**: `cmdB`.

**`/B1/cmd_vel` enviado**: `Linear.X = cmdB(1)` (vx), `Linear.Y = cmdB(2)` (vy), `Linear.Z = cmdB(3)` (vz), `Angular.Z = cmdB(4)` (ψ̇) — os 4 graus de liberdade controláveis do Bebop.

### 6. Auditoria e fim de ciclo

Registra histórico/log; se o tempo de missão (`Tsim`) acabou → aborta e pousa.

## Ao sair do loop

(qualquer motivo: Botão B, watchdog, parede virtual, fim do tempo, exceção)

Zera `cmd_vel` de ambos, manda `land` 3× ao Bebop, `rosshutdown`, salva auditoria e gráfico da trajetória XY.

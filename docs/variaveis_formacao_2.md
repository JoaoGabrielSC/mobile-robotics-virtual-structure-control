# Variáveis do `matlab/formacao_2.m`

Sumário das variáveis mais importantes da simulação de formação LIMO–Bebop, organizado por bloco funcional. Referências apontam para [`matlab/formacao_2.m`](../matlab/formacao_2.m).

## 1. Controle de execução

| Variável | Significado |
|---|---|
| [`MODO_BEBOP`](../matlab/formacao_2.m:72) | `'off'` (Bebop simulado em software), `'teste'` ou `'voo'` (Bebop real recebe `cmd_vel`) |
| [`TRAJ`](../matlab/formacao_2.m:40) | `0`: cluster para na origem (teste isolado do compensador do Bebop); `1`: lemniscata completa |
| [`cfg.auto_takeoff`](../matlab/formacao_2.m:12) | `false` (padrão): decolagem manual pelo piloto, confirmada por botão do joystick; `true`: script chama o serviço de takeoff e espera `cfg.takeoff_wait_s` |
| `em_preparacao` | `true` enquanto `t < preparation_time_s`: LIMO parado, Bebop convergindo até `p2d` com soft start. Só existe fase de preparação se `MODO_BEBOP ~= 'off'` |
| `t`, `t_traj`, `dt` | `t`: tempo desde o início do loop; `t_traj = max(0, t - preparation_time_s)`: tempo desde o início da trajetória ativa; `dt`: passo medido (não o nominal `cfg.T`), robusto a jitter do loop |
| `gamma` | Ganho de soft start, `∈[cfg.soft_start_gamma_min, 1]`, rampa em `cfg.soft_start_time_s` a partir de `t=0` |

## 2. Estado do cluster (Estrutura Virtual)

O cluster é representado por `q = [xf; yf; zf; ρ; α; β]` — posição do LIMO (ponto 1) mais a forma da formação (distância e ângulos até o Bebop, ponto 2).

| Variável | Significado |
|---|---|
| `poi` | Ponto de interesse do LIMO no mundo, deslocado `cfg.a1=0,10 m` à frente do centro do robô: `poi = [x1+a1·cosψ1; y1+a1·sinψ1]` |
| `p2` | Posição medida (ou virtual) do Bebop `[x2;y2;z2]` |
| `q` | Estado atual do cluster, calculado por `cluster_state(p1,p2)` a partir das posições reais dos dois robôs (`ρ,α,β` medidos, não só o alvo) |
| `qd` | Estado desejado do cluster: `[xd;yd;0; ρ_f;α_f;β_f]` — posição vem da lemniscata, forma é constante |
| `qd_dot` | Derivada de `qd`: feedforward de velocidade da lemniscata nas 2 primeiras posições, zero no resto |
| `q_tilde` | Erro do cluster, `qd - q`, com wrap de ângulo nas componentes 5:6 (`α,β`) |
| `q_dot_r` | Saída da lei cinemática (Eq. 5.7): `qd_dot + L·tanh(L⁻¹·K·q_tilde)` — velocidade de referência do cluster, já com o desvio de obstáculo aplicado |
| [`rho_f`, `alpha_f`, `beta_f`](../matlab/formacao_2.m:41) | Forma desejada da formação: `ρ_f=1,5 m` (distância LIMO–Bebop), `α_f=0`, `β_f=60°` |

**Convenção de α/β deste projeto** (documentada no cabeçalho do arquivo): `β` = elevação, `α` = azimute — **trocada em relação à Eq. 5.5b do livro-texto** (que usa `α`=elevação). Mantida assim por já estar validada no restante do código. Com os valores atuais, o Bebop fica a ≈1,30 m de altura e ≈0,75 m de deslocamento lateral do LIMO. A singularidade desta convenção é em `β=90°` (Bebop exatamente acima do LIMO).

## 3. Ganhos da lei cinemática

| Variável | Significado |
|---|---|
| `K`, `L` (6×6, montadas no loop) | Diagonais: posições `[kq_eff;kq_eff;1]` (LIMO, ganho **não** alterado pela refatoração) seguidas de `gamma·K_shape_diag` / `L_shape_diag` (forma `ρ,α,β`, soft-started) |
| [`K_shape_diag`, `L_shape_diag`](../matlab/formacao_2.m:46) | Ganho/saturação da lei cinemática sobre `ρ,α,β` — únicos valores realmente novos desta refatoração, ainda a calibrar em laboratório |
| [`cfg.kq`, `cfg.lq`](../matlab/formacao_2.m:18) | Ganho/saturação da posição do LIMO (`xf,yf`) — **inalterados**, mesmos valores de antes da refatoração |
| `kq_eff`, `lq_eff` | `kq`/`lq` escalados por `crossing_gain_scale` perto do cruzamento da lemniscata (origem), para suavizar a correção quando a trajetória se autointersecta |

## 4. Jacobiana e desvio de obstáculo

| Variável | Significado |
|---|---|
| `cluster_jacobian_inv(q)` | Jacobiana inversa 6×6: mapeia `q_dot` (cluster) para `x_dot = [x1_dot;y1_dot;z1_dot;x2_dot;y2_dot;z2_dot]` (velocidades cartesianas dos dois robôs) |
| `x_dot` | Velocidades cartesianas resultantes; `x_dot(3)` é sempre forçado a `0` (LIMO é terrestre) |
| `cluster_obstacle_nsb(...)` | Implementa NSB (Eq. 5.13): evasão de obstáculo com prioridade máxima, projeta a formação no espaço nulo da tarefa de evasão |
| [`cfg.obstacle_center`, `cfg.obstacle_radius`, `cfg.obstacle_influence_radius`](../matlab/formacao_2.m:23) | Geometria do obstáculo; raio de influência `0,50 m` (exigência do SPEC) |
| `obstacle_repulsive_gradient(...)` | Potencial repulsivo exponencial `V=e^{-[(dx/a)^n+(dy/b)^n]}`; retorna o gradiente já saturado em `cfg.obstacle_potential_vmax` |

## 5. LIMO (malha interna)

| Variável | Significado |
|---|---|
| `vd_L` | Velocidade desejada `[v;ω]`, obtida invertendo `x_dot(1:2)` via `A1inv` e saturando em `cfg.v_max`/`cfg.w_max` |
| `v_limo_state` | Estado de velocidade real do LIMO, integrado por `limo_inner_loop` (compensador dinâmico, Eq. 4.44, com [`cfg.theta_limo`](../matlab/formacao_2.m:20)) |
| `cmdL` | Comando final enviado (`= v_limo_state`), publicado em `Linear.X`/`Angular.Z` |

## 6. Bebop (malha externa + interna)

| Variável | Significado |
|---|---|
| `dx2` | Velocidade cartesiana desejada do Bebop no mundo — vem de `x_dot(4:6)` na fase ativa, ou da lei local `Ls_B·tanh(...)` na preparação; saturada em `vd_B_max` (nível 1) |
| `p2d` | Posição-alvo do Bebop, só para registro/plot: `[poi;0] + cluster_offset(ρ_f,α_f,β_f)` |
| [`Kp_B`, `Ls_B`](../matlab/formacao_2.m:50) | Ganho/saturação usados **só na fase de preparação** (antes, usados também na fase ativa — agora substituídos por `K_shape_diag`/`L_shape_diag`) |
| `vd_B` | Velocidade desejada no referencial do corpo do Bebop: `[A2inv·dx2; 0]` — componente de yaw sempre zero (orientação não é controlada) |
| `vB_meas` | Velocidade medida do Bebop (diferença finita da pose, rotacionada para o corpo) |
| `dvd_B` | Aceleração desejada, filtrada (passa-baixa `cfg.dvd_B_filter_alpha`) e saturada (`cfg.dvd_B_max`); resetada a zero na transição preparação→formação |
| [`KD_B`](../matlab/formacao_2.m:55), `KD_B_eff` | Ganho de amortecimento do compensador dinâmico (`= gamma·KD_B`) |
| [`f1`, `f2`](../matlab/formacao_2.m:56) | Parâmetros do modelo identificado `v̇ = f1·u - f2·v` do Bebop 2 |
| `cmdB_raw` | Comando bruto do compensador dinâmico: `f1\(dvd_B + KD_B_eff·(vd_B-vB_meas) + f2·vB_meas)` |
| [`cmdB_max`](../matlab/formacao_2.m:61) | Saturação nível 2 (comando final), por eixo |
| `cmdB` | Comando efetivamente enviado, após saturação nível 2 e rate limiter |
| [`cfg.cmdB_rate_max`](../matlab/formacao_2.m:68) | Taxa máxima de variação do comando; aplicada sobre `cmdB_prev` (último comando **realmente enviado**), o que funciona como anti-windup |

## 7. Segurança (independente do controlador)

| Variável | Significado |
|---|---|
| [`cfg.optitrack_timeout_s`](../matlab/formacao_2.m:160) | Watchdog: aborta se a pose não atualizar por mais que esse tempo |
| [`cfg.bebop_limite_xy`, `cfg.bebop_limite_z`](../matlab/formacao_2.m:69) | Parede virtual (geofence): aborta se o Bebop sair da caixa `±1,8 m` em xy ou `z>1,8 m` |
| `BTN_STOP`, `cfg.btn_start` | Botões do joystick: parada de emergência e confirmação de início (decolagem manual) |

## 8. Histórico (`H`) e auditoria

`H.*` acumula, por iteração: tempo, PoI, referência, posição/alvo/erro do Bebop, comandos de LIMO e Bebop, distância ao obstáculo, flags de saturação e `gamma`. Usado para os gráficos finais e para `results/formacao_2/audit_formacao_*.txt` (via `registrar_auditoria`), que registra também as matrizes cinemáticas e os resíduos do modelo dinâmico do Bebop a cada `cfg.audit_period` segundos.

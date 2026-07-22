# Variáveis do `matlab/limo_bebop.m`

Glossário com fórmula de cada variável calculada, organizado na ordem em que aparecem no código. Referências apontam para [`matlab/limo_bebop.m`](../matlab/limo_bebop.m).

## 1. Tempo e limites do robô

| Variável | Fórmula/valor | Significado |
|---|---|---|
| `cfg.T` | `1/30 s` | período do loop (30 Hz) |
| `cfg.Tsim` | `120 s` | duração da fase ativa (sem contar preparação) |
| `cfg.takeoff_wait_s` | `5 s` | pausa após enviar takeoff, antes de controlar |
| `cfg.preparation_time_s` | `10 s` | duração da fase de preparação (só existe se `MODO_BEBOP=='voo'`) |
| `cfg.v_max`, `cfg.w_max` | `0,30 m/s`, `1,20 rad/s` | limites físicos de velocidade do LIMO |
| `cfg.a1` | `0,10 m` | distância do PoI à frente do centro do LIMO |

## 2. LIMO — ganhos e modelo dinâmico

| Variável | Fórmula/valor | Significado |
|---|---|---|
| `cfg.kq`, `cfg.lq` | `0,8`, `0,30` | ganho proporcional / saturação da lei `tanh` do laço externo |
| `cfg.theta_limo` | `[0,1521; 0,0953; 0,0031; 0,9840; -0,0451; 1,6422]` | parâmetros identificados do modelo dinâmico (massa, atrito, acoplamento) |
| `cfg.kd_limo` | `4,0` | ganho de amortecimento do compensador dinâmico |

## 3. Formação (Bebop em relação ao LIMO)

`beta_f` = elevação, `alpha_f` = azimute, a partir do eixo X global:
```
offset_f = rho_f · [cos(beta_f)·cos(alpha_f);  cos(beta_f)·sin(alpha_f);  sin(beta_f)]
```

| Variável | Fórmula/valor | Significado |
|---|---|---|
| `TRAJ` | `0` ou `1` | `0`: alvo fixo `cfg.p2d_teste`; `1`: lemniscata |
| `cfg.p2d_teste` | `[0,75; 0,00; 1,00]` m | alvo fixo do Bebop quando `TRAJ=0` |
| `rho_f` | `1,5 m` | distância LIMO–Bebop |
| `alpha_f` | `0°` | azimute |
| `beta_f` | `60°` (`π/3`) | elevação — `90°` seria a singularidade (Bebop exatamente acima) |
| `offset_f` | `[0,750; 0,000; 1,299] m` | deslocamento resultante (fórmula acima) |

## 4. Bebop — ganhos, guinada e modelo dinâmico

Modelo identificado: `v̇ = f1·u − f2·v` (Tabela 3.1 do livro-texto, valores exatos do Bebop 2).

| Variável | Fórmula/valor | Significado |
|---|---|---|
| `Kp_B` | `diag(1,0; 1,0; 1,2)` | ganho de posição do laço externo (dentro do `tanh`) |
| `Ls_B` | `diag(0,6; 0,6; 0,6)` | saturação da lei `tanh` (teto da correção de posição, m/s) |
| `KD_B` | `diag(2,5; 2,5; 2,0; 5,0)` | ganho de amortecimento do compensador dinâmico (último elemento = guinada) |
| `cfg.yaw_d_B` | `0°` | guinada desejada (alinhado ao eixo X global) |
| `cfg.k_yaw_B` | `1,0 /s` | ganho do controle cinemático de guinada |
| `cfg.wd_B_max` | `0,6 rad/s` | saturação da taxa de guinada desejada |
| `f1` | `diag(0,8417; 0,8354; 3,966; 9,8524)` | ganho de entrada do modelo (aceleração por unidade de comando) |
| `f2` | `diag(0,18227; 0,17095; 4,001; 4,7295)` | amortecimento/arrasto do modelo |
| `cmdB_max` | `[0,5; 0,5; 0,3; 0,5]` | saturação final do comando (m/s, m/s, m/s, rad/s) |

## 5. Obstáculo (desvio em espaço nulo)

Potencial repulsivo exponencial, ativo só se `distância < obstacle_influence_radius`:
```
V(offset) = exp(-((dx/a)^n + (dy/b)^n)),   grad = ∇V escalado por obstacle_potential_gain, saturado em obstacle_potential_vmax
```
Projeção NSB (só a componente radial do rastreamento é removida, a tangencial sobrevive):
```
task_dir = grad / ‖grad‖
vel_poi ← grad + (I − task_dir·task_dirᵀ) · vel_poi
```

| Variável | Valor | Significado |
|---|---|---|
| `cfg.obstacle_center` | `[-0,20; 0,425]` | centro do obstáculo |
| `cfg.obstacle_radius` | `0,15 m` | raio físico |
| `cfg.obstacle_influence_radius` | `0,25 m` | raio a partir do qual a repulsão é considerada |
| `cfg.obstacle_potential_gain/exponent/shape_a/shape_b/vmax` | — | parâmetros da forma do potencial (ver fórmula acima) |

## 6. Segurança

| Variável | Valor | Significado |
|---|---|---|
| `cfg.bebop_limite_x_pos/x_neg/y_pos/y_neg/z_pos` | `±1,8 m` | parede virtual, limite por direção |
| `cfg.optitrack_timeout_s` | `0,5 s` | watchdog — aborta se a pose parar de atualizar |

## 7. Variáveis calculadas a cada iteração do loop

| Variável | Fórmula | Significado |
|---|---|---|
| `poi` | `[x1 + a1·cosψ1;  y1 + a1·sinψ1]` | ponto de interesse do LIMO (unicycle estendido) |
| `em_preparacao` | `t < preparation_time_s` | fase de preparação (LIMO parado) |
| `t_traj` | `max(0, t - preparation_time_s)` | tempo desde o início da trajetória ativa |
| `[vd_L, ref_xy, vel_poi_ff]` | `limo_reference_controller(...)` | ver §8 |
| `cmdL` | `= v_limo_state` (após `limo_inner_loop`) | comando final `[v; ω]` enviado ao LIMO |
| `p2d` | `[poi; z1] + offset_f` (ou `cfg.p2d_teste` se `TRAJ=0` fora da preparação) | alvo de posição do Bebop |
| `p2` | `[x2; y2; z2]` | posição medida (ou virtual) do Bebop |
| `dx2` | `[vel_poi_world; 0] + Ls_B·tanh(Ls_B⁻¹·Kp_B·(p2d−p2))` | velocidade cartesiana desejada do Bebop no mundo (feedforward + correção saturada) |
| `A2inv` | rotação mundo→corpo por `ψ2` | converte velocidade do mundo para o corpo do Bebop |
| `vB_meas` | diferença finita de `p2`/`ψ2` (ou estado da planta virtual) | velocidade medida do Bebop no corpo |
| `w_d_B` | `k_yaw_B · wrap_π(yaw_d_B − ψ2)`, saturado em `wd_B_max` | taxa de guinada desejada (regula `ψ2` para `0°`) |
| `vd_B` | `[A2inv·dx2;  w_d_B]` | velocidade desejada do Bebop no corpo (4 componentes) |
| `dvd_B` | `(vd_B − vd_B_ant) / dt` (zerado no 1º ciclo e na transição preparação→formação) | aceleração desejada (diferença finita bruta, sem filtro) |
| `cmdB_raw` | `f1⁻¹·(dvd_B + KD_B·(vd_B−vB_meas) + f2·vB_meas)` | comando bruto do compensador dinâmico |
| `cmdB` | `saturar(cmdB_raw, cmdB_max)` | comando final enviado ao Bebop |

## 8. `limo_reference_controller` (malha externa do LIMO)

```
err_xy = ref_xy − poi
vel_poi = ref_xy_dot + lq·tanh((kq/lq)·err_xy)     ← feedforward + correção saturada
vel_poi ← desvio de obstáculo (NSB, §5), se cfg.use_obstacle_avoidance
A1inv = [cosψ, sinψ; −sinψ/a1, cosψ/a1]
v_d = saturar(A1inv · vel_poi,  v_max, w_max)
```
`ref_xy`/`ref_xy_dot` vêm de `lemniscata_reference(t)`: `xd=0,75·sin(2πt/40)`, `yd=0,75·sin(4πt/40)`.

## 9. `limo_inner_loop` (compensador dinâmico do LIMO)

```
Y1 = [u, 0, w², 0, 0, 0; 0, w, 0, u, uw, w]
u_control = Y1·theta_limo + kd_limo·(v_d − v)
M1·v̇ = u_control − C1·v     (M1, C1 derivados de theta_limo)
v ← saturar(v + T·v̇,  v_max, w_max)
```

## 10. Estado que persiste entre iterações

| Variável | Papel |
|---|---|
| `v_limo_state` | velocidade real (filtrada pela dinâmica) do LIMO, realimentada no próximo `limo_inner_loop` |
| `vd_B_ant` | `vd_B` do ciclo anterior, usado para calcular `dvd_B` |
| `poseB_ant`, `poseB_psi_ant` | pose do Bebop no ciclo anterior, usados para calcular `vB_meas` por diferença finita |
| `em_preparacao_ant` | detecta a transição preparação→formação (reset de `dvd_B` nesse instante) |
| `virtual_B` (struct) | só existe se `MODO_BEBOP=='off'`: posição, guinada e velocidade da planta virtual do Bebop |

## 11. Histórico (`H`) e auditoria

`H.*` guarda, por iteração: tempo, PoI, referência, posição/alvo/erro do Bebop, comandos, distância ao obstáculo e flag de saturação — usado nos gráficos finais e no resumo de `results/limo_bebop/audit_*.txt`. `registrar_auditoria` grava, a cada `cfg.audit_period` segundos, um resumo compacto (alvo, erro, `vd_B`, `vB_meas`, `cmdB_raw`, `cmdB`) — sem as matrizes/resíduos que a versão anterior tinha (cortados para leitura mais rápida).

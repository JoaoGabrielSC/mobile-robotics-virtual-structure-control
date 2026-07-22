# Equações do controlador — LIMO + Bebop 2

Referência teórica de cada equação usada em [`matlab/limo_bebop.m`](../matlab/limo_bebop.m), com a origem (livro-texto, quando aplicável) e o significado físico. Cada equação vem seguida de um "Onde" com o nome de cada variável, seu significado e o nome correspondente no código. Livro-texto: *Control of Ground and Aerial Robots* (Sarcinelli-Filho & Carelli, 2023).

## 1. Cinemática estendida do LIMO (unicycle com ponto de interesse)

O LIMO é um robô diferencial (não-holonômico): não pode se mover lateralmente. Para contornar essa restrição, controla-se um ponto deslocado à frente do eixo das rodas, não o centro do robô:

$$\text{PoI} = \begin{bmatrix} x_1 + a_1 \cos\psi_1 \\ y_1 + a_1 \sin\psi_1 \end{bmatrix}$$

$$\mathbf{A}_1^{-1} = \begin{bmatrix} \cos\psi & \sin\psi \\ -\sin\psi/a_1 & \cos\psi/a_1 \end{bmatrix}, \qquad \begin{bmatrix}v\\ \omega\end{bmatrix} = \text{saturar}\big(\mathbf{A}_1^{-1} \cdot \text{vel}_{\text{poi}},\ v_{max},\ \omega_{max}\big)$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `x1, y1` | posição do centro do LIMO no mundo [m], lida do OptiTrack | `x1`, `y1` |
| `ψ1` | guinada (yaw) do LIMO [rad] | `psi1` |
| `a1` | distância do ponto de interesse à frente do centro do robô [m] | `cfg.a1` |
| `PoI` | ponto de interesse controlado (não é o centro do robô) | `poi` |
| `vel_poi` | velocidade cartesiana desejada do PoI no mundo (vem da Seção 2) | `vel_poi` (dentro de `limo_reference_controller`) |
| `A1⁻¹` | matriz de cinemática inversa (mundo → `[v,ω]` do robô) | `A1inv` |
| `v, ω` | velocidade linear/angular comandada ao LIMO, antes da compensação dinâmica | `v_d` (saída de `limo_reference_controller`) |
| `v_max, ω_max` | limites físicos do robô [m/s, rad/s] | `cfg.v_max`, `cfg.w_max` |

## 2. Lei de controle cinemático saturada (laço externo)

Forma geral usada tanto para o LIMO quanto para o Bebop: feedforward da referência + correção proporcional saturada por `tanh`, para que o comando nunca exploda mesmo com erro grande:

$$\dot{q}_r = \dot{q}_d + L \tanh\!\left(L^{-1} K\, \tilde q\right), \qquad \tilde q = q_d - q$$

**Onde (forma geral):**
| Símbolo | Significado |
|---|---|
| `q` | posição atual (do LIMO ou do Bebop) |
| `q_d` | posição desejada (referência) |
| `q̇_d` | velocidade desejada da referência (feedforward) |
| `q̃` | erro de posição (`q_d - q`) |
| `K` | ganho — inclinação da correção perto do erro zero (região quase-linear do `tanh`) |
| `L` | saturação — teto de velocidade que a correção pode pedir, mesmo com erro grande |
| `q̇_r` | velocidade de referência resultante (feedforward + correção saturada) |

`K` e `L` fazem juntos a mesma função da Eq. (5.7)/(5.12) do livro-texto (Seção 5.5, "Control in the Cluster Space").

**No LIMO** (função `limo_reference_controller`):
$$\text{vel}_{\text{poi}} = \dot{r}ef_{xy} + l_q \tanh\!\left(\frac{k_q}{l_q}\, \text{err}_{xy}\right)$$

| Símbolo | Significado | No código |
|---|---|---|
| `ref_xy` | posição de referência da lemniscata (Seção 3) | `ref_xy` |
| `ref_xy_dot` | velocidade de referência (feedforward) | `ref_xy_dot` |
| `err_xy` (`= ref_xy - poi`) | erro de posição do PoI | `err_xy` |
| `k_q, l_q` | ganho / saturação | `cfg.kq = 0,8`, `cfg.lq = 0,30` |
| `vel_poi` | velocidade cartesiana desejada do PoI no mundo | `vel_poi` |

**No Bebop**: `K = K_{p,B}`, `L = L_{s,B}`, aplicada ao erro de posição `p2d - p2`, somada ao feedforward `vel_poi_world` (a velocidade do PoI do LIMO, para o Bebop acompanhar o *movimento* do LIMO, não só sua posição):

$$dx_2 = \begin{bmatrix}\text{vel}_{\text{poi,world}}\\ 0\end{bmatrix} + L_{s,B} \tanh\!\left(L_{s,B}^{-1} K_{p,B} (p_{2d} - p_2)\right)$$

| Símbolo | Significado | No código |
|---|---|---|
| `vel_poi_world` | feedforward: velocidade do PoI do LIMO no mundo (`[0;0]` se `TRAJ=0`) | `vel_poi_world` (vem de `vel_poi_ff`, saída de `limo_reference_controller`) |
| `p2d` | posição-alvo do Bebop (Seção 5) | `p2d` |
| `p2` | posição medida (ou virtual) do Bebop `[x2;y2;z2]` | `p2` |
| `K_{p,B}` | ganho de posição do Bebop | `Kp_B = diag(1,0; 1,0; 1,2)` |
| `L_{s,B}` | saturação da correção do Bebop [m/s] | `Ls_B = diag(0,6; 0,6; 0,6)` |
| `dx2` | velocidade cartesiana desejada do Bebop no mundo | `dx2` |

## 3. Referência: Lemniscata de Bernoulli

$$x_d(t) = 0{,}75 \sin\!\left(\frac{2\pi t}{40}\right), \qquad y_d(t) = 0{,}75 \sin\!\left(\frac{4\pi t}{40}\right)$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `t` | tempo desde o início da trajetória ativa (não desde o início do script) | `t_traj` |
| `x_d, y_d` | posição de referência no instante `t` | `ref_xy` (saída de `lemniscata_reference`) |
| `ẋ_d, ẏ_d` | velocidade de referência, derivada analítica (feedforward exato) | `ref_xy_dot` |

Período de 40 s, amplitude de 0,75 m, conforme o enunciado do projeto.

## 4. Espaço nulo (NSB) para desvio de obstáculo

Estrutura de prioridade: a tarefa de evasão do obstáculo tem prioridade máxima; o rastreamento da trajetória só atua na direção que **não** interfere na evasão (Cap. 5, "Multiple Objectives", Eq. 5.13 do livro):

$$\dot{x} = \dot{x}_1 + \left(I - J_1^\dagger J_1\right) \dot{x}_2$$

No código, a tarefa 1 é escalar (a distância ao obstáculo), sua Jacobiana é a direção do gradiente repulsivo:

$$\text{vel}_{\text{poi}} \leftarrow \nabla U + \left(I - \text{task\_dir}\cdot\text{task\_dir}^T\right) \cdot \text{vel}_{\text{poi}}$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `ẋ_1` | velocidade da tarefa 1 (evasão) — aqui, `∇U` | não nomeada isoladamente, é `grad` |
| `ẋ_2` | velocidade da tarefa 2 (rastreamento) — o `vel_poi` já calculado na Seção 2 | `vel_xy` (parâmetro de entrada de `apply_obstacle_null_space_xy`) |
| `J1` | Jacobiana da tarefa 1 (aqui, um vetor-linha `1×2` = direção do gradiente) | implícito em `task_dir` |
| `J1†` | pseudo-inversa de `J1` | implícito (`task_dir` como coluna) |
| `∇U` (`grad`) | gradiente do potencial repulsivo, já saturado em `v_max^{obs}` | `grad`, de `obstacle_repulsive_gradient` |
| `task_dir` | direção unitária do gradiente (`grad/‖grad‖`) | `task_dir` |
| `(I - task_dir·task_dirᵀ)` | projetor no espaço nulo: remove só a componente radial | `null_projector` |

**Potencial repulsivo** (forma exponencial, só ativo dentro do raio de influência):

$$U(\Delta x, \Delta y) = \eta \cdot \exp\!\left(-\left[\left(\frac{\Delta x}{a}\right)^n + \left(\frac{\Delta y}{b}\right)^n\right]\right)$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `Δx, Δy` | vetor do centro do obstáculo até o PoI (`poi - obstacle_center`) | `offset`, `dx = offset(1)`, `dy = offset(2)` |
| `η` | ganho (amplitude) do potencial | `cfg.obstacle_potential_gain = 0,80` |
| `a, b` | escala da "largura" do potencial em cada eixo | `cfg.obstacle_potential_shape_a/b` (`[]` → usa `influence_radius - radius`) |
| `n` | expoente (quão "íngreme" é a subida do potencial) | `cfg.obstacle_potential_exponent = 4` |
| raio físico do obstáculo | distância mínima antes de considerar colisão | `cfg.obstacle_radius = 0,15 m` |
| raio de influência | distância a partir da qual a repulsão é ignorada | `cfg.obstacle_influence_radius = 0,25 m` |
| teto do gradiente | valor máximo que `∇U` pode assumir | `cfg.obstacle_potential_vmax = 0,80 m/s` |

## 5. Geometria da formação (Bebop em relação ao LIMO)

Com `β_f` = elevação e `α_f` = azimute (a partir do eixo X global, convenção adotada neste projeto):

$$\text{offset}_f = \rho_f \begin{bmatrix} \cos\beta_f \cos\alpha_f \\ \cos\beta_f \sin\alpha_f \\ \sin\beta_f \end{bmatrix}, \qquad p_{2d} = \begin{bmatrix}x_{poi}\\ y_{poi}\\ z_1\end{bmatrix} + \text{offset}_f$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `ρ_f` | distância entre LIMO e Bebop na formação [m] | `rho_f = 1,5` |
| `α_f` | azimute do Bebop em relação ao LIMO [rad] | `alpha_f = 0` |
| `β_f` | elevação do Bebop em relação ao LIMO [rad] — `90°` é a singularidade, evitada | `beta_f = π/3` (60°) |
| `offset_f` | deslocamento cartesiano resultante `[Δx;Δy;Δz]` | `offset_f = [0,750; 0,000; 1,299]` m |
| `x_poi, y_poi` | posição horizontal do PoI do LIMO | `poi(1)`, `poi(2)` |
| `z1` | altura medida do LIMO (não assumida zero) | `z1` |
| `p2d` | posição-alvo do Bebop | `p2d` |

Se `TRAJ=0` fora da fase de preparação, `p2d` usa um alvo fixo (`cfg.p2d_teste`) em vez dessa fórmula.

## 6. Controle cinemático de guinada do Bebop

Regula a guinada do Bebop para um valor fixo (alinhado ao eixo X global) com uma lei proporcional simples, saturada:

$$\dot\psi_{2,d} = \text{saturar}\big(k_{yaw} \cdot \text{wrap}_\pi(\psi_{d} - \psi_2),\ \dot\psi_{max}\big)$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `ψ_d` | guinada desejada do Bebop [rad] | `cfg.yaw_d_B = 0` |
| `ψ2` | guinada medida do Bebop | `psi2` |
| `k_yaw` | ganho proporcional [1/s] | `cfg.k_yaw_B = 1,0` |
| `ψ̇_max` | saturação da taxa de guinada [rad/s] | `cfg.wd_B_max = 0,6` |
| `wrap_π(·)` | função que traz o ângulo para `[-π, π]`, tratando a descontinuidade em `±π` | `wrap_pi(...)` |
| `ψ̇_{2,d}` | taxa de guinada desejada (4ª componente de `vd_B`) | `w_d_B` |

## 7. Compensação dinâmica — princípio geral (feedback linearization)

Os laços cinemáticos acima calculam a velocidade *desejada*, mas o robô não a atinge instantaneamente (tem massa/inércia). O compensador dinâmico inverte o modelo físico conhecido do robô para calcular o comando que produz exatamente a aceleração necessária (Seção 4.11, "Cascade Dynamic Compensation", Eq. 4.44):

$$v_r = H\left[\dot v_d + K_D \tilde v\right] + C(v)\, v, \qquad \tilde v = v_d - v$$

$$\dot{\tilde v} + K_D \tilde v = 0 \quad\Rightarrow\quad \tilde v(t) \to 0 \text{ quando } t\to\infty$$

**Onde (forma geral):**
| Símbolo | Significado |
|---|---|
| `H` | matriz de inércia (ou seu equivalente no modelo usado) |
| `C(v)` | termos de Coriolis/centrípeto/atrito, dependentes da velocidade |
| `v_d` | velocidade desejada (vem do laço cinemático) |
| `v` | velocidade real, medida |
| `v̇_d` | aceleração desejada (feedforward) |
| `K_D` | ganho de amortecimento (positivo definido) |
| `ṽ` | erro de velocidade |
| `v_r` | comando final, calculado invertendo o modelo |

A convergência `ṽ → 0` é provada formalmente por Lyapunov no livro (Seção 4.10, Eqs. 4.41-4.43) — não é uma receita empírica, é uma técnica com estabilidade garantida.

## 8. Compensação dinâmica do LIMO (aplicação específica)

$$Y_1 = \begin{bmatrix}u & 0 & \omega^2 & 0 & 0 & 0\\ 0 & \omega & 0 & u & u\omega & \omega\end{bmatrix}, \qquad u_{control} = Y_1 \theta_{LIMO} + k_{d,LIMO}\,\tilde v$$

$$M_1 \dot v = u_{control} - C_1 v, \qquad v \leftarrow \text{saturar}(v + T\dot v,\ v_{max},\ \omega_{max})$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `u, ω` | velocidade linear/angular real do LIMO (estado interno) | `u_real = v_state(1)`, `w_real = v_state(2)` |
| `Y1` | matriz de regressão (reorganiza `u,ω` para multiplicar por `θ_LIMO`) | `Y1` |
| `θ_LIMO` | 6 parâmetros dinâmicos identificados (massa, atrito, acoplamento) | `cfg.theta_limo` |
| `k_{d,LIMO}` | ganho de amortecimento | `cfg.kd_limo = 4,0` |
| `M1, C1` | matrizes de massa e Coriolis/acoplamento, derivadas de `θ_LIMO` | `M1`, `C1` |
| `T` | período do loop (integração de Euler) | `cfg.T` |
| `v` (estado) | `[v;ω]` real, realimentado no próximo ciclo | `v_state` (= `v_limo_state` fora da função) |

## 9. Compensação dinâmica do Bebop (aplicação específica)

Modelo identificado, linear e desacoplado por eixo (Eq. 3.30 do livro):

$$\dot v = f_1 u - f_2 v$$

Aplicando a Eq. 4.44 a esse modelo (`H → f1⁻¹`, `C(v)v → f2 v`), obtém-se a lei de controle (Eq. 4.47):

$$u = f_1^{-1}\left(\dot v_d + K_D(v_d - v) + f_2 v\right)$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `f1` | ganho de entrada do modelo (aceleração por unidade de comando) | `f1 = diag(0,8417; 0,8354; 3,966; 9,8524)` |
| `f2` | amortecimento/arrasto do modelo | `f2 = diag(0,18227; 0,17095; 4,001; 4,7295)` |
| `v` | velocidade real do Bebop no corpo `[vx;vy;vz;ψ̇]`, medida | `vB_meas` |
| `v_d` | velocidade desejada do Bebop no corpo (saída da Seção 2 + 7, rotacionada) | `vd_B = [A2inv*dx2; w_d_B]` |
| `v̇_d` | aceleração desejada — diferença finita bruta de `v_d`, resetada no 1º ciclo e na transição preparação→formação | `dvd_B` |
| `K_D` | ganho de amortecimento do compensador | `KD_B = diag(2,5; 2,5; 2,0; 5,0)` |
| `u` | comando bruto calculado (antes da saturação final) | `cmdB_raw` |
| `A2⁻¹` | rotação mundo→corpo do Bebop, por `ψ2` | `A2inv` |

Valores de `f1`/`f2` idênticos à Tabela 3.1 do livro (coluna Bebop 2) — conferido dígito a dígito.

## 10. Saturação

$$\text{comando enviado} = \text{saturar}(u,\ \text{cmdB}_{max})$$

**Onde:**
| Símbolo | Significado | No código |
|---|---|---|
| `u` (`cmdB_raw`) | comando bruto, calculado na Seção 10 | `cmdB_raw` |
| `cmdB_max` | teto do comando final por eixo `[vx;vy;vz;ψ̇]` — vertical menor porque satura fisicamente mais rápido | `cmdB_max = [0,5; 0,5; 0,3; 0,5]` |
| comando final | o que é de fato enviado ao Bebop | `cmdB` |

**Nota de precisão**: no LIMO, `v_d` (Seção 1) e `v_state` (Seção 9) *são* explicitamente saturados em `v_max`/`w_max` — duas camadas reais. No Bebop, hoje só existe **uma** saturação de verdade, a desta seção (`cmdB_max`, sobre o comando final). O termo `Ls_B` (Seção 2) limita a *parcela de correção* dentro do `tanh`, mas a soma com o feedforward (`dx2` completo) não tem um teto explícito separado antes do compensador dinâmico — diferente do que uma versão anterior deste documento afirmava.

## 11. Camadas de segurança (fora da malha de controle)

Não são equações de controle, mas redes de segurança independentes que interrompem o experimento se algo sair do esperado.

| Mecanismo | Condição de disparo | No código |
|---|---|---|
| Parede virtual | `p2` sai da caixa `[x_{neg},x_{pos}]×[y_{neg},y_{pos}]×(-\infty,z_{pos}]` | `cfg.bebop_limite_x_pos/x_neg/y_pos/y_neg/z_pos` |
| Watchdog do OptiTrack | pose não atualiza por mais de `0,5` s | `cfg.optitrack_timeout_s` |

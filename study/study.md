# Guia de conceitos — como ler e auditar `formacao_2.m`

Este documento explica, com matemática e intuição, **todo conceito necessário para entender e revisar** [`matlab/formacao_2.m`](../matlab/formacao_2.m) e o que foi alterado em relação ao original (ver [`docs/changes_formation.md`](../docs/changes_formation.md) para o diff comentado).

Ele é complementar a [`study/README.md`](README.md) (trilha de exercícios práticos) e [`study/study.m`](study.m) (script executável). Aqui o foco é **entender o "porquê" de cada bloco matemático**, não rodar código.

Convenção: `[formacao_2.m:N]` aponta para a linha N do arquivo atual.

---

## Índice

1. Ferramentas matemáticas básicas
2. Referenciais, pose e orientação
3. Cinemática do unicíclo e o ponto de interesse (PoI)
4. Geração de trajetória — lemniscata
5. Estrutura Virtual (Virtual Structure)
6. Controle cinemático (outer loop) e saturação suave (`tanh`)
7. Controle dinâmico / compensação de modelo (inner loop, feedback linearization)
8. Diferenciação numérica e por que ela é perigosa
9. NSB — Null Space Based control e campos potenciais
10. Robustez de controle: soft start, saturação em dois níveis, rate limiter, anti-windup, filtro passa-baixa
11. Segurança de sistema: geofencing e watchdog de sensor
12. ROS: o mínimo necessário para ler o código

---

## 1. Ferramentas matemáticas básicas

Tudo no código é álgebra linear simples aplicada em malha fechada a 30 Hz.

- **Vetor coluna** representa um estado ou comando: `p2 = [x2; y2; z2]` [formacao_2.m:295](../matlab/formacao_2.m).
- **Norma Euclidiana** `norm(v)` mede "quão longe" ou "quão rápido": usada em `dist_obs = norm(poi - cfg.obstacle_center)` para saber se está perto do obstáculo.
- **Produto matricial** transforma um vetor de um espaço para outro — é assim que se troca de referencial (ver §2) e de coordenadas (mundo↔corpo).
- **Matriz diagonal** (`diag([...])`) representa ganhos independentes por eixo: `Kp_B = diag([1.0,1.0,1.2])` significa "corrija x com ganho 1.0, y com 1.0, z com 1.2", sem acoplamento entre eixos.
- **Saturação** `saturar(u,umax) = max(min(u,umax),-umax)` — um clamp. Importante: saturar melhora segurança (nunca manda mais que o fisicamente seguro) mas **piora desempenho** (se o controlador realmente precisava de mais autoridade, ele não terá). Esse é o trade-off central de todo o documento de correções.
- **Integração de Euler** `estado = estado + dt*derivada` — a forma mais simples de simular ou de manter um estado interno (usada em `limo_inner_loop` e em `avancar_bebop_virtual`). Erro de integração cresce com `dt`; por isso o laço roda a 30 Hz (`dt≈0.033s`) e não mais devagar.

**Pergunta para se testar:** por que `saturar` sozinha não resolve o problema de "comando agressivo", mesmo limitando o valor máximo? (Resposta: porque não limita a *taxa de variação* — ver §10.)

---

## 2. Referenciais, pose e orientação

- **Pose do LIMO**: `[x1; y1; psi1]` (planar — o LIMO anda no chão).
- **Pose do Bebop**: `[x2; y2; z2; psi2]` (o drone tem altura).
- `psi` (yaw) vem de quaternion → Euler: `ler_pose` [formacao_2.m:485](../matlab/formacao_2.m) usa `quat2eul` sobre `[w,x,y,z]` e pega `eul(1)`.

### Por que precisamos de yaw mesmo "não controlando orientação"

O projeto pede para **não controlar** o yaw do Bebop (`vd_B(4) = 0` sempre, [formacao_2.m:363](../matlab/formacao_2.m)) — mas ainda assim `psi2` é usado, porque a **velocidade desejada é calculada no referencial mundo** (`dx2`) e o **comando enviado ao drone é no referencial do corpo** (`cmdB`). A rotação `A2inv` faz essa conversão:

```matlab
A2inv = [cos(psi2), sin(psi2), 0;
        -sin(psi2), cos(psi2), 0;
         0, 0, 1];
```

Isto é uma matriz de rotação (transposta = inversa, pois é ortogonal): `v_corpo = A2inv * v_mundo`. Se você ignorasse o yaw atual do drone nessa conversão, um comando "vá para frente" calculado no mundo sairia errado sempre que o drone estivesse girado — o drone iria para o lado, não para onde deveria.

**Distinção chave**: "não controlar orientação" = não há termo de erro de yaw na malha de controle (`ψ_desejado - ψ_medido` não aparece). Mas o yaw *medido* ainda é usado passivamente, só para as rotações de referencial.

---

## 3. Cinemática do unicíclo e o ponto de interesse (PoI)

O LIMO é modelado como **unicíclo**: duas entradas, `v` (velocidade linear) e `w` (velocidade angular), e três estados `(x,y,ψ)`. Um unicíclo **não pode se mover lateralmente** instantaneamente (restrição não-holonômica) — por isso não dá para simplesmente mandar `(vx,vy)` diretamente.

### O truque do ponto de interesse (PoI)

Em vez de controlar o centro do robô, controla-se um ponto deslocado `a` metros à frente:

```
poi = [x1 + a*cos(psi1); y1 + a*sin(psi1)]     [formacao_2.m:271]
```

Esse truque (comum em unicycles, "input-output linearization via offset point") transforma o sistema não-holonômico numa relação **linear e invertível** entre velocidade cartesiana desejada no PoI e `(v,w)`:

```
[vel_poi] = [cos ψ, -a sin ψ] [v]
            [sin ψ,  a cos ψ] [w]
```

Invertendo (matriz `A1inv` em `limo_reference_controller` [formacao_2.m:568](../matlab/formacao_2.m)):

```
[v]  =  [ cos ψ,      sin ψ  ] [vel_poi_x]
[w]     [-sin ψ/a,   cos ψ/a ] [vel_poi_y]
```

**Por que `a` não pode ser zero**: o termo `w = (...)/a` explode quando `a→0` — pedir para o PoI se mover lateralmente com o ponto de controle *exatamente* no eixo das rodas exigiria giro infinito instantâneo. É por isso que `a1 = 0.10` (10 cm), nunca zero.

---

## 4. Geração de trajetória — lemniscata

Função `lemniscata_reference(t)` [formacao_2.m:546](../matlab/formacao_2.m):

```
x_ref(t) = 0.75 sin(2πt/40)
y_ref(t) = 0.75 sin(4πt/40)
```

Isso é uma **curva de Lissajous 1:2** (comumente chamada de lemniscata/figura-oito neste contexto), com período de 40 s.

**Por que a derivada é calculada analiticamente** (`ref_xy_dot`), em vez de por diferença finita: derivar a fórmula fechada dá um sinal exato e sem ruído, usado como *feedforward* — ver §6. Isso é preferível sempre que a trajetória é conhecida de antemão (planejada), ao contrário da *velocidade medida do robô*, que não tem fórmula fechada e precisa ser estimada por diferença finita (§8).

---

## 5. Estrutura Virtual (Virtual Structure)

**Ideia central**: tratar a formação (LIMO + Bebop) como se fosse um corpo rígido único, onde cada robô ocupa uma posição fixa relativa a um referencial "virtual" que se move com a formação.

No código, o referencial virtual é ancorado no PoI do LIMO, e a posição desejada do Bebop é dada em coordenadas esféricas de formação `(ρ, α, β)`:

```matlab
p2d = [poi(1) + rho_f*cos(alpha_f)*cos(beta_f);
       poi(2) + rho_f*sin(alpha_f)*cos(beta_f);
       z1     + rho_f*sin(beta_f)];
```

Com `rho_f=1.5, alpha_f=0, beta_f=90°`: `cos(90°)=0` (zera a contribuição em x,y) e `sin(90°)=1` (toda a distância `ρ` vai para z). Resultado: `p2d = [poi; z1+1.5]` — **o Bebop fica 1.5 m diretamente acima do PoI do LIMO**. Essa é exatamente a especificação do projeto.

**Por que usar `(ρ,α,β)` em vez de um offset `[dx,dy,dz]` direto?** Porque é mais fácil de especificar e variar formações fisicamente intuitivas (distância + dois ângulos), e generaliza para qualquer geometria de formação sem mudar a fórmula — só os três parâmetros.

---

## 6. Controle cinemático (outer loop) e saturação suave (`tanh`)

O objetivo do outer loop é: dado o erro de posição, calcular **qual velocidade** resolveria esse erro.

### Controlador do LIMO

```matlab
vel_poi = ref_xy_dot + lq*tanh((kq/lq)*err_xy)
```

Isto é um **PD-like feedforward + feedback saturado**: `ref_xy_dot` é "o que a trajetória já pede" (feedforward), e o termo `tanh` é a correção do erro, mas **limitada suavemente a ±`lq`** (em vez de crescer sem limite com o erro, como um P puro faria).

### Por que `tanh` e não saturação dura (clamp)?

- `tanh(x) ≈ x` para `x` pequeno → comporta-se como controle proporcional normal perto do equilíbrio (convergência suave, sem chattering).
- `tanh(x) → ±1` para `x` grande → satura suavemente, sem descontinuidade na derivada.
- Isso significa: erro grande → comando de velocidade limitado (não tenta corrigir tudo de uma vez); erro pequeno → resposta proporcional normal. Um clamp duro (`min/max`) faz a mesma limitação de amplitude, mas com uma "quina" — o `tanh` é diferenciável em todo lugar, o que ajuda a evitar picos na derivada calculada a jusante (`dvd_B`, ver §8).

### Controlador do Bebop (idêntica estrutura, agora com soft start — ver §10)

```matlab
dx2 = [vel_poi_world; 0] + Ls_B*tanh(Ls_B \ (Kp_B_eff*(p2d - p2)));
```

`Ls_B \ (...)` é `inv(Ls_B)*(...)` — divide antes de aplicar `tanh`, para que o "raio de ação linear" do `tanh` seja escalado corretamente por `Ls_B` (o limite de saturação). É o mesmo padrão do LIMO, generalizado para matrizes diagonais.

---

## 7. Controle dinâmico / compensação de modelo (inner loop) — feedback linearization

Depois que o outer loop decide "quero esta velocidade", o inner loop decide "que comando físico produz essa velocidade, dado como o robô realmente se comporta dinamicamente".

### Ideia de feedback linearization / dynamic inversion

Se você conhece o modelo dinâmico do robô, `v̇ = f(v,u)`, pode **inverter** esse modelo para calcular exatamente o `u` que produz a aceleração desejada. Padrão geral usado tanto no LIMO quanto no Bebop:

```
u = M⁻¹ · ( aceleração_desejada + ganho·(v_desejada - v_medida) + termos_de_compensação )
```

### No Bebop — [formacao_2.m:391](../matlab/formacao_2.m)

```matlab
cmdB_raw = f1 \ (dvd_B + KD_B_eff*(vd_B - vB_meas) + f2*vB_meas);
```

Modelo assumido: `v̇ = f1*u - f2*v` (uma planta de primeira ordem simples: o comando acelera, o arrasto/atrito freia). Invertendo para achar `u`:

```
v̇_desejado = f1*u - f2*v_medido
⟹ u = f1⁻¹ · (v̇_desejado + f2*v_medido)
```

E `v̇_desejado` não é só `dvd_B` (a derivada da trajetória) — soma-se `KD_B*(vd_B - vB_meas)`, um termo de realimentação que corrige o erro de velocidade (se o Bebop está mais lento que o desejado, pede mais aceleração). Essa combinação é **PD com compensação de dinâmica** (feedback linearization com termo integral de erro de velocidade).

### No LIMO — `limo_inner_loop` [formacao_2.m:643](../matlab/formacao_2.m)

Mesmo princípio, mas com um modelo mais rico, **linear nos parâmetros**:

```matlab
Y1 = [u_real, 0, w_real^2, 0, 0, 0;
      0, w_real, 0, u_real, u_real*w_real, w_real];
u_control = Y1*theta_limo + KD*(v_d - v_state);
v_dot = M1 \ (u_control - C1*v_state);
```

`Y1*theta` é a técnica de **identificação de parâmetros dinâmicos linear-in-parameters**: em vez de conhecer `M`, `C` diretamente, você escreve a dinâmica como uma matriz conhecida (`Y1`, o "regressor", função apenas dos estados medidos) multiplicando um vetor de parâmetros desconhecidos (`theta`, identificados previamente por experimento). `M1` (massa) e `C1` (Coriolis/centrípeta) vêm dos mesmos `theta`. Essa é uma técnica clássica de controle robótico adaptativo/baseado em modelo (ver Siciliano et al., *Robotics: Modelling, Planning and Control*, cap. 7 e 9).

**Por que isso é sensível a ruído**: o termo `KD*(v_d - v_state)` (e o equivalente `KD_B*(vd_B-vB_meas)` no Bebop) multiplica o **erro de velocidade medida**. Se a velocidade medida for ruidosa, `KD` grande amplifica esse ruído diretamente no comando — esse foi o problema central diagnosticado (ver §10 e `docs/changes_formation.md`).

---

## 8. Diferenciação numérica e por que ela é perigosa

Duas grandezas no código **não têm fórmula fechada** e precisam ser estimadas por diferença finita:

```matlab
velWB = (p2 - poseB_ant) / dt;                 % velocidade medida do Bebop
dvd_B_raw = (vd_B - vd_B_ant) / dt;             % "aceleração" do comando desejado
```

**Por que diferenciação numérica amplifica ruído**: se a posição medida tem ruído de amplitude `ε` (jitter do OptiTrack, quantização, etc.), a derivada por diferença finita tem ruído de amplitude `~2ε/dt`. Como `dt≈0.033s` é pequeno, um ruído de posição de poucos milímetros vira um ruído de velocidade considerável — e esse ruído entra diretamente no compensador dinâmico (§7), multiplicado por `KD`.

**Pico de derivada explosivo**: se o *sinal* que está sendo diferenciado tem um **salto** (descontinuidade), a diferença finita produz um pico enorme naquele instante (idealmente um impulso). No `formacao_2.m` original, `p2d` mudava de fórmula entre a fase de preparação e a formação ativa ([formacao_2.m:286-294](../matlab/formacao_2.m)) — um salto estrutural em `p2d` gera salto em `dx2`, depois em `vd_B`, e a diferença finita `dvd_B_raw` vira um pico exatamente naquele ciclo.

**Duas mitigações usadas agora** (ver §10 para os detalhes):
1. **Reset explícito** da derivada exatamente na transição de fase (`transicao_prep_formacao`), tratando aquele ciclo como "sem informação de aceleração" em vez de "aceleração enorme".
2. **Filtro passa-baixa + clamp** para atenuar ruído residual em qualquer outro instante.

---

## 9. NSB — Null Space Based control e campos potenciais

**Problema**: como fazer o LIMO seguir a trajetória *e* desviar de um obstáculo, sem que uma tarefa atrapalhe a outra de forma descontrolada?

### Campo potencial repulsivo

`obstacle_repulsive_gradient` [formacao_2.m:617](../matlab/formacao_2.m) calcula um vetor que aponta **para longe** do obstáculo, com magnitude que cresce quando você se aproxima (e satura em `obstacle_potential_vmax`). Isso é um **gradiente de uma função de potencial artificial** — a mesma ideia de "campos potenciais" clássica em robótica (Khatib, 1986): o obstáculo "empurra", o objetivo "puxa".

**Limitação clássica de campos potenciais puros**: somar ingenuamente `v_repulsao + v_atração` pode fazer as duas se cancelarem de forma ruim (mínimo local) ou a atração "vencer" perto demais do obstáculo.

### Projeção em espaço nulo (o "NSB" propriamente dito)

```matlab
task_dir = grad / norm(grad);
null_projector = eye(2) - task_dir*task_dir.';
vel_xy = grad + null_projector*vel_xy;
```

Aqui está a ideia de **prioridade de tarefas**: a tarefa de desvio (`grad`) é executada **integralmente**; a tarefa secundária (seguir a trajetória, `vel_xy`) é **projetada no espaço nulo** da primeira — ou seja, só a parte de `vel_xy` que **não interfere** na direção de desvio é somada.

`null_projector = I - d·dᵀ` (onde `d` é unitário) é a matriz de projeção ortogonal ao vetor `d`: qualquer vetor multiplicado por essa matriz perde sua componente na direção `d` e mantém só a componente perpendicular. Isso é álgebra linear pura (projeção ortogonal), mas aplicada como uma técnica de controle: **a tarefa de prioridade mais alta nunca é comprometida pela tarefa secundária**, porque a secundária só pode agir "de lado".

Esta é a essência do **NSB (Null Space Based) behavioral control**, usado em robótica de formação multi-robô (ver Antonelli & Chiaverini, *Kinematic control of platoons of autonomous vehicles*, 2006) — cada comportamento de prioridade mais baixa é projetado no núcleo (null space) do Jacobiano dos comportamentos de prioridade mais alta.

---

## 10. Robustez de controle: as modificações aplicadas

Esta seção documenta os conceitos por trás de cada correção feita em relação ao `formacao_2.m` original (detalhes de código em [`docs/changes_formation.md`](../docs/changes_formation.md)).

### 10.1 Soft start (rampa de ganho)

```matlab
gamma = min(max(t / cfg.soft_start_time_s, cfg.soft_start_gamma_min), 1);
Kp_B_eff = gamma * Kp_B;
KD_B_eff = gamma * KD_B;
```

**Conceito**: em vez de aplicar o ganho nominal desde o primeiro ciclo, ele cresce (linearmente, aqui) de um valor mínimo até o valor nominal ao longo de `soft_start_time_s`. É análogo ao *gain scheduling* / *bumpless transfer* usado em sistemas de controle industriais ao trocar de modo de operação — evita um "chute" de comando no instante em que a malha é fechada com erro inicial grande.

### 10.2 Saturação em dois níveis

- **Nível 1** — na *velocidade desejada* (`dx2`, antes de virar comando): garante que o outer loop nunca *peça* algo irreal.
- **Nível 2** — no *comando final* (`cmdB`, depois do compensador dinâmico): garante que, mesmo que o inner loop calcule algo estranho (por exemplo por um pico de `dvd_B` residual), o valor **enviado ao driver** nunca ultrapasse o envelope físico seguro.

Ter os dois é redundante por design: cada nível protege contra uma falha diferente na cadeia.

### 10.3 Rate limiter (limitador de taxa de variação)

```matlab
delta_max = cfg.cmdB_rate_max * dt;
delta_cmd = max(min(cmdB_satN2 - cmdB_prev, delta_max), -delta_max);
cmdB = cmdB_prev + delta_cmd;
```

**Conceito**: limitar não o *valor* do comando, mas o quanto ele pode **mudar por ciclo**. Isso é um filtro de "slew rate" — comum em eletrônica de potência e em controle de atuadores físicos, onde uma mudança instantânea de comando pode excitar dinâmicas não modeladas (vibração, transientes de motor) mesmo que o valor final esteja dentro dos limites.

### 10.4 Anti-windup (implícito no rate limiter)

**Conceito clássico de anti-windup**: em controladores com ação integral (ou, aqui, com um limitador com estado), se o valor "desejado" ficar preso no limite por muito tempo, um estado interno pode "acumular" erro que depois causa um overshoot quando a saturação é liberada. Aqui, como o limitador de taxa usa `cmdB_prev` (o **último valor realmente enviado**, já saturado) como referência — e não o alvo bruto `cmdB_raw` — ele nunca tenta "compensar" um salto que nunca foi de fato comandado. Esse é o mecanismo de anti-windup mais simples possível: **a memória do sistema é sempre a ação real, nunca a intenção não realizada**.

### 10.5 Filtro passa-baixa (low-pass filter)

```matlab
dvd_B_filt = (1 - alpha)*dvd_B_filt_prev + alpha*dvd_B_raw;
```

Este é um **filtro exponencial de primeira ordem** (equivalente discreto de um filtro RC passa-baixa). `alpha` controla o compromisso entre suavidade e atraso:
- `alpha` pequeno → filtra mais ruído, mas reage mais devagar a mudanças reais.
- `alpha` próximo de 1 → quase sem filtro (responde rápido, mas deixa passar ruído).

Combinado com o **reset explícito** na troca de fase (`transicao_prep_formacao`), isso resolve tanto o ruído contínuo quanto o pico pontual estrutural descrito em §8.

---

## 11. Segurança de sistema: geofencing e watchdog de sensor

Essas duas técnicas **não fazem parte do controlador** propriamente dito — são redes de segurança independentes, que interrompem o experimento se algo sair do esperado, *seja qual for a causa*.

### Geofencing (parede virtual)

```matlab
if abs(p2(1)) > cfg.bebop_limite_xy || abs(p2(2)) > cfg.bebop_limite_xy || p2(3) > cfg.bebop_limite_z
    break;  % aborta com segurança
end
```

Conceito simples: uma caixa de operação segura. Qualquer saída da caixa é tratada como emergência, independentemente de o controlador achar que está "tudo certo". É a última linha de defesa contra bugs de controle, falhas de sensor não previstas, ou qualquer comportamento emergente não antecipado.

### Watchdog de sensor (freshness check)

```matlab
if ts1 > last_ts_L
    last_ts_L = ts1; last_update_L = tic;
end
if toc(last_update_L) > cfg.optitrack_timeout_s
    break;  % OptiTrack parado — aborta
end
```

Conceito: **verificar que o dado do sensor está realmente sendo atualizado**, não apenas ler o último valor disponível (`LatestMessage`, que pode estar "congelado" se o publisher parou). Isso é crítico em sistemas de tempo real com sensores em rede: sem essa checagem, uma pose parada parece (matematicamente) uma velocidade zero — e quando a pose finalmente atualiza, a diferença finita (§8) produz um salto artificial de velocidade que não corresponde a nenhum movimento real do robô.

---

## 12. ROS: o mínimo necessário para ler o código

Não é preciso saber ROS a fundo — apenas os quatro conceitos usados no arquivo:

- **Publisher/Subscriber**: `rospublisher`/`rossubscriber` — comunicação assíncrona por tópicos nomeados (`/L1/cmd_vel`, `/natnet_ros/B1/pose`, etc.), sem conexão direta entre quem envia e quem recebe.
- **Mensagens tipadas**: `geometry_msgs/Twist` (velocidades lineares/angulares) e `geometry_msgs/PoseStamped` (posição + orientação + timestamp).
- **Loop de controle a frequência fixa**: o padrão `tic ... processa ... pause(max(0, T - toc))` mede quanto tempo o processamento levou e dorme só o suficiente para completar o período `T=1/30 s` — mantendo a frequência de amostragem aproximadamente constante mesmo com variação de carga de processamento.
- **Comandos discretos de voo**: `/B1/takeoff` e `/B1/land` são mensagens vazias (`std_msgs/Empty`) que disparam ações no firmware do drone, fora da malha de controle contínua.

---

## Como usar este documento para revisar o código

Ao ler qualquer trecho novo de `formacao_2.m`, pergunte:

1. **Que grandeza física é essa variável?** (posição, velocidade, aceleração, comando normalizado?)
2. **Em que referencial ela está?** (mundo ou corpo? Precisa de uma rotação para converter?)
3. **De onde vem esse valor — é medido, é integrado, é diferenciado, ou é uma referência planejada?** (medido = ruidoso; diferenciado = ainda mais ruidoso; planejado = exato.)
4. **Essa grandeza está saturada em algum nível? Deveria estar?**
5. **Se essa grandeza tiver um salto/descontinuidade, o que acontece a jusante (derivadas, ganhos altos)?**

Essas cinco perguntas cobrem a origem de praticamente todos os bugs de robustez encontrados no diagnóstico original ([`docs/diagnostico_formacao_2_queda_bebop.md`](../docs/diagnostico_formacao_2_queda_bebop.md)).

## Leituras complementares

- Siciliano, Sciavicco, Villani, Oriolo — *Robotics: Modelling, Planning and Control*, caps. 7 (dinâmica), 9 (controle baseado em modelo).
- Siegwart, Nourbakhsh, Scaramuzza — *Introduction to Autonomous Mobile Robots*, caps. 3 (cinemática), 10 (planejamento/campos potenciais).
- Antonelli, Chiaverini — *Kinematic control of platoons of autonomous vehicles* (NSB / prioridade de tarefas).
- Khatib — *Real-Time Obstacle Avoidance for Manipulators and Mobile Robots* (campos potenciais, 1986).
- Åström, Murray — *Feedback Systems*, cap. sobre PID e anti-windup (para os conceitos de §10).

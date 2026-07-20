# Alterações em `formacao_2.m` — correção incremental de robustez do Bebop

Documento de acompanhamento da evolução incremental de [`matlab/formacao_2.m`](../matlab/formacao_2.m), comparando com o controlador que voou com sucesso, [`external/formacion_limo_bebop_final.m`](../external/formacion_limo_bebop_final.m), e com o diagnóstico prévio já existente em [`docs/diagnostico_formacao_2_queda_bebop.md`](diagnostico_formacao_2_queda_bebop.md).

**Escopo respeitado:** nenhuma arquitetura foi reescrita. Inner loop → Outer loop → Estrutura Virtual → NSB → `cmd_vel` permanece igual. Os ganhos do LIMO (`cfg.kd_limo`, `cfg.kq`, `cfg.lq`, `cfg.v_max`, `cfg.w_max`, `cfg.theta_limo`) **não foram tocados**. A orientação do Bebop continua não controlada explicitamente (`vd_B(4) = 0`, igual ao original). Toda mudança está marcada no código com um bloco `MODIFICAÇÃO / Motivo / Impacto esperado`.

---

## 1. Comparação de arquiteturas

| Bloco | `formacao_2.m` (original) | `formacion_limo_bebop_final.m` | Classificação |
| --- | --- | --- | --- |
| Geração de referência (lemniscata) | `lemniscata_reference(t)`, mesma fórmula | Inline, mesma fórmula (`w_tray=2π/40`) | Estética |
| Estrutura virtual (offset PoI, ρ/α/β) | `p2d` via `rho_f, alpha_f, beta_f` (formulação geral) | `pd_B1 = [p_ctrl_L1; altura_deseada]` (equivalente a α=0, β=90°) | Estética — mesma geometria, `formacao_2.m` é mais geral |
| Controle do LIMO (outer+inner) | Outer com `tanh` + espaço nulo de obstáculo + inner dinâmico com `theta_limo` (idêntico a `limoControl.m`) | Outer PD linear (`K_kin_L1`) + espaço nulo de obstáculo + inner dinâmico com `H_L1`/`Kd_L1` | Estética/desempenho — ambos fisicamente equivalentes; `formacao_2.m` tem `tanh` (mais suave) e zona de cruzamento adicional |
| NSB / desvio de obstáculo | `apply_obstacle_null_space_xy` com potencial superelíptico suave, satura `v_rep` | Projeção de espaço nulo padrão com repulsão `1/d - 1/R_infl` | Estética — ambos corretos, `formacao_2.m` é mais elaborado |
| Controle do Bebop (outer) | `dx2 = feedforward + Ls_B*tanh(Ls_B\(Kp_B*erro))` — **já tinha uma saturação suave** | `v_cmd_B1_global = vd_B1_global + Kp_B1*erro` — **sem saturação nenhuma no laço cinemático** | Estética/desempenho — `formacao_2.m` já era mais conservador aqui |
| Compensação dinâmica do Bebop | `cmdB_raw = f1\(dvd_B + KD_B*(vd_B-vB_meas) + f2*vB_meas)`, `KD_B=diag([4,4,4,4])` | Mesma fórmula, `Kd_B1=diag([2,2,1.8,5])` | **Instabilidade** — `KD_B` do original amplifica 2x mais o ruído de `vB_meas` |
| Derivada `dvd_B`/`dot_vd_B1` | Diferença finita bruta `(vd_B-vd_B_ant)/dt`, sem filtro | Diferença finita bruta, sem filtro | Instabilidade em ambos, mas `formacao_2.m` tem um ponto extra de descontinuidade (ver §3) que o outro não tem |
| Saturação de comando final | **Único limite `umax_B=1.0` para vx,vy,vz,yaw** | **Limites por eixo**: vx,vy=±0.5, vz=±0.3, yaw=±0.5 | **Instabilidade — principal diferença** |
| Rate limiter / anti-windup | Nenhum | Nenhum | Ambos ausentes; adicionado agora por exigência de robustez |
| Parede virtual (geofence) | Nenhuma | `limite_xy=1.8`, `limite_z=1.8`, aborta e pousa | **Instabilidade** (rede de segurança ausente) |
| Watchdog de OptiTrack | Nenhum (lê `LatestMessage` sem checar timestamp) | Checa `Header.Stamp`, timeout de 0.5 s | Instabilidade potencial (picos de derivada por amostra repetida) |
| Takeoff → hover → formação | Existe (`cfg.takeoff_wait_s` + `cfg.preparation_time_s`), mas ganhos plenos desde o 1º ciclo da preparação | Takeoff está **comentado** no arquivo (não representa a sequência real usada em voo) | Estética/robustez — mantido e reforçado com soft start |
| Auditoria/log | Arquivo `.txt` detalhado por ciclo | CSV + gráficos ao final | Estética |

---

## 2. Por que o Bebop caiu

A evidência já registrada em `docs/diagnostico_formacao_2_queda_bebop.md` (log de voo real) mostra:

- comando bruto `[+1.129; -2.496; +0.554; +0.001]` saturado para `[+1.000; -1.000; +0.554; +0.001]` — **saturação horizontal recorrente**, com `cmdB_raw` pedindo até 2,5x o limite enviado;
- altura subindo até 1,59 m e caindo a -1,12 m/s logo em seguida, com o controlador ainda pedindo subida (`vz desejada = +0,49` a `+0,60 m/s`) — ou seja, o controlador **não** mandou descer; a perda de controle veio de comandos horizontais/verticais desproporcionais e persistentes, não de um sinal de descida deliberado;
- erro inicial de formação de ~0,81 m (Y) e ~0,78 m (Z) já na entrada da fase de preparação, corrigido em todos os eixos simultaneamente com ganho pleno.

Cadeia causal reconstruída, do ponto de vista de controle (excluindo ROS/driver/firmware, conforme escopo definido):

1. **`umax_B = 1.0` único para todos os eixos.** O canal vertical do Bebop satura fisicamente muito antes do horizontal; permitir `vz` até o mesmo teto de `vx/vy` é desproporcional e é a causa mais provável, isoladamente, do comportamento agressivo relatado.
2. **`KD_B = diag([4,4,4,4])`, o dobro do valor usado no controlador que voou (`Kd_B1 ≈ diag([2,2,1.8,5])`).** Esse ganho multiplica `(vd_B - vB_meas)`, e `vB_meas` vem de diferença finita bruta da pose do OptiTrack a 30 Hz — um sinal ruidoso por natureza. Com ganho maior, o ruído é amplificado com mais força dentro de `cmdB_raw`, empurrando o comando para a saturação com mais frequência.
3. **Ausência de saturação por eixo no comando final** — combinada com os dois pontos acima, o comando enviado ficava preso no teto (`±1.0`) de forma sustentada em vez de oscilar perto do valor real necessário, o que é consistente com o padrão "saturação recorrente" visto nos gráficos citados no diagnóstico.
4. **Erro inicial grande em todos os eixos ao mesmo tempo, com ganho pleno desde o primeiro ciclo** — a fase de preparação já existia, mas não havia rampa de ganho; o primeiro ciclo após o takeoff pedia, de uma vez, a correção de X, Y e Z simultaneamente, com o mesmo ganho usado em regime permanente.
5. **Derivada `dvd_B` sem filtro, e com uma descontinuidade estrutural extra**: `p2d` é calculado por uma fórmula durante a preparação (`p2d=[poi; z1+rho_f]`) e por outra na formação ativa (`p2d=[poi+ρcos(α)cos(β); ...]`). Na troca de fase, `p2d` pode saltar, `dx2` salta, `vd_B` salta, e `dvd_B=(vd_B-vd_B_ant)/dt` vira um pico de "aceleração" instantâneo — exatamente o tipo de evento que o compensador dinâmico (`f1\(...)`) converte diretamente em um pico de comando.
6. **Nenhuma rede de segurança** (geofence, watchdog de pose) para interromper o experimento quando o comportamento já estava divergindo, ao contrário do `formacion_limo_bebop_final.m`.

Nenhum desses pontos exige mudar a arquitetura Inner→Outer→Virtual Structure→NSB→`cmd_vel`; todos são ajustes de ganho, saturação e filtragem dentro da mesma estrutura.

---

## 3. Modificações implementadas

Todas com o bloco `MODIFICAÇÃO/Motivo/Impacto esperado` no código-fonte.

1. **`KD_B` reduzido** de `diag([4,4,4,4])` para `diag([2.5,2.5,2.0,2.5])` — ainda acima do valor do código que voou, mas com folga, já que agora há filtro de derivada e rate limiter cobrindo o resto.
2. **Saturação por eixo do comando final** (`cmdB_max = [0.5;0.5;0.3;0.5]`) substitui o `umax_B=1.0` único, replicando o envelope validado em voo real.
3. **Saturação de nível 1** na velocidade mundo desejada `dx2` (`vd_B_max = [0.5;0.5;0.3]`), antes de rotacionar para o corpo e antes do compensador dinâmico.
4. **Soft start dos ganhos do Bebop**: `gamma = min(max(t/8, 0.3), 1)` multiplica `Kp_B` e `KD_B`. Começa em 0,3 (nunca zero, para não perder toda a autoridade de controle logo após o hover) e sobe a 1,0 em 8 s.
5. **Filtro passa-baixa + saturação de `dvd_B`**, com **reset explícito na transição preparação→formação** (o ponto exato de descontinuidade estrutural de `p2d` identificado no item 2.5 acima).
6. **Rate limiter (limitador de taxa)** do comando final, referenciado ao último comando **realmente enviado** (`cmdB_prev`), não ao alvo bruto — isso também implementa o **anti-windup** pedido: o limitador nunca tenta compensar um salto que nunca chegou a ser comandado de fato.
7. **Parede virtual (geofence)** para o Bebop: aborta o experimento se `|x|>1.8`, `|y|>1.8` ou `z>1.8`, copiado de `formacion_limo_bebop_final.m`.
8. **Watchdog de OptiTrack**: `ler_pose` agora retorna o timestamp do header; se a pose do LIMO ou do Bebop ficar parada por mais de 0,5 s, o experimento é abortado com segurança. Copiado do padrão de `formacion_limo_bebop_final.m`.
9. **NSB e parede de obstáculo do LIMO preservados sem alteração** (`apply_obstacle_null_space_xy`, `obstacle_repulsive_gradient`).
10. **Orientação do Bebop continua não controlada** (`vd_B(4)=0`), como já era e como os orientadores pediram — nenhuma mudança aqui, mas registrado para deixar explícito que não foi copiada a lógica de `angdiff(psi_B1, yaw_d_B1)` do código de referência (que controla yaw), pois isso contrariaria o requisito do projeto.

### O que foi copiado do código do outro grupo, e por quê

| Lógica copiada | Por que funciona | Problema que resolve | Por que é melhor que o original aqui |
| --- | --- | --- | --- |
| Limites de comando por eixo (`0.5/0.5/0.3/0.5`) | Respeita a proporção real de autoridade de cada eixo do Bebop; validado em voo | Saturação persistente e desproporcional no eixo vertical | O original usava um teto único de 1.0 para todos os eixos, ignorando que o eixo vertical satura antes |
| Parede virtual (geofence XY/Z) | Interrompe o experimento antes que o erro divirja fisicamente, independente da causa | Ausência total de rede de segurança espacial | O original não tinha nenhuma; é aditivo, não conflita com a estrutura virtual existente |
| Watchdog de timestamp do OptiTrack | Evita que uma amostra repetida (velocidade aparente zero) seguida de uma atualização atrasada gere picos de derivada | Picos de `vB_meas`/`dvd_B` por amostra "congelada" | O original lia `LatestMessage` sem verificar se havia mudado |

### O que **não** foi copiado, e por quê

- **Controle explícito de yaw do Bebop** (`w_cmd_B1 = 1.0*angdiff(psi_B1, yaw_d_B1)`): contraria o requisito explícito do projeto de não controlar orientação do drone.
- **Ganhos cinemáticos maiores** (`Kp_B1=diag([2,2,2])`, `K_kin_L1=diag([1.5,1.5])`): o `formacao_2.m` já usa `tanh` para suavizar o laço cinemático do Bebop e ganhos de LIMO já validados (`kq=0.8, lq=0.30`); não havia motivo para adotar ganhos maiores e menos suaves.
- **Reescrita do laço do LIMO**: por instrução explícita, os ganhos do LIMO não foram tocados.

---

## 4. Justificativa matemática (resumo)

O compensador dinâmico do Bebop é uma linearização por realimentação (feedback linearization / dynamic inversion):

```
cmdB_raw = f1⁻¹ · ( dvd_B + KD_B·(vd_B − vB_meas) + f2·vB_meas )
```

Isso é válido *desde que* `vd_B`, `dvd_B` e `vB_meas` sejam sinais bem-comportados. Três das mudanças atacam diretamente essa premissa:

- `KD_B` menor reduz o ganho com que ruído em `vB_meas` (finita, discreta, ruidosa) é injetado em `cmdB_raw`.
- O filtro + saturação de `dvd_B` evita que descontinuidades em `vd_B` (por exemplo na troca de fase preparação→formação, ou por saltos de `vB_meas` decorrentes de pose obsoleta) apareçam como picos de "aceleração" no comando.
- A saturação em dois níveis (na velocidade desejada e no comando final) garante que, mesmo se `cmdB_raw` explodir por qualquer motivo residual, o valor **enviado ao driver** nunca ultrapassa o envelope validado, e o rate limiter garante que a transição para esse valor também seja suave.

O soft start (`gamma`) não muda a estrutura do controlador — apenas escala `Kp_B`/`KD_B` continuamente de 0,3 a 1,0 — portanto preserva a mesma lei de controle em regime permanente, apenas atenuando o transiente inicial.

## 5. Justificativa baseada na dinâmica do Bebop

- O Bebop 2 tem autoridade de controle muito diferente por eixo: o eixo vertical satura com comandos bem menores que os eixos horizontais (rotores respondem a variações de empuxo total, que têm limites de aceleração vertical menores que a manobra de inclinação usada para XY). Um teto único de 1.0 para todos os eixos ignora essa assimetria física — daí a mudança mais crítica (item 3.2).
- O quadrotor é instável em malha aberta; qualquer comando sustentado no limite superior por vários ciclos consecutivos (como visto no log) tende a produzir uma resposta de atitude agressiva o suficiente para perder altitude rapidamente quando combinado com dinâmica de arrasto/rotação não modelada — por isso a prioridade de evitar saturação persistente, não apenas "cortar" o valor.
- Comandos que mudam abruptamente entre ciclos (30 Hz) excitam a dinâmica de atitude do drone com uma taxa de variação que o controlador de baixo nível do Bebop pode não conseguir rastrear suavemente — daí o rate limiter.

---

## 6. Trechos de código alterados

Ver [`matlab/formacao_2.m`](../matlab/formacao_2.m); cada trecho está marcado com:

```matlab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MODIFICAÇÃO
% Motivo: ...
% Impacto esperado: ...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
```

Principais blocos alterados (linhas aproximadas no arquivo final):

- Configuração de ganhos e novos parâmetros do Bebop (`KD_B`, `cmdB_max`, `vd_B_max`, `cfg.soft_start_*`, `cfg.dvd_B_*`, `cfg.cmdB_rate_max`, `cfg.bebop_limite_*`).
- Novos estados antes do loop (`cmdB_prev`, `dvd_B_filt`, `em_preparacao_ant`, watchdog de timestamp).
- `ler_pose` retornando timestamp do header.
- Dentro do loop: watchdog de OptiTrack, parede virtual, soft start (`gamma`), saturação de nível 1, filtro/reset/saturação de `dvd_B`, saturação de nível 2 por eixo, rate limiter com anti-windup implícito.
- Atualização de estado ao fim do ciclo (`cmdB_prev`, `em_preparacao_ant`, histórico `H.gamma`, `H.satRate`).

O restante do arquivo (geração de lemniscata, controle do LIMO, NSB, zona de cruzamento, auditoria, plots) permanece **inalterado**.

---

## 7. Checklist para teste no laboratório

Seguir em ordem, sem pular etapas (consistente com `docs/diagnostico_formacao_2_queda_bebop.md`):

1. **`MODO_BEBOP = 'off'`** — rodar a simulação virtual completa, checar no arquivo de auditoria (`results/formacao_2/audit_*.txt`) que `H.satB` e `H.satRate` ficam majoritariamente falsos e que o erro de formação converge.
2. **`MODO_BEBOP = 'teste'`** (sem enviar comando real, se aplicável) ou hover manual — confirmar que o Bebop não recebe `cmd_vel` fora dos novos limites (`±0.5/±0.5/±0.3/±0.5`).
3. **Bebop em hover parado, LIMO parado** (`TRAJ = 0`) — validar que:
   - `gamma` sobe suavemente de 0,3 a 1,0 nos primeiros 8 s (checar `H.gamma`);
   - não há saturação persistente (`H.satB`) além de picos isolados;
   - a parede virtual e o watchdog de OptiTrack não disparam falsamente.
4. **LIMO parado, Bebop tentando alcançar `p2d` fixo** — confirmar erro de formação decrescente sem oscilação visível.
5. **Formação completa com `TRAJ = 1` (lemniscata) em área livre de obstáculo** — monitorar em tempo real o print de saturação/erro a cada 30 ciclos; abortar manualmente (botão do joystick) ao primeiro sinal de comando sustentado no limite.
6. **Somente depois**, habilitar `cfg.use_obstacle_avoidance = true` com o obstáculo físico presente.
7. Em todos os testes com Bebop real: manter piloto de segurança pronto para pousar manualmente; ter o botão de parada do joystick testado antes de cada execução; revisar o arquivo de auditoria após cada voo antes do próximo teste, olhando especialmente `H.satB`, `H.satRate` e o resumo de "Amostras com saturação do Bebop".
8. Validar que o pouso automático (`pub_LD`) ocorre corretamente em todas as saídas do loop: parada por joystick, parede virtual, watchdog de OptiTrack e término normal do tempo de simulação.

---

## 8. Observação sobre escopo

Este documento e as mudanças em `formacao_2.m` **não tratam** de: possível divergência de unidade/escala entre o `u` do compensador dinâmico e o `cmd_vel` esperado pelo driver do Bebop, namespaces, IP do OptiTrack, firmware ou biblioteca ROS — apontado como hipótese em aberto no diagnóstico anterior (`docs/diagnostico_formacao_2_queda_bebop.md`, item 2). Essa hipótese deve ser validada isoladamente com `matlab/test_bebop.m` em teleop antes do próximo voo de formação, conforme já recomendado naquele documento.

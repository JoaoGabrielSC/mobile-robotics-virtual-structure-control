# Propostas de mudança — `matlab/limo_bebop.m`

Este documento mostra como ficariam, no código, as três ressalvas levantadas sobre o controlador do Bebop. **Nada foi aplicado ainda** — são propostas para revisão. Como já registrado, nenhuma delas existe no `external/formacion_limo_bebop_final.m` (o script que voou com sucesso) — são margem de segurança extra, motivada pelo histórico de queda deste projeto, não uma correção de algo comprovadamente necessário.

## 1. Rate limiter (limitador de taxa de variação do comando)

**Motivo**: hoje `cmdB` vai direto do cálculo pro `send()`. Se o cálculo pedir uma mudança grande num único ciclo (pico de ruído, transição de fase), o Bebop recebe esse salto integralmente. O limitador garante que o comando nunca varia mais que uma taxa máxima por segundo — e, aplicado sobre o *último comando realmente enviado* (não o alvo bruto), funciona também como anti-windup.

**Config** (perto de `cmdB_max`):
```matlab
cmdB_max = [0.5; 0.5; 0.3; 0.5];
cfg.cmdB_rate_max = [1.2; 1.2; 0.8; 1.2]; % taxa máxima de variação do comando (unid./s)
```

**Estado inicial** (perto de `vd_B_ant`):
```matlab
vd_B_ant = [0; 0; 0; 0];
cmdB_prev = zeros(4, 1);
```

**No loop**, onde hoje é:
```matlab
cmdB_raw = f1 \ (dvd_B + KD_B * (vd_B - vB_meas) + f2 * vB_meas);
cmdB = max(min(cmdB_raw, cmdB_max), -cmdB_max);
```
passaria a ser:
```matlab
cmdB_raw = f1 \ (dvd_B + KD_B * (vd_B - vB_meas) + f2 * vB_meas);
cmdB_satN2 = max(min(cmdB_raw, cmdB_max), -cmdB_max); % nível 2: saturação do comando final
delta_max = cfg.cmdB_rate_max * dt;
delta_cmd = max(min(cmdB_satN2 - cmdB_prev, delta_max), -delta_max);
cmdB = cmdB_prev + delta_cmd;
```

**No fim do loop**, junto de `vd_B_ant = vd_B;`:
```matlab
cmdB_prev = cmdB;
```

## 2. Filtro passa-baixa + saturação de `dvd_B`

**Motivo**: `dvd_B` hoje é diferença finita bruta de `vd_B` — qualquer ruído na diferenciação (inclusive de uma pose levemente atrasada) entra direto no compensador dinâmico, multiplicado por `KD_B`. Um filtro passa-baixa suaviza isso, e um teto evita que um pico isolado ainda passe integralmente.

**Config**:
```matlab
cfg.dvd_B_filter_alpha = 0.3;        % filtro passa-baixa (0<alpha<=1)
cfg.dvd_B_max = [1.0; 1.0; 0.6; 1.0]; % teto de |dvd_B| (m/s², rad/s²)
```

**Estado inicial**:
```matlab
dvd_B_filt = zeros(4, 1);
```

**No loop**, onde hoje é:
```matlab
transicao_prep_formacao = em_preparacao_ant && ~em_preparacao;
if k == 1 || transicao_prep_formacao
    dvd_B = zeros(4, 1);
else
    dvd_B = (vd_B - vd_B_ant) / dt;
end
```
passaria a ser:
```matlab
transicao_prep_formacao = em_preparacao_ant && ~em_preparacao;
if k == 1 || transicao_prep_formacao
    dvd_B_filt = zeros(4, 1);
else
    dvd_B_raw = (vd_B - vd_B_ant) / dt;
    dvd_B_filt = (1 - cfg.dvd_B_filter_alpha) * dvd_B_filt + cfg.dvd_B_filter_alpha * dvd_B_raw;
end
dvd_B = max(min(dvd_B_filt, cfg.dvd_B_max), -cfg.dvd_B_max);
```

## 3. Soft start (ganhos do Bebop crescendo suavemente)

**Motivo**: `Kp_B` e `KD_B` valem o total desde o primeiro ciclo após a decolagem. Se o erro inicial for grande, o comando inicial pode ser agressivo. Uma rampa de ganho nos primeiros segundos amortece esse transiente sem mudar o comportamento em regime permanente.

**Config**:
```matlab
cfg.soft_start_time_s = 8.0;    % tempo de rampa suave dos ganhos do Bebop
cfg.soft_start_gamma_min = 0.3; % ganho mínimo no início (evita autoridade nula)
```

**No loop**, logo após calcular `t`/`dt`:
```matlab
gamma = min(max(t / cfg.soft_start_time_s, cfg.soft_start_gamma_min), 1);
```

Onde hoje é:
```matlab
dx2 = [vel_poi_world; 0] + Ls_B * tanh(Ls_B \ (Kp_B * (p2d - p2)));
```
passaria a ser:
```matlab
Kp_B_eff = gamma * Kp_B;
dx2 = [vel_poi_world; 0] + Ls_B * tanh(Ls_B \ (Kp_B_eff * (p2d - p2)));
```

E onde hoje é:
```matlab
cmdB_raw = f1 \ (dvd_B + KD_B * (vd_B - vB_meas) + f2 * vB_meas);
```
passaria a ser:
```matlab
KD_B_eff = gamma * KD_B;
cmdB_raw = f1 \ (dvd_B + KD_B_eff * (vd_B - vB_meas) + f2 * vB_meas);
```

## 4. (Opcional) Decolagem manual com confirmação por botão

**Motivo**: o `external` decola manualmente (o `send(pub_tkf,...)` está comentado no script deles) e só libera o controle depois que o piloto aperta um botão — uma convenção operacional, não um gate no código. Se vocês quiserem o mesmo protocolo mas com o script realmente esperando a confirmação (em vez de confiar só na convenção), dá pra adicionar um `cfg.auto_takeoff` + espera de botão antes da decolagem — like já tínhamos implementado numa versão anterior. Chamo separadamente porque é uma mudança de **fluxo operacional**, não de robustez do compensador — vale decidir independente das três acima.

---

Se quiser que eu aplique alguma dessas (ou todas) no `limo_bebop.m`, é só confirmar quais.

# Diagnóstico do `formacao_2.m` e queda do Bebop

## Escopo

Este documento analisa a execução registrada em [`matlab/audit_formacao_20260716_175134_drop_drone.txt`](../matlab/audit_formacao_20260716_175134_drop_drone.txt), os gráficos em [`matlab/drop_drone/`](../matlab/drop_drone/), o controlador atual [`matlab/formacao_2.m`](../matlab/formacao_2.m) e a especificação formal em [`SPEC.md`](../SPEC.md).

O objetivo não é atribuir uma causa física sem telemetria do Bebop. O objetivo é separar:

1. fatos observados no registro;
2. incompatibilidades ou riscos confirmados no código;
3. hipóteses que precisam ser testadas isoladamente;
4. correções necessárias antes de outro voo de formação.

## Objetivo do trabalho

O trabalho requer uma estrutura virtual LIMO–Bebop com:

- PoI do LIMO deslocado `0,10 m`;
- lemniscata de período `40 s`;
- Bebop em altitude constante de `1,5 m`;
- laço externo cinemático;
- compensadores internos;
- evasão de obstáculo por espaço nulo;
- frequência de `30 Hz`;
- integração MATLAB, ROS e OptiTrack.

O requisito de segurança é implícito: a validação deve ser incremental e nunca começar diretamente com voo de formação.

## Resultado observado no voo auditado

O registro foi gerado em `MODO_BEBOP = voo` e `TRAJ = 0`, portanto não valida a lemniscata. Ele registra uma tentativa de estabilização e posicionamento.

### Sequência vertical observada

| Tempo | Altura medida do Bebop | Observação |
| --- | ---: | --- |
| `0,000 s` | `0,903 m` | início da preparação |
| `1,024 s` | `1,323 m` | Bebop sobe |
| `2,052 s` | `1,589 m` | Bebop próximo do alvo vertical |
| `3,068 s` | `1,117 m` | queda iniciada; velocidade vertical medida `-1,120 m/s` |
| `4,080 s` | `0,116 m` | Bebop praticamente no chão |
| `5,091 s` | `0,113 m` | permanece próximo ao chão |

O drone caiu entre aproximadamente `2,05 s` e `4,08 s`.

### O controlador mandava subir durante a queda

No instante `t = 3,068 s`, o log registra:

```text
vB desejada:       vz = +0,487 m/s
vB medida:          vz = -1,120 m/s
comando enviado:    uz = +0,613
```

Em `t = 4,080 s`, ainda após a queda:

```text
vB desejada:       vz = +0,598 m/s
comando enviado:    uz = +0,604
```

Portanto, o log não sustenta a hipótese de que o controlador mandou explicitamente o Bebop descer. Também não prova que o comando vertical enviado seja interpretado pelo driver com a escala e semântica esperadas.

### Saturação horizontal confirmada

No primeiro registro, o comando bruto calculado foi aproximadamente:

```text
[+1,129; -2,496; +0,554; +0,001]
```

Após saturação, o Bebop recebeu:

```text
[+1,000; -1,000; +0,554; +0,001]
```

Os gráficos em [`matlab/drop_drone/sinais_e_metricas.png`](../matlab/drop_drone/sinais_e_metricas.png) mostram saturação recorrente em X e Y. Isso é um fato observado, não uma hipótese.

## Problemas confirmados no código atual

### 1. Limite único e agressivo para todos os canais do Bebop

`formacao_2.m` usa um único limite:

```matlab
umax_B = 1.0;
cmdB = saturar(cmdB_raw, umax_B);
```

Isso permite `±1,0` simultaneamente em `Linear.X`, `Linear.Y`, `Linear.Z` e `Angular.Z`.

O teste isolado do Bebop usa limites seguros distintos:

```matlab
cfg.vxy_max_safe = 0.5;
cfg.vz_max_safe = 0.3;
cfg.vyaw_max_safe = deg2rad(30);
```

Solução:

- usar saturação por eixo;
- começar com `±0,5 m/s` em XY, `±0,3 m/s` em Z e `±30°/s` em yaw;
- registrar a saturação e reduzir a referência ou os ganhos quando ela persistir.

### 2. Inconsistência potencial entre `u` do compensador e `cmd_vel`

Em `formacao_2.m`, a entrada `u` obtida pela inversão do modelo dinâmico é publicada diretamente:

```matlab
cmdB_raw = f1 \ (dvd_B + KD_B * (vd_B - vB_meas) + f2 * vB_meas);
msg_B.Linear.X = cmdB(1);
msg_B.Linear.Y = cmdB(2);
msg_B.Linear.Z = cmdB(3);
msg_B.Angular.Z = cmdB(4);
```

Por outro lado, o driver do Bebop espera velocidades em `cmd_vel`. O script isolado [`matlab/test_bebop.m`](../matlab/test_bebop.m) evolui um estado de velocidade interno e publica velocidades limitadas no corpo do drone.

Isso não prova que `formacao_2.m` esteja matematicamente errado, pois `u` pode ter sido identificado com essa semântica. Porém, é a hipótese de interface mais importante: se `u` não tiver unidades de velocidade compatíveis com `cmd_vel`, o Bebop real não seguirá a planta usada no compensador.

Solução:

1. validar a escala e o sinal de cada eixo com `test_bebop.m` em teleop;
2. validar hover antes de usar formação;
3. só publicar `u` diretamente se a identificação de `f1` e `f2` tiver usado exatamente o mesmo driver, frame e escala;
4. caso contrário, publicar a velocidade desejada filtrada, como no teste isolado.

### 3. Derivadas de pose sem validação de amostra nova

O controlador estima velocidade e aceleração por diferenças:

```matlab
velWB = (p2 - poseB_ant) / dt;
psidot2 = wrap_pi(psi2 - poseB_psi_ant) / dt;
dvd_B = (vd_B - vd_B_ant) / dt;
```

Mas lê apenas `LatestMessage`, sem verificar se o timestamp mudou. Uma pose repetida, atrasada ou ruidosa do OptiTrack pode introduzir picos em `vB_meas` e `dvd_B`; ambos são amplificados pelo compensador dinâmico.

Solução:

- guardar e validar `Header.Stamp`;
- atualizar estimativas somente quando houver uma mensagem nova;
- aplicar filtro de velocidade;
- declarar falha se a pose ficar velha por mais de um limite, por exemplo `0,5 s`;
- enviar comando nulo e pousar em caso de perda de rastreamento.

### 4. Preparação com erro inicial grande em todos os eixos

Durante a preparação, o alvo é diretamente sobre o PoI:

```matlab
p2d = [poi; z1 + rho_f];
```

No audit, o Bebop começou com erro aproximado de `0,81 m` no eixo Y e `0,78 m` em Z. O controlador tentou corrigir XY e altitude ao mesmo tempo, provocando saturação já no primeiro instante.

Solução:

1. takeoff com comando nulo e hover do firmware;
2. corrigir somente altitude com XY bloqueado;
3. corrigir XY com velocidade limitada depois que Z estiver estável;
4. liberar formação móvel apenas quando erro de altitude e velocidade estiverem abaixo de limiares definidos.

### 5. Ausência de geofence e de detecção de queda

`formacao_2.m` não possui limite espacial, teto, piso, watchdog de pose ou condição de queda durante o loop. Após o Bebop atingir aproximadamente `z = 0,11 m`, o controlador continuou tentando perseguir o alvo.

O script externo [`external/formacion_limo_bebop_final.m`](../external/formacion_limo_bebop_final.m) contém timeout de OptiTrack, geofence XY/Z e sequência explícita de parada e pouso. Isso é uma boa referência de segurança, embora não seja prova de estabilidade do controlador.

Solução:

- definir `x_min`, `x_max`, `y_min`, `y_max`, `z_min` e `z_max`;
- impedir velocidade que se aproxime de uma parede;
- entrar em emergência ao cruzar um limite;
- tratar `z < z_min` como queda ou pouso inesperado;
- enviar comando nulo, executar `land` e sair do loop.

### 6. Divergência da zona de evasão de obstáculo

O SPEC exige raio de influência de `0,50 m`. `formacao_2.m` usa `0,25 m`:

```matlab
cfg.obstacle_influence_radius = 0.25;
```

Solução:

- usar `0,50 m` como ponto inicial, conforme o requisito;
- ajustar esse valor apenas após medir margem de pose e espaço físico;
- nunca validar evasão de obstáculo no mesmo primeiro voo que valida formação.

### 7. Diferença entre o audit e o arquivo atual

O audit executado com `TRAJ = 0` registra alvo posterior `[0; 0,4; 1]`. O arquivo atual define:

```matlab
p2d = [0; 0; 1];
```

Logo, o log representa uma variante histórica de `formacao_2.m` ou parâmetros diferentes. Ele evidencia saturação e queda, mas não pode ser usado para afirmar que cada linha do arquivo atual produziu aquele mesmo voo.

## O que os códigos externos demonstram

Os scripts Pioneer demonstram estrutura virtual, `tanh`, Jacobianas e projeção em espaço nulo. Porém, ROS e envio de comandos estão comentados e as poses são integradas em simulação. Eles não validam dinâmica de quadricóptero, atraso, ruído de OptiTrack ou interface `cmd_vel`.

`external/formacion_limo_bebop_final.m` é mais próximo do cenário real porque usa LIMO, Bebop, OptiTrack e `cmd_vel`. Seus pontos úteis são:

- saturação por eixo conservadora;
- timeout de OptiTrack;
- geofence;
- parada e pouso repetido;
- logs de posição e erro.

Ele também não deve ser copiado sem validação: o takeoff está comentado e seu controlador não representa integralmente o estado de estrutura virtual definido no SPEC.

## Plano de correção e validação

### Fase 1: interface e segurança

1. Adicionar limites distintos por eixo.
2. Adicionar watchdog de timestamp do OptiTrack.
3. Adicionar paredes virtuais, teto, piso e detecção de queda.
4. Garantir comando nulo antes de `land`.
5. Confirmar namespace e interface reais do Bebop: `/B1/*` ou `/bebop/*`.

### Fase 2: Bebop isolado

1. Executar `test_bebop.m` em `monitor`.
2. Testar `takeoff_land`.
3. Testar teleop em cada eixo, com `±0,1`, `±0,3` e `±0,5`.
4. Testar hover em `z = 1,5 m`.
5. Testar deslocamentos XY pequenos a altitude constante.

Critério: nenhum comando pode causar saturação persistente, perda de altitude significativa ou divergência entre o movimento esperado e o observado.

### Fase 3: formação sem risco

1. Executar `formacao_2.m` em `MODO_BEBOP = 'off'`.
2. Revisar auditoria, erros e saturações.
3. Executar com Bebop em hover e LIMO parado.
4. Aplicar formação com velocidade XY muito baixa.
5. Só então liberar lemniscata e evasão de obstáculo.

## Conclusão

O evento de queda não é explicado por um comando vertical negativo no audit. A evidência aponta para uma combinação de saturação horizontal intensa, transição inicial agressiva e ausência de mecanismos de segurança.

Antes de alterar a geometria da formação ou os ganhos da lemniscata, a prioridade é validar a semântica de `cmd_vel`, limitar cada eixo de forma conservadora e tornar o loop seguro diante de queda, perda de pose ou saída da área permitida.

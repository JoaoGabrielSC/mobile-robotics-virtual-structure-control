# Loop de controle — `matlab/formacao_2.m`

Diagrama do loop de controle a 30 Hz, incluindo a fase de preparação, o laço externo em espaço de cluster (Eq. 5.7 + NSB, Eq. 5.13), os laços internos do LIMO e do Bebop, e as camadas de segurança independentes do controlador.

```mermaid
flowchart TD
    Start(["Início: ROS/OptiTrack + decolagem"]) --> ReadPose

    subgraph LOOP["Loop de controle a 30 Hz"]
        direction TB
        ReadPose["Ler poses do LIMO e do Bebop"] --> Watchdog{"Pose atualizou<br/>há menos de 0,5 s?"}
        Watchdog -- "não" --> AbortSafety["Abortar: comando nulo + pouso"]
        Watchdog -- "sim" --> POI["Calcular PoI do LIMO"]
        POI --> Prep{"Em preparação?"}

        Prep -- "sim" --> PrepBranch["LIMO parado (cmdL = 0)<br/>Bebop converge a p2d = poi + rho_f<br/>com soft start (Ls_B, tanh)"]

        Prep -- "não" --> Ref["Referência da lemniscata:<br/>xd, yd, xd_dot, yd_dot"]
        Ref --> BuildQ["Montar q (atual) e qd (desejado)<br/>do cluster"]
        BuildQ --> Law["Lei cinemática do cluster (Eq. 5.7)<br/>q_dot_r = qd_dot + L·tanh(L⁻¹·K·q_til)"]
        Law --> NSB{"Obstáculo dentro do<br/>raio de influência?"}
        NSB -- "sim" --> Repulsive["NSB (Eq. 5.13): gradiente repulsivo<br/>com prioridade máxima + projeção<br/>da formação no espaço nulo"]
        NSB -- "não" --> Jacobian
        Repulsive --> Jacobian["Jacobiana inversa<br/>x_dot = J⁻¹(q) · q_dot_r"]
        Jacobian --> ForceZ["Forçar z1_dot = 0<br/>(LIMO é terrestre)"]

        ForceZ --> LimoKin["Inversão cinemática A1inv"]
        LimoKin --> LimoDyn["Compensador dinâmico do LIMO<br/>(Eq. 4.44, theta_limo)"]
        LimoDyn --> CmdL["cmdL"]

        ForceZ --> Dx2["dx2 = x_dot do Bebop"]
        PrepBranch --> Wall
        Dx2 --> Wall{"Parede virtual<br/>violada?"}
        Wall -- "sim" --> AbortSafety
        Wall -- "não" --> Sat1["Saturação nível 1<br/>(vd_B_max)"]
        Sat1 --> Body["Rotação para o corpo<br/>A2inv → vd_B"]
        Body --> Deriv["dvd_B: diferença + filtro passa-baixa<br/>+ saturação (reset na transição<br/>preparação → formação)"]
        Deriv --> DynComp["Compensador dinâmico do Bebop<br/>cmdB_raw (f1, f2, KD_B)"]
        DynComp --> Sat2["Saturação nível 2<br/>(cmdB_max)"]
        Sat2 --> RateLimit["Rate limiter sobre cmdB_prev<br/>(anti-windup)"]
        RateLimit --> CmdB["cmdB"]

        CmdL --> SendL["Publicar /L1/cmd_vel"]
        CmdB --> SendB["Publicar /B1/cmd_vel"]
        SendL --> Audit["Registrar histórico e auditoria"]
        SendB --> Audit
        Audit --> StopBtn{"Botão de parada<br/>pressionado?"}
        StopBtn -- "não" --> ReadPose
    end

    StopBtn -- "sim" --> End
    AbortSafety --> End(["Encerrar: comando nulo,<br/>pouso, rosshutdown"])
```

## Notas de leitura

- **Camadas de segurança** (watchdog, parede virtual, botão de parada) são independentes da lógica de controle — interrompem o loop mesmo que o cálculo de comando esteja correto.
- O bloco **NSB** só é avaliado na fase ativa; durante a preparação o LIMO fica parado e não há obstáculo a evitar.
- `z1_dot` é sempre forçado a zero porque o LIMO é um robô terrestre — mesmo que a Jacobiana do cluster produza um valor não nulo para essa componente.
- O rate limiter atua sobre `cmdB_prev` (o último comando **realmente enviado**, não o alvo bruto), o que o torna também o mecanismo de anti-windup.

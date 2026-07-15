# drone_test.m — implementação pronta

> **Ação:** alterne para **Agent mode** e peça: *"aplique docs/drone_test_implementation.md em matlab/drone_test.m"*

Arquivo alvo: [`../matlab/drone_test.m`](../matlab/drone_test.m)

## Modos implementados

| Modo | Foco agora | Config principal |
|------|------------|------------------|
| `monitor` | **Sim** — 1º teste | `do_takeoff=false`, namespace |
| `hover` | Sim — só drone | `drone_altitude`, `altitude_kp`, `kd_drone` |
| `teleop` | Sim — manual | eixos joystick, `teleop_scale_*` |
| `formation` | Preparado | `kq/lq`, `rho_f`, `beta_f`, LIMO lido, `command_limo=false` |

## Ordem de teste no lab

1. `cfg.mode='monitor'`, `do_takeoff=false` → pose no chão  
2. `monitor` + `do_takeoff=true` → takeoff + cmd neutro  
3. `hover` + `do_takeoff=true` → z ≈ 1,5 m  
4. `formation` + LIMO parado visível no Motive  

## Diferença vs test_limo

- **Sem** potencial exponencial no drone  
- Obstáculo (modo `formation`) usa null-space no **PoI LIMO** (lei `main.m`)  
- Encerramento: **land** / **kill** (não só zero cmd)

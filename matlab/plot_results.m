function plot_results(H, cfg)
% Gera gráficos de trajetória, erro de formação e sinais de controle.

if isempty(H.t)
    return;
end

figure('Name', 'Trajetórias da formação', 'Color', 'w');
hold on;
grid on;
axis equal;
plot(H.qd(1, :), H.qd(2, :), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Referência');
plot(H.p_limo(1, :), H.p_limo(2, :), 'b', 'LineWidth', 1.5, 'DisplayName', 'LIMO');
plot(H.p_cf(1, :), H.p_cf(2, :), 'r', 'LineWidth', 1.2, 'DisplayName', 'Crazyflie');
draw_circle(cfg.obstacle_center, cfg.obstacle_radius, 'r-');
draw_circle(cfg.obstacle_center, cfg.obstacle_influence_radius, 'k:');
xlabel('x (m)');
ylabel('y (m)');
title('Trajetórias no plano XY');
legend('Location', 'best');

labels = {'x_f (m)', 'y_f (m)', 'z_f (m)', '\rho (m)', '\alpha (rad)', '\beta (rad)'};
figure('Name', 'Estado e erro da formação', 'Color', 'w');
for i = 1:6
    subplot(3, 2, i);
    hold on;
    grid on;
    plot(H.t, H.qd(i, :), 'k--', 'LineWidth', 1.1);
    plot(H.t, H.q(i, :), 'b', 'LineWidth', 1.1);
    ylabel(labels{i});
    if i > 4
        xlabel('Tempo (s)');
    end
end
sgtitle('Estado desejado e medido');

figure('Name', 'Erros da formação', 'Color', 'w');
plot(H.t, H.error_q.', 'LineWidth', 1.1);
grid on;
xlabel('Tempo (s)');
ylabel('Erro');
legend(labels, 'Location', 'best');
title('Erros no espaço do cluster');

figure('Name', 'Sinais de controle', 'Color', 'w');
subplot(2, 1, 1);
plot(H.t, H.cmd_limo.', 'LineWidth', 1.1);
grid on;
legend('v', '\omega', 'Location', 'best');
ylabel('Comando LIMO');
subplot(2, 1, 2);
plot(H.t, H.cmd_cf.', 'LineWidth', 1.1);
grid on;
legend('\phi', '\theta', 'ż', '\dot{\psi}', 'Location', 'best');
xlabel('Tempo (s)');
ylabel('Comando Crazyflie');

figure('Name', 'Obstáculo', 'Color', 'w');
plot(H.t, H.obstacle_distance, 'b', 'LineWidth', 1.2);
hold on;
grid on;
yline(cfg.obstacle_radius, 'r--', 'Raio físico');
yline(cfg.obstacle_influence_radius, 'k:', 'Zona de influência');
xlabel('Tempo (s)');
ylabel('Distância do PoI LIMO (m)');
title('Ativação da evasão');
end

function draw_circle(center, radius, style)
angle = linspace(0, 2 * pi, 100);
plot(center(1) + radius * cos(angle), center(2) + radius * sin(angle), ...
    style, 'HandleVisibility', 'off');
end

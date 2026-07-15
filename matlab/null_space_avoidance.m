function [q_dot_safe, active, distance] = null_space_avoidance(q_dot_formation, poi_xy, cfg)
% Prioriza a evasão do obstáculo e projeta a formação no espaço nulo.

offset = poi_xy - cfg.obstacle_center;
distance = norm(offset);
active = distance < cfg.obstacle_influence_radius;
q_dot_safe = q_dot_formation;

if ~active
    return;
end

if distance < 1e-6
    direction = [1; 0];
else
    direction = offset / distance;
end

span = cfg.obstacle_influence_radius - cfg.obstacle_radius;
escape_rate = cfg.obstacle_gain * max(0, cfg.obstacle_influence_radius - distance) / span;
J_obstacle = [eye(2), zeros(2, 4)];
J_obstacle_pinv = J_obstacle.';
null_projector = eye(6) - J_obstacle_pinv * J_obstacle;

q_dot_safe = J_obstacle_pinv * (escape_rate * direction) + ...
    null_projector * q_dot_formation;
end

function [q, poi_limo] = get_formation_state(p_limo, yaw_limo, p_cf, a_limo)
% Obtém q = [xf; yf; zf; rho; alpha; beta] a partir das poses dos robôs.

poi_limo = p_limo + [a_limo * cos(yaw_limo); a_limo * sin(yaw_limo); 0];
delta = p_cf - poi_limo;

rho = norm(delta);
horizontal_distance = hypot(delta(1), delta(2));
if horizontal_distance < 1e-6
    alpha = 0;
else
    alpha = atan2(delta(2), delta(1));
end
beta = atan2(delta(3), horizontal_distance);

q = [poi_limo; rho; alpha; beta];
end

import numpy as np
import numpy.typing as npt

from plotting import plot_results
from config import SimulationConfig

POI_OFFSET = 0.10
RHO_MIN = 1e-3
BETA_DESIRED = np.pi / 2.0
BEBOP_ALTITUDE = 1.5
FORMATION_RHO = 1.5

OBSTACLE_CENTER = np.array([-0.2, 0.425])
OBSTACLE_RADIUS = 0.15
OBSTACLE_INFLUENCE = 0.5

V_MAX_LIMO = 2.0
V_MAX_BEBOP_XY = 2.0
V_MAX_BEBOP_Z = 1.0
V_MAX_STATE = 3.0

FloatArray = npt.NDArray[np.floating]


def _clamp(values: FloatArray, limit: float) -> FloatArray:
    return np.clip(values, -limit, limit)


def _desired_formation(current_time: float) -> tuple[FloatArray, FloatArray]:
    """Bernoulli lemniscate in XY; constant rho, alpha, beta per project spec."""
    phase_x = 2.0 * np.pi * current_time / 40.0
    phase_y = 4.0 * np.pi * current_time / 40.0

    x_desired = 0.75 * np.sin(phase_x)
    y_desired = 0.75 * np.sin(phase_y)

    q_desired = np.array(
        [
            x_desired,
            y_desired,
            0.0,
            FORMATION_RHO,
            0.0,
            BETA_DESIRED,
        ]
    )
    q_desired_dot = np.array(
        [
            0.75 * (2.0 * np.pi / 40.0) * np.cos(phase_x),
            0.75 * (4.0 * np.pi / 40.0) * np.cos(phase_y),
            0.0,
            0.0,
            0.0,
            0.0,
        ]
    )
    return q_desired, q_desired_dot


def _apply_obstacle_null_space(q_reference: FloatArray, poi_xy: FloatArray) -> FloatArray:
    """
    Null-space projection: obstacle avoidance (priority) + formation tracking (secondary)
    in the LIMO PoI XY subspace.
    """
    offset = poi_xy - OBSTACLE_CENTER
    distance = float(np.linalg.norm(offset))
    if distance >= OBSTACLE_INFLUENCE or distance <= 1e-6:
        return q_reference

    direction = offset / distance
    jacobian_obstacle = direction.reshape(1, 2)
    jacobian_pseudo_inverse = direction.reshape(2, 1)

    clearance = distance - OBSTACLE_RADIUS
    if clearance <= 0.0:
        obstacle_rate = 0.8
    else:
        obstacle_rate = 0.4 * (1.0 / clearance - 1.0 / (OBSTACLE_INFLUENCE - OBSTACLE_RADIUS))

    primary_velocity = (jacobian_pseudo_inverse @ np.array([obstacle_rate])).reshape(2)
    null_projector = np.eye(2) - jacobian_pseudo_inverse @ jacobian_obstacle
    secondary_velocity = null_projector @ q_reference[0:2]

    modified = q_reference.copy()
    modified[0:2] = primary_velocity + secondary_velocity
    return modified


def run_simulation(config: SimulationConfig) -> None:
    dt = config.dt
    time = np.arange(0, config.t_final, dt)
    steps = len(time)

    theta_limo = np.array([0.1521, 0.0953, 0.0031, 0.9840, -0.0451, 1.6422])

    kq = np.diag(
        [
            config.kq,
            config.kq,
            config.kq * 0.83,
            config.kq * 1.25,
            config.kq * 0.83,
            config.kq,
        ]
    )
    lq = np.diag(
        [
            config.lq,
            config.lq,
            config.lq * 0.62,
            config.lq * 1.25,
            config.lq * 0.62,
            config.lq,
        ]
    )
    kd_limo = np.diag([config.kd_limo, config.kd_limo])

    # Spec: LIMO at (0.4, -0.25), heading along +X; Bebop ~30 cm to the left at 1.5 m.
    pose_limo = np.array([0.4, -0.25, 0.0])
    velocity_limo = np.array([0.0, 0.0])
    pose_bebop = np.array([0.4, -0.55, BEBOP_ALTITUDE, 0.0])
    velocity_bebop = np.array([0.0, 0.0, 0.0, 0.0])

    hist_q = np.zeros((6, steps))
    hist_qd = np.zeros((6, steps))
    hist_error = np.zeros((6, steps))
    hist_limo = np.zeros((3, steps))
    hist_bebop = np.zeros((4, steps))
    hist_poi_limo = np.zeros((3, steps))
    hist_poi_bebop = np.zeros((3, steps))

    for step in range(steps):
        current_time = time[step]
        q_desired, q_desired_dot = _desired_formation(current_time)

        x1, y1, psi1 = pose_limo
        x2, y2, z2, psi2 = pose_bebop

        poi_limo = np.array(
            [x1 + POI_OFFSET * np.cos(psi1), y1 + POI_OFFSET * np.sin(psi1), 0.0]
        )
        poi_bebop = np.array([x2, y2, z2])

        delta = poi_bebop - poi_limo
        dist_2d = np.sqrt(delta[0] ** 2 + delta[1] ** 2)

        # Spec: rho and beta are horizontal; alpha = 0 (drone altitude handled separately).
        rho = max(float(dist_2d), RHO_MIN)
        alpha = 0.0
        beta = np.arctan2(delta[1], delta[0])

        # Formation PoI is the LIMO control point (spec figure).
        q = np.array([poi_limo[0], poi_limo[1], poi_limo[2], rho, alpha, beta])

        error_q = q_desired - q
        error_q[5] = np.arctan2(
            np.sin(q_desired[5] - q[5]),
            np.cos(q_desired[5] - q[5]),
        )
        tanh_term = np.tanh(np.linalg.solve(lq, kq @ error_q))
        q_reference = q_desired_dot + lq @ tanh_term
        q_reference = _apply_obstacle_null_space(q_reference, poi_limo[0:2])

        jacobian_s = np.array(
            [
                [
                    np.cos(alpha) * np.cos(beta),
                    -rho * np.sin(alpha) * np.cos(beta),
                    -rho * np.cos(alpha) * np.sin(beta),
                ],
                [
                    np.cos(alpha) * np.sin(beta),
                    -rho * np.sin(alpha) * np.sin(beta),
                    rho * np.cos(alpha) * np.cos(beta),
                ],
                [np.sin(alpha), rho * np.cos(alpha), 0.0],
            ]
        )

        jacobian_inverse = np.block([[np.eye(3), np.zeros((3, 3))], [np.eye(3), jacobian_s]])

        robot_reference = jacobian_inverse @ q_reference

        limo_kinematics_inverse = np.array(
            [
                [np.cos(psi1), np.sin(psi1)],
                [-np.sin(psi1) / POI_OFFSET, np.cos(psi1) / POI_OFFSET],
            ]
        )
        velocity_desired_limo = _clamp(limo_kinematics_inverse @ robot_reference[0:2], V_MAX_LIMO)

        yaw_rotation = np.array(
            [
                [np.cos(psi2), np.sin(psi2), 0.0],
                [-np.sin(psi2), np.cos(psi2), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        velocity_desired_bebop_body = yaw_rotation @ robot_reference[3:6]
        velocity_desired_bebop_body[0:2] = _clamp(
            velocity_desired_bebop_body[0:2], V_MAX_BEBOP_XY
        )
        velocity_desired_bebop_body[2] = _clamp(
            velocity_desired_bebop_body[2] + 2.0 * (BEBOP_ALTITUDE - z2),
            V_MAX_BEBOP_Z,
        )
        velocity_desired_bebop = np.append(velocity_desired_bebop_body, 0.0)

        linear_velocity, angular_velocity = velocity_limo
        regression_matrix = np.array(
            [
                [linear_velocity, 0.0, angular_velocity**2, 0.0, 0.0, 0.0],
                [0.0, angular_velocity, 0.0, linear_velocity, linear_velocity * angular_velocity, angular_velocity],
            ]
        )
        control_limo = regression_matrix @ theta_limo + kd_limo @ (
            velocity_desired_limo - velocity_limo
        )

        control_bebop = velocity_desired_bebop + 1.0 * (velocity_desired_bebop - velocity_bebop)

        mass_matrix = np.array([[theta_limo[0], 0.0], [0.0, theta_limo[1]]])
        coriolis_matrix = np.array(
            [
                [theta_limo[3] * linear_velocity, theta_limo[2] * angular_velocity],
                [
                    theta_limo[4] * linear_velocity + theta_limo[5] * angular_velocity,
                    0.0,
                ],
            ]
        )

        velocity_dot_limo = np.linalg.solve(
            mass_matrix, control_limo - coriolis_matrix @ velocity_limo
        )
        velocity_limo = _clamp(velocity_limo + dt * velocity_dot_limo, V_MAX_STATE)
        velocity_bebop[:3] = _clamp(
            velocity_bebop[:3] + dt * 15.0 * (control_bebop[:3] - velocity_bebop[:3]),
            V_MAX_STATE,
        )
        velocity_bebop[2] = _clamp(velocity_bebop[2], V_MAX_BEBOP_Z)

        pose_limo[0] += dt * (velocity_limo[0] * np.cos(psi1))
        pose_limo[1] += dt * (velocity_limo[0] * np.sin(psi1))
        pose_limo[2] += dt * velocity_limo[1]

        global_velocity_bebop = yaw_rotation.T @ velocity_bebop[:3]
        pose_bebop[0:3] += dt * global_velocity_bebop
        pose_bebop[3] += dt * velocity_bebop[3]

        if not np.all(np.isfinite(pose_limo)) or not np.all(np.isfinite(pose_bebop)):
            break

        hist_q[:, step] = q
        hist_qd[:, step] = q_desired
        hist_error[:, step] = error_q
        hist_limo[:, step] = pose_limo
        hist_bebop[:, step] = pose_bebop
        hist_poi_limo[:, step] = poi_limo
        hist_poi_bebop[:, step] = poi_bebop

    plot_results(
        time=time,
        hist_q=hist_q,
        hist_qd=hist_qd,
        hist_error=hist_error,
        hist_limo=hist_limo,
        hist_bebop=hist_bebop,
        hist_poi_limo=hist_poi_limo,
        hist_poi_bebop=hist_poi_bebop,
        poi_offset=POI_OFFSET,
        config=config,
    )

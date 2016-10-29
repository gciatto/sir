from numpy import array, zeros


def extended_kalman_filter_2d(controller, believes: dict, actuators: dict, dt: float):
    if 'expected_state' in believes:
        state = believes['expected_state']
    else:
        state = zeros(3)
        believes['expected_state'] = state

    if 'state_covariance_matrix' in believes:
        covariances = believes['state_covariance_matrix']
    else:
        covariances = zeros((3, 3))
        believes['state_covariance_matrix'] = covariances

    control = controller.get_odometry()
    obstacles = controller.get_obstacles()

    _extended_kalman_filter_2d(state, covariances, control, obstacles)


def _extended_kalman_filter_2d(state, covariances, control, observations):
    pass

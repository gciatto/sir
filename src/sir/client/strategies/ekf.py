from numpy import array, matrix, zeros, diag, eye, cos, sin, sqrt, hstack, vstack
from numpy.linalg import inv

from sir.const import ODOMETRY_DX_STDEV as epsilon_x, \
    ODOMETRY_DY_STDEV as epsilon_y, \
    ODOMETRY_DYAW_STDEV as epsilon_theta
from sir.const import LASER_RANGE_STDEV as delta_rho, \
    LASER_HORIZONTAL_ANGLE_STDEV as delta_theta

Epsilon = diag((epsilon_x ** 2, epsilon_y ** 2, epsilon_theta ** 2))
Delta = diag((delta_rho ** 2, delta_theta ** 2))


def mahalonobis(x, y, covariances=None, inverse=None):
    inverse_covs = inv(covariances) if inverse is None else inverse

    v = x - y

    return sqrt(
        v.dot(inverse_covs.dot(v.transpose()))
    )


def movement(state, control):
    return state[0:3] + control[0:3]

def jacobian_of_movement_wrt_state(state, control):
    return eye(3)


def jacobian_of_movement_wrt_control(state, control):
    return eye(3)


def prediction(state, covariances, control, epsilon=Epsilon):
    state[0:3] = movement(state, control)

    jacobian_state = jacobian_of_movement_wrt_state(state, control)
    jacobian_control = jacobian_of_movement_wrt_control(state, control)
    covariances[0:3, 0:3] = jacobian_state.dot(covariances[0:3, 0:3]).dot(jacobian_state.transpose()) + \
                            jacobian_control.dot(epsilon).dot(jacobian_control.transpose())


def landmarks(state, covariances=None):
    tot = len(state)
    if tot <= 3:
        yield (-1,)
    else:
        for i in range(3, tot, 2):
            index = (i - 3) // 2
            if covariances is not None:
                yield (index, state[i:i+2], covariances[i:i+2, i:i+2])
            else:
                yield (index, state[i:i+2])


def rotation_matrix(angle):
    return matrix(
        [[cos(angle), -sin(angle)],
         [sin(angle), cos(angle)]]
    )


def inverse_observation(state, observation):
    r = observation[0]
    alpha = observation[1]
    pose = state[0:2]
    theta = state[2]
    local_vector = array((r * cos(alpha), r * sin(alpha)))
    return pose + rotation_matrix(theta).dot(local_vector)


def jacobian_of_inv_obs_wrt_state(state, observation):
    r, alpha = observation
    theta = state[2]
    return matrix(
        [[1, 0, -r * sin(alpha + theta)],
         [0, 1, r * cos(alpha + theta)]]
    )


def jacobian_of_inv_obs_wrt_sensor(state, observation):
    r, alpha = observation
    theta = state[2]
    c = cos(alpha + theta)
    s = sin(alpha + theta)
    return matrix(
        [[c, -r * s],
         [s, r * c]]
    )


def add_new_landmark(state, covariances, landmark, landmark_covariances):
    state_size = len(state)
    landmark_size = len(landmark)
    state = vstack(state, landmark)
    new_columns = zeros((state_size, landmark_size))
    covariances = hstack(covariances, new_columns)
    new_rows = hstack(zeros((landmark_size, state_size)), landmark_covariances)
    covariances = vstack(covariances, new_rows)

    return ((state_size - 3) / 2, state, covariances)


def classification(state, covariances, observation, delta=Delta, threshold=3):
    landmark_candidate = inverse_observation(state, observation)

    jacobian_state = jacobian_of_inv_obs_wrt_state(state, observation)
    jacobian_sense = jacobian_of_inv_obs_wrt_sensor(state, observation)

    state_cov = covariances[0:3, 0:3]

    z1 = jacobian_state.dot(state_cov.dot(jacobian_state.traspose()))
    z2 = jacobian_sense.dot(delta.dot(jacobian_sense.traspose()))

    candidate_covariances = z1 + z2
    candidate_inverse = inv(candidate_covariances)

    for index, position in landmarks(state):
        distance = mahalonobis(position, landmark_candidate, inverse=candidate_inverse)

        if distance <= threshold:
            return index, state, covariances

    return add_new_landmark(state, covariances, landmark_candidate, candidate_covariances)




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

    control = array(controller.get_odometry())
    obstacles = [array(o[0:3]) for o in controller.get_obstacles()]

    _extended_kalman_filter_2d(state, covariances, control, obstacles)


def _extended_kalman_filter_2d(state, covariances, control, observations):
    pass

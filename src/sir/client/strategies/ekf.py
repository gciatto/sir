from numpy import array, matrix, zeros, diag, eye, cos, sin, sqrt, hstack, vstack, append, arctan2
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


def vector(*args, **kwargs):
    a = array(*args, **kwargs)
    return a.reshape(a.size, 1)


def rearray(numpy_obj):
    return array(numpy_obj).reshape(numpy_obj.size)


def rotation_matrix(angle):
    return matrix(
        [[cos(angle), -sin(angle)],
         [sin(angle), cos(angle)]]
    )


def move(state, control):
    return state[0:3] + control[0:3]


def jacobian_of_move_wrt_state(state, control):
    return eye(3)


def jacobian_of_move_wrt_control(state, control):
    return eye(3)


def prediction(state, covariances, control, epsilon=Epsilon):
    state[0:3] = move(state, control)

    jacobian_state = jacobian_of_move_wrt_state(state, control)
    jacobian_control = jacobian_of_move_wrt_control(state, control)
    covariances[0:3, 0:3] = jacobian_state.dot(covariances[0:3, 0:3]).dot(jacobian_state.transpose()) + \
                            jacobian_control.dot(epsilon).dot(jacobian_control.transpose())

    return rearray(state), covariances


def count_landmarks(state):
    return (state.size - 3) // 2


def get_landmark(index, state, covariances=None, columns=False):
    inner_index = index * 2 + 3
    if covariances is None:
        return index, \
               state[inner_index: inner_index + 2]
    elif columns:
        return index, \
               state[inner_index: inner_index + 2], \
               covariances[inner_index: inner_index + 2, inner_index: inner_index + 2], \
               covariances[:, inner_index: inner_index + 2]
    else:
        return index, \
               state[inner_index: inner_index + 2], \
               covariances[inner_index: inner_index + 2, inner_index: inner_index + 2]


def landmarks(state, covariances=None, columns=False):
    n = count_landmarks(state)

    for i in range(0, n):
        yield get_landmark(i, state, covariances, columns)


def inv_observe(state, observation):
    r = observation[0]
    alpha = observation[1]
    pose = state[0:2]
    theta = state[2]
    local_vector = array((r * cos(alpha), r * sin(alpha)))
    return pose + rotation_matrix(theta).dot(local_vector)


def jacobian_of_inv_observe_wrt_state(state, observation):
    r, alpha = observation
    theta = state[2]
    return matrix(
        [[1, 0, -r * sin(alpha + theta)],
         [0, 1, r * cos(alpha + theta)]]
    )


def jacobian_of_inv_observe_wrt_sensor(state, observation):
    r, alpha = observation
    theta = state[2]
    c = cos(alpha + theta)
    s = sin(alpha + theta)
    return matrix(
        [[c, -r * s],
         [s, r * c]]
    )


def add_new_landmark(state, covariances, landmark, landmark_covariances):
    state_size = state.size
    landmark_size = landmark.size
    state = append(
        state,
        landmark
    )
    new_columns = zeros([
        state_size,
        landmark_size
    ])
    covariances = hstack([
        covariances,
        new_columns
    ])
    new_rows = hstack([
        zeros([
            landmark_size,
            state_size
        ]),
        landmark_covariances
    ])
    covariances = vstack([
        covariances,
        new_rows
    ])

    return (state_size - 3) // 2, state, covariances


# TODO cambiare strategia: scansionare ogni landmark per vedere se il candidato si trova all'interno di un ellissoide d'errore
def classification(state, covariances, observation, delta=Delta, threshold=3):
    landmark_candidate = inv_observe(state, observation)

    jacobian_state = jacobian_of_inv_observe_wrt_state(state, observation)
    jacobian_sense = jacobian_of_inv_observe_wrt_sensor(state, observation)

    state_cov = covariances[0:3, 0:3]

    z1 = jacobian_state.dot(state_cov.dot(jacobian_state.transpose()))
    z2 = jacobian_sense.dot(delta.dot(jacobian_sense.transpose()))

    candidate_covariances = z1 + z2
    candidate_inverse = inv(candidate_covariances)

    for index, position in landmarks(state):
        distance = mahalonobis(position, landmark_candidate, inverse=candidate_inverse)

        if distance <= threshold:
            return index, state, covariances

    return add_new_landmark(state, covariances, landmark_candidate, candidate_covariances)


def observe(state, landmark):
    x, y, theta = state[0:3]
    x_lmk, y_lmk = landmark
    dx = x_lmk - x
    dy = y_lmk - y
    q2 = dx * dx + dy * dy
    return array([
        sqrt(q2),
        arctan2(dy, dx) - theta
    ])


def jacobian_observation_wrt_state(state, landmark):
    x, y, theta = state[0:3]
    x_lmk, y_lmk = landmark
    dx = x_lmk - x
    dy = y_lmk - y
    q2 = dx * dx + dy * dy
    q = sqrt(q2)
    return matrix(
        [[-dx / q, -dy / q, 0],
         [dy / q2, -dx / q2, -1]]
    )


def jacobian_observation_wrt_landmark(state, landmark, jacobian_wrt_state=None):
    if jacobian_wrt_state is None:
        x, y, theta = state[0:3]
        x_lmk, y_lmk = landmark
        dx = x_lmk - x
        dy = y_lmk - y
        q2 = dx * dx + dy * dy
        q = sqrt(q2)
        return matrix(
            [[dx / q, dy / q],
             [-dy / q2, dx / q2]]
        )
    else:
        return jacobian_wrt_state[0:2, 0:2] * -1


def correction(state, covariances, sensor_observation, landmark_index, landmark, landmark_covariances, landmark_columns,
               delta=Delta):
    offset = sensor_observation - observe(state, landmark)

    jacobian_state = jacobian_observation_wrt_state(state, landmark)
    jacobian_landmark = jacobian_observation_wrt_landmark(state, landmark, jacobian_state)
    mini_jacobian = hstack([
        jacobian_state, jacobian_landmark
    ])
    mini_jacobian_transpose = mini_jacobian.transpose()

    landmark_columns_transpose = landmark_columns.transpose()
    mini_covariances = vstack([
        hstack([
            covariances[0:3, 0:3],
            landmark_columns[0:3, :]]
        ),
        zeros([2, 5])
    ])
    mini_covariances[3:5, 0:3] = landmark_columns_transpose[:, 0:3]
    mini_covariances[3:, 3:] = landmark_covariances
    offset_covariances = delta + mini_jacobian.dot(mini_covariances.dot(mini_jacobian_transpose))
    inv_offset_covariances = inv(offset_covariances)

    long_covariances = hstack([
        covariances[:, 0:3],
        landmark_columns
    ])
    kalman_gain = long_covariances.dot(mini_jacobian_transpose.dot(inv_offset_covariances))

    new_state = state + kalman_gain.dot(offset)
    new_covariances = covariances - kalman_gain.dot(offset_covariances.dot(kalman_gain.transpose()))

    return rearray(new_state), new_covariances


def extended_kalman_filter_2d(controller, believes: dict, actuators: dict, dt: float):
    if 'expected_state' in believes:
        state = believes['expected_state']
    else:
        state = zeros(3)
        believes['expected_state'] = state

    if 'expected_state_covariance_matrix' in believes:
        covariances = believes['expected_state_covariance_matrix']
    else:
        covariances = zeros((3, 3))
        believes['expected_state_covariance_matrix'] = covariances

    bel_odometry = believes['odometry']
    prev_update = believes['last_control_timestamp'] if 'last_control_timestamp' in believes else 0
    curr_update = bel_odometry['timestamp'] if bel_odometry is not None and 'timestamp' in bel_odometry else 0

    control = array(controller.get_odometry())
    obstacles = [array(o[0:2]) for o in controller.get_obstacles()]

    if curr_update > prev_update:
        state, covariances = _extended_kalman_filter_2d(state, covariances, control, obstacles)

    believes['expected_state'] = state
    believes['expected_state_covariance_matrix'] = covariances
    believes['last_control_timestamp'] = curr_update


def _extended_kalman_filter_2d(state, covariances, control, observations):
    state, covariances = prediction(state, covariances, control)

    for observation in observations:
        index, state, covariances = classification(state, covariances, observation)

        index, landmark, landmark_covariances, landmark_columns = get_landmark(index, state, covariances, columns=True)

        state, covariances = correction(state, covariances, observation, index, landmark, landmark_covariances, landmark_columns)

    return state, covariances

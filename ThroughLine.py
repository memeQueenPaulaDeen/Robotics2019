import numpy as np


def pose2transform(pose):

    x = pose[0]
    y = pose[1]
    theta = pose[2]

    t = np.array([
                [np.cos(theta), -np.sin(theta), x],
                [np.sin(theta),  np.cos(theta), y],
                [0,              0,             1]
                ])

    return t


def propagate_pose(pose, v, w, dt):

    # Unwrap Pose
    x = pose[0]
    y = pose[1]
    theta = pose[2]

    # do the translation first then the rotation, this is a fair assumption if
    # the time step is small enough
    x = x + v * dt * np.cos(theta)
    y = y + v * dt * np.sin(theta)
    theta = theta + w * dt
    theta = np.unwrap([theta])

    # Generate Pose
    pose = np.array([[x], [y], [theta]])

    return pose


def through_point(state, w, v_set, dt):

    # Get number of waypoints
    num_waypts = len(w[0, :])  # CHECKED

    # Add Initial Point to Beginning of Waypoint Matrix
    # Copy Final Point to End of Waypoint Matrix
    pos = np.array([[state[0]], [state[1]]])  # CHECKED
    w = np.concatenate([pos, w, w[:, -1].reshape(num_waypts - 1, 1)], axis=1)  # CHECKED

    # Iterate through Waypoints
    # Start at Second Index (First Waypoint) and End at Last Waypoint
    for j in range(1, num_waypts):

        # Obtain Waypoints
        w_prev = w[:, j-1].reshape(2, 1)  # CHECKED
        w_curr = w[:, j].reshape(2, 1)  # CHECKED
        w_next = w[:, j+1].reshape(2, 1)  # CHECKED

        # Get rover's angle relative to waypoint
        # NOTE: Values are wrapped in a matrix.
        #   i.e. 3 is represented as [3]
        #        May be needed in the future
        dx = w_curr[0] - state[0]  # CHECKED
        dy = w_curr[1] - state[1]  # CHECKED
        theta_to_waypt = np.arctan2(dy, dx)  # CHECKED

        # Determine Rover Pose
        rover_pose = np.array([state[0], state[1], state[2]])  # CHECKED
        line_pose = np.array([state[0], state[1], theta_to_waypt])  # CHECKED

        # Define params for controller
        # SHOULD BE CHANGED
        v = v_set  # P.v_const in MATLAB, global constant
        chi = line_pose[2]  # CHECKED
        chi_inf = np.radians(90)  # CHECKED
        k_p = 2
        k_t = 0.01

        # Get Direction Vectors
        q_i_prev = (w_curr - w_prev)/np.linalg.norm(w_curr - w_prev)  # CHECKED
        if j == num_waypts:
            # For the last point, there is no 'next point'
            # therefore, define the direction vectors as
            # being in the same direction, meaning that the
            # halfplane is perpendicular to the direction
            # to the final waypoint.
            q_i_curr = q_i_prev  # CHECKED
        else:
            q_i_curr = (w_next - w_curr)/np.linalg.norm(w_next - w_curr)  # CHECKED

        # Determine Normal Vector
        n = (q_i_prev + q_i_curr)/np.linalg.norm(q_i_prev + q_i_curr)  # CHECKED

        # Determine If Halfplane Is Crossed
        p = np.array([[state[0]], [state[1]]])  # CHECKED
        r = w_curr  # CHECKED

        # While halfplane condition is not met
        # keep going and updating state etc.
        while np.matmul((p - r).transpose(), n)[0] < 0:  # CHECKED

            # Get error from line
            t_body = pose2transform(rover_pose)  # CHECKED
            t_line = pose2transform(line_pose)  # CHECKED
            line_t_body = np.matmul(np.linalg.inv(t_line), t_body)  # CHECKED
            e_y = line_t_body[1, 2]  # CHECKED

            # Get command angle and current angle.
            theta = rover_pose[2]  # CHECKED
            theta_cmd = chi - np.arctan(k_t*e_y)*(2/np.pi)*chi_inf  # CHECKED
            e_theta = np.unwrap([theta_cmd - theta])  # CHECKED (DISCONTINUITIES?)

            # Get angular velocity to reach line
            w = k_p*e_theta  # CHECKED

            # Increment pose and draw car
            rover_pose = propagate_pose(rover_pose, v, w, dt)  # CHECKED
            state = np.append(rover_pose, (v, w, state[5] + dt)).reshape(6, 1)  # CHECKED

            # Update Current Location
            p = np.array([[state[0]], [state[1]]])

        return state

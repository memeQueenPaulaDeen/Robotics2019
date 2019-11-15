import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np
# import time
import get_scans_vector as scan
import Particle_Filter as Pf


def show_map(figure):
    figure.canvas.draw()
    figure.canvas.flush_events()


def update_map(plot, gridmap):
    plot.imshow(gridmap, 'Greys')


def buffer_array(array, size):
    # In the case that the specified file is 0 and
    # the for loop is never entered
    if size == 0:
        return array
    else:
        # Switcharoo
        array = 1 - array

        num_rows = array.shape[0]  # number of rows (size of Y direction)
        num_cols = array.shape[1]  # number of columns (size of X)

        ones_row = np.ones((1, num_cols), dtype=np.int)  # add to top and bottom
        ones_col = np.ones(num_rows, dtype=np.int)  # add to left and right

        buffered_array = []
        for k in range(size):
            # arr1 - shift up (remove first row, add bottom row)
            arr1 = np.delete(array, 0, axis=0)  # removes first row
            arr1 = np.insert(arr1, (num_rows - 1), ones_row, axis=0)  # zero adds to the row axis

            # arr2 - shift down (remove last row, add first row)
            arr2 = np.delete(array, (num_rows - 1), axis=0)  # removes last row
            arr2 = np.insert(arr2, 0, ones_row, axis=0)

            # arr3 - shift right (remove last column, add first column)
            arr3 = np.delete(array, (num_cols - 1), axis=1)
            arr3 = np.insert(arr3, 0, ones_col, axis=1)

            # arr4 - shift left (remove first column, add last column)
            arr4 = np.delete(array, 0, axis=1)
            arr4 = np.insert(arr4, (num_cols - 1), ones_col, axis=1)

            # arr5 shift up one AND left one, arr1 is already shifted up, now shift left
            arr5 = np.delete(arr1, 0, axis=1)
            arr5 = np.insert(arr5, (num_cols - 1), ones_col, axis=1)

            # arr6 shift up one AND right one -- shift arr1 right
            arr6 = np.delete(arr1, (num_cols - 1), axis=1)
            arr6 = np.insert(arr6, 0, ones_col, axis=1)

            # arr7 shift down one AND left one -- shift arr2 left
            arr7 = np.delete(arr2, 0, axis=1)
            arr7 = np.insert(arr7, (num_cols - 1), ones_col, axis=1)

            # arr8 shift down one AND right one -- shift arr2 right
            arr8 = np.delete(arr2, (num_cols - 1), axis=1)
            arr8 = np.insert(arr8, 0, ones_col, axis=1)

            buffered_array = array * arr1 * arr2 * arr3 * arr4 * arr5 * arr6 * arr7 * arr8
            array = buffered_array

        # Switcharoo
        buffered_array = 1 - buffered_array

        return buffered_array


if __name__ == "__main__":

    file_name = 'test_map2.csv'
    cell_resolution = 50
    num_scans = 8
    num_particles = 1000
    resampling = 0.1
    sigma_measure = 1  #50
    sigma_pose = 1  #10
    sigma_angle = 1  #4
    sigma_noise = 1  #30

    # Instantiate Particle Filter
    a = Pf.ParticleFilter(file_name, cell_resolution, num_scans, num_particles, resampling, sigma_measure,
                          sigma_pose, sigma_angle)

    # Define Start Pose
    x = 5*cell_resolution
    y = 45*cell_resolution
    th = 90

    # Define Deltas (Not Constant in Real Life)
    dx = 1*cell_resolution
    dy = 1*cell_resolution
    dth = 0

    for i in range(50):
        x = x + dx
        y = y + dy
        th = th + dth

        measures = scan.get_scans_vector(a.grid_map, a.n_angles, a.cell_size, x, y, th)
        measures = measures + np.random.normal(0, sigma_noise, a.n_angles)

        a.update_state(dx, dy, dth, measures)
        est_x = a.x_pos
        est_y = a.y_pos
        est_th = a.th_pos

        print("Actual Pose: (", x, ", ", y, ", ", th, ")")
        print("Estimated Pose: (", est_x, ", ", est_y, ", ", est_th, ")")

import signal
import time
import PID
import numpy as np
import RPi.GPIO as GPIO
import Follower




class Line():

    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def get_pose(self):
        
        return np.array([self.x, self.y, self.theta])
    
class WaypointFollower():


    pos = None
    motors = None
    PIDleft = None
    PIDRight = None
    waypoint_array = None

    def __init__(self,pos,motors,PIDleft,PIDRight,waypoint_array):
        self.pos = pos
        self.motors = motors
        self.PIDleft = PIDleft
        self.PIDRight = PIDRight

        start_position = np.array([pos.start_x_cm, pos.start_y_cm])
        p = None
        self.waypoint_array = np.vstack((start_position, waypoint_array))
        self.waypoint_array = np.vstack((self.waypoint_array, self.waypoint_array[-1]))


    def followWaypoints(self):

        try:


            # Define Line Following Parameters
            phi_dot_set = 6
            chi_inf = np.radians(90)
            k_theta_cmd = 0.03
            k_heading_change = 1
            follower = Follower.Follower(phi_dot_set, self.pos.encoder, self.PIDleft, self.PIDRight, self.motors, chi_inf, k_theta_cmd,
                                         k_heading_change)

            # Define Waypoints
            start_position = np.array([self.pos.start_x_cm, self.pos.start_y_cm])
            num_waypoints = len(self.waypoint_array) - 2

            # Iterate Through Waypoints
            for i in range(1, num_waypoints + 1):

                # Define Current Set of Waypoints
                w_prev = self.waypoint_array[i - 1, :]
                w_curr = self.waypoint_array[i, :]
                w_next = self.waypoint_array[i + 1, :]

                # Calculate Line Properties and Generate Line Between Waypoints
                start_x = w_prev[0]
                start_y = w_prev[1]
                dx = w_curr[0] - w_prev[0]
                dy = w_curr[1] - w_prev[1]
                theta_line = np.arctan2(dy, dx)
                line = Line(start_x, start_y, theta_line)

                # Calculate Normal Vectors
                q_i_prev = (w_curr - w_prev) / np.linalg.norm(w_curr - w_prev)
                if i == num_waypoints:
                    q_i_curr = q_i_prev
                else:
                    q_i_curr = (w_next - w_curr) / np.linalg.norm(w_next - w_curr)

                n = (q_i_prev + q_i_curr) / np.linalg.norm(q_i_prev + q_i_curr)

                if i == 1:
                    p = start_position
                else:
                    p = np.array([self.pos.encoder.x_inertial, self.pos.encoder.y_inertial])

                r = w_curr

                # print("q_prev = " + str(q_i_prev))
                # print("q_curr = " + str(q_i_curr))
                # print("p = " + str(p))
                # print("r = " + str(r))
                # print("n = " + str(n))
                # print("Halfplane -> " + str(np.matmul((p - r).transpose(), n)))

                while np.matmul((p - r).transpose(), n) + 20 < 0:
                    # Follow Line
                    #follower.followLine(line, [self.pos.encoder.x_inertial, self.pos.encoder.y_inertial, self.pos.encoder.theata])
                    #follower.followLine(line, [self.pos.localizationClient.x_cm, self.pos.localizationClient.y_cm, self.pos.localizationClient.th])
                    follower.followLine(line, [self.pos.X_fused, self.pos.Y_fused, self.pos.TH_fused])


                    print("Encoder: " + str(self.pos.encoder.x_inertial) + " "+  str(self.pos.encoder.y_inertial) + " " + str(self.pos.encoder.theata))
                    print("Localizaton: " + str(self.pos.localizationClient.x_cm) +" " +str(self.pos.localizationClient.y_cm)+ " "+ str (self.pos.localizationClient.th))
                    # Print Stuff
                    # print("p = " + str(p))
                    # print("r = " + str(r))
                    # print("n = " + str(n))
                    #print("Halfplane -> " + str(np.matmul((p - r).transpose(), n)))

                    # Update Position
                    p = [self.pos.X_fused, self.pos.Y_fused]

                    # Sleep
                    time.sleep(.2)

                # Print Out Update
                print("Waypoint " + str(i) + " reached!")

            self.motors.brake()
            time.sleep(1)
            self.motors.off()

        except KeyboardInterrupt:
            GPIO.cleanup()
            print("Killed")
            self.motors.off()
            exit(1)
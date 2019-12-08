import signal
import time
import PID
import numpy as np
import RPi.GPIO as GPIO

import Encoder
import Motor
import Follower




class Line():

    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def get_pose(self):
        
        return np.array([self.x, self.y, self.theta])
    
if __name__ == "__main__":
    
    try:
        # Init Encoders
        encoder = Encoder.Encoder()
        encoder.start()
        
        # Init Motors
        motors = Motor.Motor()
        
        # Init PID Controllers
        PIDleft = PID.PID(Kp = 0.6, Ki = 0.25, Kd = 0)
        PIDRight = PID.PID(Kp = 0.7, Ki = 0.3, Kd = 0)
        
        # Define Line Following Parameters
        phi_dot_set = 6
        chi_inf = np.radians(90)
        k_theta_cmd = 0.02
        k_heading_change = 1
        Follower = Follower.Follower(phi_dot_set, encoder, PIDleft, PIDRight, motors, chi_inf, k_theta_cmd, k_heading_change)
        
        # Define Waypoints
        start_position = np.array([0, 0])
        p = None
        waypoint_array = np.array([start_position, [80, 0],[80, -190], [80, -190]])
        num_waypoints = len(waypoint_array) - 2
        
        # Iterate Through Waypoints
        for i in range(1, num_waypoints + 1):
            
            # Define Current Set of Waypoints
            w_prev = waypoint_array[i-1, :]
            w_curr = waypoint_array[i, :]
            w_next = waypoint_array[i+1, :]

            # Calculate Line Properties and Generate Line Between Waypoints
            start_x = w_prev[0]
            start_y = w_prev[1]
            dx = w_curr[0] - w_prev[0]
            dy = w_curr[1] - w_prev[1]
            theta_line = np.arctan2(dy, dx)
            line = Line(start_x, start_y, theta_line)
                        
            # Calculate Normal Vectors
            q_i_prev = (w_curr - w_prev)/np.linalg.norm(w_curr - w_prev)
            if i == num_waypoints:
                q_i_curr = q_i_prev
            else:
                q_i_curr = (w_next - w_curr)/np.linalg.norm(w_next - w_curr)
            
            n = (q_i_prev + q_i_curr)/np.linalg.norm(q_i_prev + q_i_curr)

            if i == 1:
                p = start_position
            else:
                p = np.array([encoder.x_inertial,encoder.y_inertial])

            r = w_curr
            
            print("q_prev = " + str(q_i_prev))
            print("q_curr = " + str(q_i_curr))
            print("p = " + str(p))
            print("r = " + str(r))
            print("n = " + str(n))
            print("Halfplane -> " + str(np.matmul((p-r).transpose(), n)))
                        
            while np.matmul((p-r).transpose(), n) + 20 < 0:
                
                # Follow Line
                Follower.followLine(line,[encoder.x_inertial,encoder.y_inertial,encoder.theata])
                
                # Print Stuff
                # print("p = " + str(p))
                # print("r = " + str(r))
                # print("n = " + str(n))
                print("Halfplane -> " + str(np.matmul((p-r).transpose(), n)))
                
                # Update Position
                p = [encoder.x_inertial, encoder.y_inertial]
                
                # Sleep
                time.sleep(.2)
            
            # Print Out Update
            print("Waypoint " + str(i) + " reached!")
            
        motors.brake()
        time.sleep(1)
        motors.off()

    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Killed")
        motors.off()
        exit(1)
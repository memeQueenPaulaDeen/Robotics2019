import numpy as np
import Encoder
import PID

class Follower:

    phiDotSet = 0
    encoder = None
    PIDleft = None
    PIDright =None
    motors = None
    phiDotMax = 10
    phiDotMin = 2



    def __init__(self,phiDotSet,encoder,PIDLeft,PIDRight,motors, chi_inf, k_theta_cmd, k_heading_change):
        self.phiDotSet = phiDotSet
        self.encoder = encoder
        self.PIDleft = PIDLeft
        self.PIDright = PIDRight
        self.motors = motors
        self.chi_inf = chi_inf
        self.k_theta_cmd = k_theta_cmd
        self.k_heading_change = k_heading_change
    
    def followLine(self, line, pose):
        thIDX = 2

        # Determine Cross-Track Error
        err_y = self.getDistFromLineAsPos(line, pose)[1]
        print("cross track err " + str(err_y))
        
        # Get Theta-Command
        thCMD = line.theta - np.arctan(self.k_theta_cmd*err_y)*(2/np.pi)*self.chi_inf#in rad
        print("Theta CMD: " + str(np.degrees(thCMD)))
        
        # Get Current Theta from Encoders
        thCurent = pose[thIDX]
        
        # Change Heading Command
        self.changeHeading(thCMD,thCurent,self.k_heading_change)
        

    def changeHeading(self,thCMD,thCurent,kHeadingChange):
        th_err = thCMD - thCurent
        errNormed = np.arctan2(np.sin(th_err),np.cos(th_err))
        W_CMD = errNormed * kHeadingChange

        phiDotRight = self.phiDotSet + W_CMD*self.encoder.wheelBaseCm/(2*self.encoder.wheelRadiusCm)
        phiDotLeft = self.phiDotSet - W_CMD*self.encoder.wheelBaseCm/(2*self.encoder.wheelRadiusCm)

        if phiDotRight > self.phiDotMax:
            phiDotRight = self.phiDotMax
        if phiDotRight < self.phiDotMin:
            phiDotRight = self.phiDotMin

        if phiDotLeft > self.phiDotMax:
            phiDotLeft = self.phiDotMax
        if phiDotLeft < self.phiDotMin:
            phiDotLeft = self.phiDotMin

        self.PIDleft.control(phiDotLeft, self.encoder.phiDotLeft, self.motors.setPhiDotDesiredLeft)
        self.PIDright.control(phiDotRight, self.encoder.phiDotRight, self.motors.setPhiDotDesiredRight)

    def getDistFromLineAsPos(self, line, pose):
        xIDX = 0
        yIDX = 1
        thIDX = 2

        ITL = self.p2t(line.x, line.y, line.theata)
        ITB = self.p2t(pose[xIDX], pose[yIDX], pose[thIDX])
        LTB = np.matmul(np.linalg.inv(ITL), ITB)
        # print(LTB)
        pose_err = self.t2p(LTB)
        return pose_err

    def getDistFromLineAsPos_ENC(self, line, encoder):
        ITL = self.p2t(line.x,line.y,line.theta)
        ITB = self.p2t(encoder.x_inertial,encoder.y_inertial,encoder.theata)
        LTB = np.matmul(np.linalg.inv(ITL),ITB)
        #print(LTB)
        pose_err = self.t2p(LTB)
        return pose_err

    def p2t(self,x, y, th):
        return np.array([[np.cos(th), -np.sin(th), x],[ np.sin(th), np.cos(th), y],[ 0, 0, 1]])

    def t2p(self,T):
        return np.array([T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])])

    def rot(self,th):
        return np.array([[np.cos(th),-np.sin(th),0],[np.sin(th),np.cos(th),0],[1,1,1]])

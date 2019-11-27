import example_motor

class PID:

    error = 0
    #cumErrList = []
    cumErr = 0
    derErr = 0
    Kp = 0
    Ki = 0
    Kd = 0

    def control(self,desired,mesured,controlTarget):
        olderr = self.error
        self.error = desired - mesured
        self.derErr = self.error - olderr
        self.cumErr = self.cumErr + self.error
        #print(desired+self.Kp*self.error+self.Ki*self.cumErr+self.Kd*self.derErr)
        controlTarget(desired+self.Kp*self.error+self.Ki*self.cumErr+self.Kd*self.derErr)



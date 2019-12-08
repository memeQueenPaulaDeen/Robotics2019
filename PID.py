class PID:

    error = 0
    cumErr = 0
    derErr = 0
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def control(self,desired,mesured,controlTarget):
        olderr = self.error
        self.error = desired - mesured
        self.derErr = self.error - olderr
        self.cumErr = self.cumErr + self.error
        #print(desired+self.Kp*self.error+self.Ki*self.cumErr+self.Kd*self.derErr)
        controlTarget(desired+self.Kp*self.error+self.Ki*self.cumErr+self.Kd*self.derErr)



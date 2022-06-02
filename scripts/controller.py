import numpy as np


class PIDController:
    # MaxTurnSpeed is fastest turn speed in degrees/s, #dt is timestep in s
    def __init__(self, setPoint, Kp, Ki, Kd, Ks, maxCommand):
        self.setPoint = setPoint
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ks = Ks
        self.error = self.lastError = 0
        self.derivativeError = self.integralError = self.saturationError = 0
        self.maxCommand = maxCommand

    def updateError(self, currentState, dt):  # update error and setpoint values
        self.error = self.setPoint - currentState  # get error
        self.integralError += (self.error - self.saturationError * self.Ks) * dt  # get cumulative error
        # get derivative of error
        self.derivativeError = (self.error - self.lastError) / dt
        self.lastError = self.error  # save current error

    def getError(self):
        return self.error

    # maybe create a ramp to avoid integral windup
    def updateSetpoint(self, newSetPoint):
        self.setPoint = newSetPoint

    def evaluate(self):  # return command value
        out = self.Kp * self.error + self.Ki * self.integralError + self.Kd * self.derivativeError

        if abs(out) > self.maxCommand:
            outSaturated = np.copysign(self.maxCommand, out)
        else:
            outSaturated = out

        self.saturationError = out - outSaturated

        return outSaturated

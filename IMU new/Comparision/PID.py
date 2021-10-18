import math

class PID:

    def __init__(self, p_gain, i_gain, d_gain):
        self.last_error = 0.0
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.i_error = 0.0

    def Error(self, input, target):
        return (target - input)


    def Compute(self, input, target, dt):
        error = self.Error(input, target)

        p_error = error

        self.i_error += (error + self.last_error) * dt
        i_error = self.i_error

        d_error = (error - self.last_error) / dt

        p_output = self.p_gain * p_error
        i_output = self.i_gain * i_error
        d_output = self.d_gain * d_error
        
        # Store last error for integral error and differential error calculation
        error = p_output + i_output + d_output
        self.last_error = error
        
        # Return the output, which has been tuned to be the increment / decrement in ESC PWM
        return p_output, i_output, d_output


# Subclass of PID
class YAW_PID(PID):

    def Error(self, input, target):
        #-------------------------------------------------------------------------------------------
        # target and input are in the 0 - 2 pi range.  This is asserted. Results are in the +/- pi
        # range to make sure we spin the shorted way.
        #-------------------------------------------------------------------------------------------
        assert (abs(input) <= math.pi), "yaw input out of range %f" % math.degrees(input)
        assert (abs(target) <= math.pi), "yaw target out of range %f" % math.degrees(target)
        error = ((target - input) + math.pi) % (2 * math.pi) - math.pi
        return error




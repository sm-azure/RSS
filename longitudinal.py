#Compute longituditnal safety metics
# Definition 1 (Safe longitudinal distance — same direction) A longitudinal distance between a car c_r that drives
# behind another car c_f , where both cars are driving at the same direction, is safe w.r.t. a response time t_response if for any
# braking of at most a_max_brake, performed by c_f , if c_r will accelerate by at most a_max_accel during the response time,
# and from there on will brake by at least a_min_brake until a full stop then it won’t collide with c_f .

import math

class RSS_Longitudinal:

    def __init__(self, t_response, a_max_brake, a_max_accel, a_min_brake):
        self.t_response = t_response
        self.a_max_brake = a_max_brake
        self.a_max_accel = a_max_accel
        self.a_min_brake = a_min_brake
    

    # compute min distance required given velocities of the cars v_f and v_r
    def _min_distance(self, v_f, v_r):
        c_r_accel_stage = v_r * self.t_response + 0.5 * self.a_max_accel * self.t_response **2 
        c_r_decel_stage = (v_r + self.t_response * self.a_max_accel)**2 / 2 * self.a_min_brake
        c_f_decel_stage = v_f ** 2 / 2 * self.a_max_brake

        s = c_r_accel_stage + c_r_decel_stage - c_f_decel_stage 

        return s
    
    # check if given c_f, c_r params is RSS satisfied given coordinate positions and velocities
    def check_RSS_Longitidinal(self, c_f_x, c_f_y, c_f_v, c_r_x, c_r_y, c_r_v):
        s_required = self._min_distance(c_f_v, c_r_v)
        s_actual = math.sqrt((c_f_x-c_r_x)**2 - (c_f_y-c_r_y)**2)

        return max(0, s_actual-s_required)  # safe if value is > 0
    


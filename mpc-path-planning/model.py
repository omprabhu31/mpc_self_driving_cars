import do_mpc
import numpy as np
from casadi import *
from casadi.tools import *

import sys
import globals

sys.path.append("../../")


class simple_bicycle_model:
    def __init__(self, reference_path, length, width, Ts):
        # car parameters
        self.length = length
        self.width = width

        # reference and safety margins
        self.reference_path = reference_path
        self.safety_margin = self._compute_safety_margin()

        # get current waypoint by referencing the particular ID from the reference path
        self.wp_id = 0
        self.current_waypoint = self.reference_path.waypoints[self.wp_id]

        # model
        self.Ts = Ts

    def model_setup(self):
        '''
        function to define the model type, and state & input variables for the optimizer
        '''
        model_type = "continuous"   # this can either be 'discrete' or 'continuous'
        self.model = do_mpc.model.Model(model_type)

        # optimization variables for vehicle states:
        pos_x = self.model.set_variable(var_type="_x", var_name="pos_x")
        pos_y = self.model.set_variable(var_type="_x", var_name="pos_y")
        psi = self.model.set_variable(var_type="_x", var_name="psi")
        vel = self.model.set_variable(var_type="_x", var_name="vel")
        a_y = self.model.set_variable(var_type="_x", var_name="a_y")

        # optimization variables for vehicle inputs:
        acc = self.model.set_variable(var_type="_u", var_name="acc")
        delta = self.model.set_variable(var_type="_u", var_name="delta")

        # reference data using time-varing parameters data type
        x_ref = self.model.set_variable(var_type="_tvp", var_name="x_ref")
        y_ref = self.model.set_variable(var_type="_tvp", var_name="y_ref")
        psi_ref = self.model.set_variable(var_type="_tvp", var_name="psi_ref")
        vel_ref = self.model.set_variable(var_type="_tvp", var_name="vel_ref")
        ey_lb = self.model.set_variable(var_type="_tvp", var_name="ey_lb")
        ey_ub = self.model.set_variable(var_type="_tvp", var_name="ey_ub")

        # optimization variables for tracking error:
        psi_diff = (fmod(psi - psi_ref + np.pi, 2 * np.pi) - np.pi)
        self.model.set_expression('psi_diff', psi_diff)

        self.model.set_rhs("pos_x", vel * cos(psi))
        self.model.set_rhs("pos_y", vel * sin(psi))
        self.model.set_rhs("psi", vel * delta / self.length)
        self.model.set_rhs("vel", acc)
        self.model.set_rhs("a_y", vel * sin(psi_diff))

        self.model.setup()

    def _compute_safety_margin(self):
        '''
        function to compute the safety margin: this is set to roughly 0.7 times the width of the car
        '''
        return self.width / np.sqrt(2)

    def get_current_waypoint(self):
        '''
        function to update the current waypoint based on the distance of the vehicle from the next waypoint
        '''
        # compute the sum of the path length traveled so far, and find out if it is greater than the distance covered so far
        sum_length = np.cumsum(self.reference_path.segment_lengths)
        find_greater_value = sum_length > globals.s

        # find the index of the next waypoint based on whether vehicle has crossed the current waypoint
        next_wp_id = find_greater_value.searchsorted(True)
        prev_wp_id = next_wp_id - 1
        s_next = sum_length[next_wp_id]
        s_prev = sum_length[prev_wp_id]

        # update the waypoint based if the distance of the vehicle from the next waypoint is less than the distance from the previous waypoint
        if np.abs(globals.s - s_next) < np.abs(globals.s - s_prev):
            self.wp_id = next_wp_id
            self.current_waypoint = self.reference_path.waypoints[next_wp_id]
        else:
            self.wp_id = prev_wp_id
            self.current_waypoint = self.reference_path.waypoints[prev_wp_id]

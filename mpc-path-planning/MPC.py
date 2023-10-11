import globals
import numpy as np
import do_mpc
from casadi import *
from casadi.tools import *

import sys

sys.path.append('../../')


class MPC:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.model = vehicle.model

        # define the prediction horizon and sampling time
        self.pred_horizon = 15
        globals.pred_horizon = self.pred_horizon 
        self.Ts = 0.05
        self.length = 0.12
        self.width = 0.06

        self.current_prediction = None

        # define mpc controller characteristics
        self.mpc = do_mpc.controller.MPC(self.model)
        setup_mpc = {
            'n_robust': 0,
            'n_horizon': self.pred_horizon,
            't_step': self.Ts,
            'state_discretization': 'collocation',
            'store_full_solution': True,
        }
        self.mpc.set_param(**setup_mpc)

        # define the constraints and objective function
        self.objective_function()
        self.constraints()

        # provide time-varing parameters and setup the mpc controller
        self.tvp_template = self.mpc.get_tvp_template()
        self.mpc.set_tvp_fun(self.get_tvp_data)

        self.mpc.setup()

    def get_tvp_data(self, t_now):
        '''
        function to provide data regarding the time-varying parameters 
        '''
        ey_ub, ey_lb, _ = self.update_new_bound()
        for k in range(self.pred_horizon):
            # extract information from current waypoint
            current_waypoint = self.vehicle.reference_path.get_waypoint(
                self.vehicle.wp_id + k
            )
            self.tvp_template['_tvp', k, 'x_ref'] = current_waypoint.x
            self.tvp_template['_tvp', k, 'y_ref'] = current_waypoint.y
            self.tvp_template['_tvp', k, 'psi_ref'] = current_waypoint.psi
            self.tvp_template['_tvp', k, 'ey_lb'] = ey_lb[k]
            self.tvp_template['_tvp', k, 'ey_ub'] = ey_ub[k]
            if current_waypoint.v_ref is not None:
                self.tvp_template['_tvp', k,
                                  'vel_ref'] = current_waypoint.v_ref
            else:
                self.tvp_template['_tvp', k, 'vel_ref'] = 0

        return self.tvp_template

    def objective_function(self):
        '''
        function to define the objective function for the optimization problem
        '''
        lterm = (100000 * (self.model.x['a_y'] - (self.model.tvp['ey_lb'] + self.model.tvp['ey_ub']) / 2) ** 2 + self.model.aux['psi_diff'] ** 2
        )
        mterm = (100000 * (self.model.x['a_y'] - (self.model.tvp['ey_lb'] + self.model.tvp['ey_ub']) / 2) ** 2 + 0.1 * (self.model.x['vel'] - self.model.tvp['vel_ref']) ** 2)

        self.mpc.set_objective(mterm, lterm)

    def constraints(
        self, vel_bound=[0.0, 1.0], reset=False
    ):
        # states constraints
        self.mpc.bounds['lower', '_x', 'pos_x'] = -np.inf
        self.mpc.bounds['upper', '_x', 'pos_x'] = np.inf
        self.mpc.bounds['lower', '_x', 'pos_y'] = -np.inf
        self.mpc.bounds['upper', '_x', 'pos_y'] = np.inf
        self.mpc.bounds['lower', '_x', 'psi'] = - 2 * np.pi
        self.mpc.bounds['upper', '_x', 'psi'] = 2 * np.pi
        self.mpc.bounds['lower', '_x', 'vel'] = vel_bound[0]
        self.mpc.bounds['upper', '_x', 'vel'] = vel_bound[1]
        self.mpc.bounds['lower', '_x', 'a_y'] = -2
        self.mpc.bounds['upper', '_x', 'a_y'] = 2

        # input constraints
        self.mpc.bounds['lower', '_u', 'acc'] = -0.5
        self.mpc.bounds['upper', '_u', 'acc'] = 0.5
        self.mpc.bounds['lower', '_u', 'delta'] = -1
        self.mpc.bounds['upper', '_u', 'delta'] = 1

        if reset is True:
            self.mpc.setup()

    def update_new_bound(self, ay_max=4.0):
        # Compute dynamic constraints on a_y
        ey_ub, ey_lb, _ = self.vehicle.reference_path.update_path_constraints(
            self.vehicle.wp_id + 1,
            globals.pred_horizon,
            2 * self.vehicle.safety_margin,
            self.vehicle.safety_margin,
        )

        # constrain the maximum speed
        vel_ub = np.sqrt(ay_max / (1e-12))

        return ey_ub, ey_lb, vel_ub

    def get_control(self, x0):
        '''
        function to get the current waypoint and compute the control input
        '''
        self.vehicle.get_current_waypoint()
        u0 = self.mpc.make_step(x0)

        return np.array([u0[0], u0[1]])

    def update_distance(self, states):
        '''
        function to compute vehicle velocity and distance travelled along the reference path
        '''
        vel = states[3]

        s_dot = vel * np.cos(self.mpc.data['_aux', 'psi_diff'][0])
        globals.s += s_dot * self.Ts

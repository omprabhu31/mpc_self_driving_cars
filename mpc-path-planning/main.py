import globals
import numpy as np
import matplotlib.pyplot as plt
import do_mpc
from casadi import *
from casadi.tools import *

import sys

from model import simple_bicycle_model
from map import Map, Obstacle
from reference_path import ReferencePath
from simulator import Simulator
from MPC import MPC

sys.path.append('../../')


def environment_setup(map):

    # Load map file
    map = Map(file_path=map, origin=[-1, -2], resolution=0.005)

    # Specify waypoints
    waypoints_x = [-0.75, -0.25, -0.25, 0.25, 0.25, 1.25, 1.25, 0.75, 0.75, 1.25, 1.25, -0.75, -0.75, -0.25]
    waypoints_y = [-1.5, -1.5, -0.5, -0.5, -1.5, -1.5, -1, -1, -0.5, -0.5, 0, 0, -1.5, -1.5]

    # Specify path resolution
    path_resolution = 0.05  # m / wp

    # Create smoothed reference path
    reference_path = ReferencePath(
        map,
        waypoints_x,
        waypoints_y,
        path_resolution,
        smoothing_distance=5,
        max_width=0.23,
        circular=True,
    )

    # add obstacles to the map - more can be added if necessary
    obs1 = Obstacle(cx=0.0, cy=0.05, radius=0.05)
    obs2 = Obstacle(cx=-0.85, cy=-0.5, radius=0.07)
    obs3 = Obstacle(cx=-0.75, cy=-1.5, radius=0.06)
    obs4 = Obstacle(cx=-0.35, cy=-1.0, radius=0.08)
    obs5 = Obstacle(cx=0.35, cy=-1.0, radius=0.02)
    obs6 = Obstacle(cx=0.78, cy=-1.47, radius=0.06)
    obs7 = Obstacle(cx=0.73, cy=-0.9, radius=0.03)
    obs8 = Obstacle(cx=1.2, cy=0.0, radius=0.04)
    obs9 = Obstacle(cx=0.67, cy=-0.05, radius=0.07)
    map.add_obstacles([obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9])

    return reference_path


def MPC_Problem_setup(reference_path, ay_max=4.0, a_min=-1, a_max=1):
    # model setup
    vehicle = simple_bicycle_model(
        length=0.12, width=0.06, reference_path=reference_path, Ts=0.05
    )
    vehicle.model_setup()

    controller = MPC(vehicle)

    sim = Simulator(vehicle)

    # compute the vehicle speed profile based on speed and acceleration constraints
    speed_constraints = {
        'a_min': a_min,
        'a_max': a_max,
        'v_min': 0.0,
        'v_max': 1.0,
        'ay_max': ay_max,
    }
    vehicle.reference_path.compute_speed_profile(speed_constraints)

    return vehicle, controller, sim


if __name__ == '__main__':

    ''' User settings: '''
    show_animation = True

    map = 'maps/sim_map.png'

    # set up the map/ assign the waypoints/ add obstacles
    reference_path = environment_setup(map)

    # initiate the vehicle, controller and simulator objects
    vehicle, controller, sim = MPC_Problem_setup(reference_path)

    # set initial state
    x0 = np.array([vehicle.reference_path.waypoints[0].x, vehicle.reference_path.waypoints[0].y,
                   vehicle.reference_path.waypoints[0].psi, 0.3, 0])
    controller.mpc.x0 = x0
    sim.simulator.x0 = x0
    controller.mpc.set_initial_guess()

    '''
    Main MPC Loop
    '''
    t = 0
    while globals.s < reference_path.length:
        # get control inputs and update the distance covered by the vehicle
        u = controller.get_control(x0)
        x0 = sim.simulator.make_step(u)
        controller.update_distance(x0)

        # plot the path and update the driveable area of the environment
        x_predicted = controller.mpc.data.prediction(('_x', 'pos_x'), t_ind=t)[0]
        y_predicted = controller.mpc.data.prediction(('_x', 'pos_y'), t_ind=t)[0]
        reference_path.show(x_predicted, y_predicted)
        sim.show(x0)

        # update boundary constraints for the next time step
        controller.constraints()
        t += 1

        plt.axis('off')
        plt.pause(0.001)

    input('Press any key to exit.')

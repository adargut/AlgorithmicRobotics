import math
import numpy as np
import matplotlib.pylab as plt


class Util:

    @staticmethod
    def dist(a, b):
        """
        Simple function to calculate distance
        """

        return np.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def minus_inf():
        """
        Returns minus infinity
        """

        return math.inf * (-1)

    @staticmethod
    def scene_dims():
        """
        Returns properties of scene
        """

        return np.zeros([int(600 * 1.0), int(600 * 1.0), 3], dtype=np.uint8)


class Obstacle:
    """
    Obstacles are taken to be circles, defined by midpoint and radius
    """

    def __init__(self, r, x, y):
        self.r = r  # radius
        self.x = x  # x coord
        self.y = y  # y coord

    @staticmethod
    def obs_to_tuple(self, obs):
        """
        Convert an obstacle to tuple
        """

        return obs.x, obs.y

    @staticmethod
    def generate_random_obstacle():



class Scene:

    def __init__(self):
        self.dims = Util.scene_dims()
        self.obstacles = []

    def _generate_random_scene(self):



class Robot:

    def __init__(self, src, dst):
        """
        Parameters of the robot
        """

        self.pos = src  # current config
        self.util = Util()  # utilities
        self.theta = 0.0  # orientation
        self.target = dst  # current target
        self.radius = 0.5  # radius of robot
        self.max_speed = 0.01  # max speed of robot
        self.max_acc = 0.01  # maximum acceleration of robot
        self.left_vel = 0.0  # left velocity
        self.right_vel = 0.0  # right velocity
        self.clearance = 0.01  # clearance from obstacles
        self.predict_time = 2.0  # time step of trajectory prediction
        self.epsilon = 0.01  # precision required
        self.dt = 0.1  # size of step in motion prediction
        self.gain_weight = 10  # use weights for gain
        self.loss_weight = 20  # use weights for loss
        self.scene = Scene()  # scene of the robot

    def _speed_limit(self, speed):
        """
        Assert maximum velocity limit has not been exceeded
        """

        return np.abs(speed) <= self.max_speed

    def _predict_trajectory(self, src, left_speed, right_speed, theta, dt):
        """
        Predict trajectory of the robot, starting at src going at angle theta for dt time with velocities given
        """

        if np.abs(left_speed + right_speed) < self.epsilon / 10 ** 10:
            next_theta = theta + (right_speed - left_speed) * dt / (self.radius * 2)
            next_x = src[0]
            next_y = src[1]
        else:
            next_theta = theta
            next_x = src[0] + left_speed * dt * np.cos(theta)
            next_y = src[1] + right_speed * dt * np.sin(theta)

        return np.array([next_x, next_y]), next_theta  # predicted trajectory

    def _position_gain(self, new_pos):
        """
        Calculate gain defined as how much closer we got to the moving target
        We wish to maximize this gain as we pinpoint the target
        """

        curr_dist_to_goal = Util.dist(self.pos, self.target)
        new_dist_to_goal = Util.dist(new_pos, self.target)

        return curr_dist_to_goal - new_dist_to_goal

    def _position_loss(self, new_pos):
        """
        Calculate loss defined as how much closer we got to the obstacles
        We wish to minimize this loss in order to avoid crashing into obstacles
        """

        closest = min([Util.dist(new_pos, Obstacle.obs_to_tuple(x)) -
                       self.radius - x.radius for x in self.scene.obstacles])

        return max(self.clearance - closest, 0)

    def solution_found(self):
        """
        Check if moving target was caught with precision of epsilon
        """

        return Util.dist(self.pos, self.target) < self.epsilon

    def next_config(self):
        """
        Predict trajectory of the robot, starting at src going at angle theta for dt time with velocities given
        """

        delta = self.max_acc * self.dt  # denotes possible change in velocity
        right_vels = [self.right_vel + x * delta for x in [-1, 0, 1]]
        left_vels = [self.left_vel + x * delta for x in [-1, 0, 1]]
        best_possible = Util.minus_inf()
        best_l, best_r = 0.0, 0.0

        # attempt all velocities possible, check for the best one
        for new_r in right_vels:
            for new_l in left_vels:
                if not self._speed_limit(new_r) or not self._speed_limit(new_l):
                    break  # speed limit exceeded

                # steer the robot into a possible trajectory in the window to see if it is beneficial
                new_pos, theta = self._predict_trajectory(self.pos, new_l, new_r, self.theta, self.predict_time)
                gain = self.gain_weight * self._position_gain(new_pos)  # gain from steering times gain weight
                loss = self.loss_weight * self._position_loss(new_pos)  # loss from steering times loss weight
                total_cost = gain - loss  # total cost is taken to be gain - loss

                # candidate for best steering option, as it maximizes the total cost
                if total_cost > best_possible:
                    best_r, best_l, best_possible = new_r, new_l, total_cost

        # next config taken to be the argmax for total cost function
        self.right_vel, self.left_vel = best_r, best_l
        self.pos, self.theta = self._predict_trajectory(self.pos, self.right_vel, self.left_vel, self.theta, self.dt)

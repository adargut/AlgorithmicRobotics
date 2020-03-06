import math
import time
import cv2
import itertools
import numpy as np
import matplotlib.pylab as plt

N = -1
R = 0.01
GRID_SIZE = 20


class Utils:

    @staticmethod
    def dist(a, b):
        """
        Simple function to calculate distance
        """

        return np.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def generate_random_pos():
        """
        Scene parameters
        """

        return np.random.uniform(low=0.0, high=2.0), np.random.uniform(low=0.0, high=2.0)

    @staticmethod
    def generate_random_radius():
        """
        Sample pseudorandom radius for a circle
        """

        return np.random.uniform(low=0.02, high=0.03)

    @staticmethod
    def generate_image(zoom, h, w):
        """
        Generate a new underlying image for scene
        """

        return np.zeros([int(zoom * h), int(zoom * w), 3], dtype=np.uint8)


class Obstacle:
    """
    Obstacles are taken to be circles, defined by midpoint and radius
    """

    def __init__(self, r, p):
        self.r = r  # radius
        self.x = p[0]  # x coord
        self.y = p[1]  # y coord

    @staticmethod
    def obs_to_tuple(obs):
        """
        Convert an obstacle to tuple
        """

        return obs.x, obs.y

    @staticmethod
    def generate_random_obstacle():
        """
        Construct new random obstacle
        """

        return Obstacle(Utils.generate_random_radius(), Utils.generate_random_pos())


class Scene:

    def __init__(self, size, source, target, zoom, height, weight, epsilon):
        """
        Scene parameters
        """

        self.size = size
        self.obstacles = [None] * size
        self.img = None
        self.source = source
        self.target = target
        self.epsilon = epsilon
        self.height = height
        self.width = weight
        self.zoom = zoom

    @staticmethod
    def build_scene(size, source, target, zoom, height, weight, epsilon):
        """
        Construct a new scene
        """

        scene = Scene(size=size, source=source, target=target, zoom=zoom, height=height, weight=weight, epsilon=epsilon)
        scene._generate_random_scene()

        return scene

    def move_target(self):
        """
        Let target float above obstacles
        """

        global N

        if self.target[0] < 0.1 or self.target[0] > 1.98:
            N *= -1

        self.target[0] += N * 0.0005
        self.target[1] += N * 0.0005

    def _is_free_obstacle(self, obstacle):
        """
        Check if obstacle is free
        """

        obst_coords = Obstacle.obs_to_tuple(obstacle)
        delta = obstacle.r + 0.01

        for prev_obstacle in self.obstacles:
            if prev_obstacle:
                obs = Obstacle.obs_to_tuple(prev_obstacle)
                if Utils.dist(obs, obst_coords) < self.epsilon:
                    return False

        if Utils.dist(obst_coords, self.target) < delta or Utils.dist(obst_coords, self.source) < delta:
            return False

        return True

    def _generate_random_scene(self):
        """
        Fill scene with randomly generated obstacles
        """

        self.img = Utils.generate_image(self.zoom, self.height, self.width)

        for i in range(self.size):
            free_obstacle = False
            candidate_obstacle = None

            while not free_obstacle:
                candidate_obstacle = Obstacle.generate_random_obstacle()

                if self._is_free_obstacle(candidate_obstacle):
                    free_obstacle = True

            self.obstacles[i] = candidate_obstacle

    def play_animation(self, robot):
        """
        Play animation
        """

        assert isinstance(robot, Robot)  # input validity

        colors = [(100, 0, 0), (255, 15, 0), (0, 255, 0)]
        radius = int(R * self.zoom)

        cv2.rectangle(1, (1, 1),
                      (1, 1),
                      colors[0], 3)
        self.img[:] = (255, 255, 255)

        x, y = robot.pos
        c1 = (int(x * self.zoom), int((self.height - y) * self.zoom))
        c2 = (int(self.target[0] * self.zoom), int((self.height - self.target[1]) * self.zoom))
        centers = c1, c2
        i = 1

        for center in centers:
            cv2.circle(self.img, center, radius, colors[i], cv2.FILLED)
            i += 1

        for obstacle in self.obstacles:
            x, y = int(obstacle.x * self.zoom), int((self.height - obstacle.y) * self.zoom)
            radius = int(obstacle.r * self.zoom)
            cv2.circle(self.img, (x, y), radius, (0, 0, 0))

        cv2.imshow("Random Scene", self.img)
        cv2.waitKey(2)


class Robot:

    def __init__(self, src, dst, scene, max_speed=0.02, max_acc=0.02, r=0.01, clearance=0.3):
        """
        Parameters of the robot
        """

        self.pos = src  # current config
        self.util = Utils()  # utilities
        self.theta = 0.0  # orientation
        self.r = r  # radius of robot
        self.target = dst  # current target
        self.max_speed = max_speed  # max speed of robot
        self.max_acc = max_acc  # maximum acceleration of robot
        self.clearance = clearance  # clearance from obstacles
        self.left_vel = 0.0  # left velocity
        self.right_vel = 0.0  # right velocity
        self.predict_time = 1.0  # time step of trajectory prediction
        self.epsilon = 0.01  # precision required
        self.step_time = 0.2  # size of step in motion prediction
        self.gain_weight = 15.5  # use weights for gain
        self.loss_weight = 18.5  # use weights for loss
        self.scene = scene  # scene of the robot

    def _speed_limit(self, speed):
        """
        Assert maximum velocity limit has not been exceeded
        """

        return np.abs(speed) <= self.max_speed

    def _predict_trajectory(self, src, left_speed, right_speed, theta, dt):
        """
        Predict trajectory of the robot, starting at src going at angle theta for dt time with velocities given
        """

        delta1 = left_speed + right_speed
        delta2 = left_speed - right_speed

        if np.abs(delta1) < self.epsilon / 10000:
            next_theta = theta + (right_speed - left_speed) * dt / (self.r * 2)
            next_x = src[0]
            next_y = src[1]
        elif np.abs(delta2) < self.epsilon / 10000:
            next_theta = theta
            next_x = src[0] + left_speed * dt * np.cos(theta)
            next_y = src[1] + right_speed * dt * np.sin(theta)
        else:
            width = self.r * 2
            path = width * delta1 / (2 * delta2)
            next_theta = theta + delta2 * self.step_time / width
            next_x = src[0] + path * (np.sin(next_theta) - np.sin(theta))
            next_y = src[1] - path * (np.cos(next_theta) - np.cos(theta))

        trajectory = np.array([next_x, next_y]), next_theta
        return trajectory  # predicted trajectory

    def _position_gain(self, new_pos):
        """
        Calculate gain defined as how much closer we got to the moving target
        We wish to maximize this gain as we pinpoint the target
        """

        curr_dist_to_goal = Utils.dist(self.pos, self.scene.target)
        new_dist_to_goal = Utils.dist(new_pos, self.scene.target)

        return curr_dist_to_goal - new_dist_to_goal

    def _position_loss(self, new_pos):
        """
        Calculate loss defined as how much closer we got to the obstacles
        We wish to minimize this loss in order to avoid crashing into obstacles
        """

        closest = min([Utils.dist(new_pos, Obstacle.obs_to_tuple(x)) -
                       self.r - x.r for x in self.scene.obstacles])

        return max(self.clearance - closest, 0)

    def solution_found(self):
        """
        Check if moving target was caught with precision of epsilon
        """

        return Utils.dist(self.pos, self.target) < self.epsilon

    def next_config(self):
        """
        Find next config by maximizing total cost function
        The total cost is the gain from closing in on the target minus the loss from getting closer to the obstacles
        """

        delta = self.max_acc * self.step_time  # denotes possible change in velocity
        right_vels = [self.right_vel + x * delta for x in [-1, 0, 1]]
        left_vels = [self.left_vel + x * delta for x in [-1, 0, 1]]
        best_possible = -math.inf
        best_l, best_r = 0, 0

        # attempt all velocities possible, check for the best one
        for new_r, new_l in itertools.product(left_vels, right_vels):
            if not self._speed_limit(new_r) or not self._speed_limit(new_l):
                continue  # speed limit exceeded

            # steer the robot into a possible trajectory in the window to see if it is beneficial
            new_pos, theta = self._predict_trajectory(self.pos, new_l, new_r, self.theta, self.predict_time)
            gain = self.gain_weight * self._position_gain(new_pos)  # gain from steering times gain weight
            loss = self.loss_weight * self._position_loss(new_pos)  # loss from steering times loss weight
            total_cost = gain - loss

            # candidate for best steering option, as it maximizes the total cost
            if total_cost > best_possible:
                best_r, best_l, best_possible = new_r, new_l, total_cost

        # next config taken to be the argmax for total cost function
        self.right_vel, self.left_vel = best_r, best_l
        self.pos, self.theta = self._predict_trajectory(self.pos, self.left_vel, self.right_vel,
                                                        self.theta, self.step_time)


def algorithm_step(robot):
    """
    Run one iteration of the algorithm
    """

    global N

    robot.next_config()
    robot.scene.play_animation(robot)
    robot.scene.move_target()


def run_algorithm():
    """
    Run algorithm until convergence to goal
    """

    # parameters for program, hardcoded for now
    w, h, eps, size, zoom = 1.2, 1.2, 0.1, 200, 550
    target = np.array([1.0, 1.0])
    source = np.array([0.8, 0.2])

    scene = Scene.build_scene(size, source, target, zoom, w, h, eps)
    robot = Robot(source, target, scene)

    while not robot.solution_found():
        algorithm_step(robot)


def main():
    """
    Execute program
    """

    begin = time.time()
    run_algorithm()
    end = time.time()
    print("Target was caught in approximately", int(end - begin), "seconds")


if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-

"""Main module."""

import random
import numpy as np


class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


def twiddle(objFunction, args, init=0.5, tolerance=0.00001, domain=(float("-inf"), float("inf"))):
    """
      Optimize a single parameter given an objective function.

      This is a local hill-climbing algorithm. Here is a simple description of it:
      https://www.youtube.com/watch?v=2uQ2BSzDvXs

      @param args       (tuple)   Arguments necessary for the objective function.

      @param tolerance  (float)   Number used to determine when optimization has
                                  converged to a sufficiently good score.

      @param objFunction(function)Objective Function used to quantify how good a
                                  particular parameter choice is.

      @param init       (float)   Initial value of the parameter.

      @param domain     (tuple)   Domain of parameter values, as (min, max).

      @return (dict) Contains:
            "parameter" (float)   Threshold that returns the largest score from the
                                  Objective function.

            "score"     (float)   The score from the objective function given the
                                  threshold.
    """

    pastCalls = {}
    x = init
    delta = 0.1
    bestScore = objFunction(x, args)

    pastCalls[x] = bestScore

    while delta > tolerance:

        # Keep x within bounds
        if x + delta > domain[1]:
            delta = abs(domain[1] - x) / 2
        x += delta

        if x not in pastCalls:
            score = objFunction(x, args)
            pastCalls[x] = score

        score = pastCalls[x]

        if score > bestScore:
            bestScore = score
            delta *= 2

        else:
            # Keep x within bounds
            if x - delta < domain[0]:
                delta = abs(domain[0] - x) / 2
            x -= 2 * delta

            if x not in pastCalls:
                score = objFunction(x, args)
                pastCalls[x] = score

            score = pastCalls[x]

            if score > bestScore:
                bestScore = score
                delta *= 2
            else:
                x += delta
                delta *= 0.5

        print("Parameter:", x)
        print("Best score:", bestScore)
        print("Step size:", delta)
        print()

    return {"parameter": x,
            "score": bestScore}

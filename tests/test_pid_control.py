#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tests for `pid_control` package."""


import unittest
from click.testing import CliRunner

from pid_control.pid_control import Robot
import matplotlib.pyplot as plt
import numpy as np

from pid_control import cli


class TestPid_control(unittest.TestCase):
    """Tests for `pid_control` package."""

    def setUp(self):
        """Set up test fixtures, if any."""

    def tearDown(self):
        """Tear down test fixtures, if any."""

    def test_p_conroller(self):
        """Test implements a P controller by running 100 iterations of robot motion.
        The desired trajectory for the robot is the x-axis. The steering angle should be set
        by the parameter tau so that:

                        steering = -tau * crosstrack_error"""

        robot = Robot()
        robot.set(0, 1, 0)

        def run(robot, tau, n=100, speed=1.0):
            x_trajectory = []
            y_trajectory = []

            for i in range(n):
                cte = robot.y
                steer = -tau * cte
                robot.move(steer, speed)
                x_trajectory.append(robot.x)
                y_trajectory.append(robot.y)

            return x_trajectory, y_trajectory

        x_trajectory, y_trajectory = run(robot, 0.2)
        n = len(x_trajectory)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
        ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
        ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

    def test_pd_conroller(self):
        """
        Test implements a PD controller where the steering angle should be set by the parameter
        tau_p and tau_d so that:
            steering = -tau_p * CTE - tau_d * diff_CTE
        where differential crosstrack error (diff_CTE)
        is given by (CTE(t) - CTE(t-1))/deltat, deltat=1
        :return: x_trajectory, y_trajectory
        """

        def run(robot, tau_p, tau_d, n=100, speed=1.0):
            x_trajectory = []
            y_trajectory = []
            cte_prev = robot.y
            for i in range(n):
                cte = robot.y
                cte_diff = cte - cte_prev
                cte_prev = cte
                steer = -tau_p * cte - tau_d * cte_diff
                robot.move(steer, speed)
                x_trajectory.append(robot.x)
                y_trajectory.append(robot.y)
            return x_trajectory, y_trajectory

        robot = Robot()
        robot.set(0, 1, 0)
        robot.set_steering_drift(10.0 / 180.0 * np.pi)

        x_trajectory, y_trajectory = run(robot, 0.2, 0.0)
        n = len(x_trajectory)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
        ax1.plot(x_trajectory, y_trajectory, 'g', label='PD controller')
        ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

    def test_pid_conroller(self):
        """
        Test implements a PID controller where the steering angle should be set by the parameter
        tau_p, tau_d and tau_i so that:

            steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE

        where differential crosstrack error (diff_CTE) is given by (CTE(t) - CTE(t-1))/deltat, deltat=1
        and the integral cte is given by sum of cte over a period of time

        :return: x_trajectory, y_trajectory
        """

        robot = Robot()
        robot.set(0, 1, 0)

        def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
            x_trajectory = []
            y_trajectory = []
            cte_prev = robot.y
            cte_sum = 0.0
            for i in range(n):
                cte = robot.y
                cte_diff = cte - cte_prev
                cte_prev = cte
                cte_sum += cte
                steer = -tau_p * cte - tau_d * cte_diff - tau_i * cte_sum
                robot.move(steer, speed)
                x_trajectory.append(robot.x)
                y_trajectory.append(robot.y)

            return x_trajectory, y_trajectory

        x_trajectory, y_trajectory = run(robot, 0.2, 3.0, 0.004)
        n = len(x_trajectory)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
        ax1.plot(x_trajectory, y_trajectory, 'g', label='PID controller')
        ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

    def test_parameter_optrimization(self):
        """

        :return:
        """

        def make_robot():
            """
            Resets the robot back to the initial position and drift.
            You'll want to call this after you call `run`.
            """
            robot = Robot()
            robot.set(0, 1, 0)
            robot.set_steering_drift(10 / 180 * np.pi)
            return robot

        # NOTE: We use params instead of tau_p, tau_d, tau_i
        def run(robot, params, n=100, speed=1.0):
            x_trajectory = []
            y_trajectory = []
            err = 0
            prev_cte = robot.y
            int_cte = 0
            for i in range(2 * n):
                cte = robot.y
                diff_cte = cte - prev_cte
                int_cte += cte
                prev_cte = cte
                steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
                robot.move(steer, speed)
                x_trajectory.append(robot.x)
                y_trajectory.append(robot.y)
                if i >= n:
                    err += cte ** 2
            return x_trajectory, y_trajectory, err / n

        # Make this tolerance bigger if you are timing out!
        def twiddle(tol=0.2):
            # Don't forget to call `make_robot` before you call `run`!
            p = [0, 0, 0]
            dp = [1, 1, 1]
            robot = make_robot()
            x_trajectory, y_trajectory, best_err = run(robot, p)

            while sum(dp) > tol:

                for i in range(len(p)):
                    p[i] += dp[i]
                    robot = make_robot()
                    x_trajectory, y_trajectory, error = run(robot, p)
                    if error < best_err: # There was some improvement
                        best_err = error
                        dp[i] *= 1.1
                    else: # There was no improvement
                        p[i] -= 2.*p[i] # Go into the other direction
                        robot = make_robot()
                        x_trajectory, y_trajectory, error = run(robot, p)

                        if error < best_err:  # There was an improvement
                            best_err = error
                            dp[i] *= 1.1
                        else: # There was no improvement
                            p[i] += dp[i]
                            # As there was no improvement, the step size in either
                            # direction, the step size might simply be too big.
                            dp[i] *= 0.9

                return p, best_err

        params, err = twiddle()
        print("Final twiddle error = {}".format(err))
        robot = make_robot()
        x_trajectory, y_trajectory, err = run(robot, params)
        n = len(x_trajectory)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
        ax1.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')
        ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

    def test_command_line_interface(self):
        """Test the CLI."""
        runner = CliRunner()
        result = runner.invoke(cli.main)
        assert result.exit_code == 0
        assert 'pid_control.cli.main' in result.output
        help_result = runner.invoke(cli.main, ['--help'])
        assert help_result.exit_code == 0
        assert '--help  Show this message and exit.' in help_result.output

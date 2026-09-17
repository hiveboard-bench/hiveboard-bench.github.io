#!/usr/bin/env python3

import numpy as np

from cumotion_planner import CuMotionPlanner


def main():
    planner = CuMotionPlanner()

    q_start = np.array(
        [0.0, -0.0881, 0.0, -2.1491, 0.0, 2.0611, 0.79]
    )

    q_goal = np.array(
        [-0.64995754, -0.03649896, 0.64340085,
         -2.08020893, 0.02490260, 2.05131621, 0.77201368]
    )

    result = planner.plan_joint_trajectory(
        q_start=q_start,
        q_goal=q_goal,
        num_samples=100,
    )

    print("Status:", result["status"])
    print("Amostras:", len(result["times"]))
    print("Duração:", result["duration"])
    print("q_start:", result["qpos"][0])
    print("q_goal:", result["qpos"][-1])


if __name__ == "__main__":
    main()

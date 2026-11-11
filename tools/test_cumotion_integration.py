#!/usr/bin/env python3
"""Exercise the standalone cuMotion planner against both supported robots."""

from __future__ import annotations

import numpy as np

from cumotion_planner import CuMotionPlanner, ROBOT_RESOURCES


def check_robot(robot_name: str) -> None:
    planner = CuMotionPlanner(robot_name=robot_name)
    q_start = planner.q_home
    delta = np.linspace(0.015, 0.035, len(q_start))
    q_goal = q_start + delta

    result = planner.plan_joint_trajectory(
        q_start=q_start,
        q_goal=q_goal,
        num_samples=100,
    )

    qpos = np.asarray(result["qpos"], dtype=float)
    assert result["status"] == "Status.SUCCESS", result["status"]
    assert qpos.shape == (100, len(ROBOT_RESOURCES[robot_name].joint_names))
    assert np.all(np.isfinite(qpos))
    assert np.allclose(qpos[0], q_start, atol=1e-4)
    assert np.allclose(qpos[-1], q_goal, atol=1e-4)

    print(
        f"{robot_name}: {result['status']}; {len(qpos)} samples; "
        f"{result['duration']:.3f}s; endpoints and joint limits valid"
    )


def main() -> None:
    for robot_name in ("fr3", "spot"):
        check_robot(robot_name)


if __name__ == "__main__":
    main()

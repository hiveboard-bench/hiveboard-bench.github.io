#!/usr/bin/env python3
"""FR3 joint planning backend using cuMotion standalone."""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Sequence

import numpy as np
import mujoco
import cumotion


# ---------------------------------------------------------------------------
# PATHS
# ---------------------------------------------------------------------------

from dataclasses import dataclass
from pathlib import Path
import os


@dataclass(frozen=True)
class RobotResources:
    name: str
    mujoco_xml_pattern: str
    urdf: Path
    xrdf: Path
    joint_names: tuple[str, ...]
    home: tuple[float, ...]

    def mujoco_xml(self) -> Path:
        return REPO_ROOT / self.mujoco_xml_pattern.format()

REPO_ROOT = Path(__file__).resolve().parents[1]

CUMOTION_ROOT = Path(
    os.environ.get(
        "CUMOTION_HOME",
        str(
            Path.home()
            / "hiveboard-cumotion"
            / "cumotion"
            / "cumotion-1.1.0-cuda12.6-x86_64"
        ),
    )
)

CUMOTION_ROBOTS_ROOT = Path(
    os.environ.get(
        "CUMOTION_ROBOTS_HOME",
        str(Path.home() / "hiveboard-cumotion"),
    )
)


ROBOT_RESOURCES = {
    "fr3": RobotResources(
        name="fr3",
        mujoco_xml_pattern="public/sim/models/fr3.xml",
        urdf=CUMOTION_ROBOTS_ROOT
        / "fr3_cumotion"
        / "fr3.urdf",
        xrdf=CUMOTION_ROBOTS_ROOT
        / "fr3_cumotion"
        / "fr3.xrdf",
        joint_names=tuple(f"fr3_joint{i}" for i in range(1, 8)),
        home=(
            0.0,
            -0.0881,
            0.0,
            -2.1491,
            0.0,
            2.0611,
            0.79,
        ),
    ),
}

# ---------------------------------------------------------------------------
# Utils
# ---------------------------------------------------------------------------

def _as_qpos(values: Sequence[float], name: str) -> np.ndarray:
    """Converts a configuration into a 7-DOF FR3 joint vector."""
    q = np.asarray(values, dtype=np.float64)

    if q.shape != (7,):
        raise ValueError(
            f"{name} must contain exactly 7 values; "
            f"received shape={q.shape}"
        )

    if not np.all(np.isfinite(q)):
        raise ValueError(f"{name} contains non-finite values.")

    return q


def _domain_bounds(domain: Any) -> tuple[float, float]:
    """Gets the temporal bounds of Trajectory.Domain."""
    return float(domain.lower), float(domain.upper)


def _sample_trajectory(
    trajectory: Any,
    num_samples: int,
) -> dict[str, Any]:
    """Samples position, velocity, and acceleration of the trajectory."""
    if num_samples < 2:
        raise ValueError("num_samples must be >= 2.")

    lower, upper = _domain_bounds(trajectory.domain())
    times = np.linspace(lower, upper, num_samples)

    qpos = []
    qvel = []
    qacc = []
    qjerk = []

    for t in times:
        position, velocity, acceleration, jerk = trajectory.eval_all(
            float(t)
        )

        qpos.append(np.asarray(position, dtype=np.float64).tolist())
        qvel.append(np.asarray(velocity, dtype=np.float64).tolist())
        qacc.append(np.asarray(acceleration, dtype=np.float64).tolist())
        qjerk.append(np.asarray(jerk, dtype=np.float64).tolist())

    return {
        "duration": upper - lower,
        "times": times.tolist(),
        "qpos": qpos,
        "qvel": qvel,
        "qacc": qacc,
        "qjerk": qjerk,
    }


def _validate_limits(
    qpos: np.ndarray,
    model: mujoco.MjModel,
    joint_ids: Sequence[int],
) -> None:
    """Validate MuJoCo joint limits."""
    for i, joint_id in enumerate(joint_ids):
        if not model.jnt_limited[joint_id]:
            continue

        lower, upper = model.jnt_range[joint_id]

        if np.any(qpos[:, i] < lower - 1e-6):
            raise RuntimeError(
                f"Trajectory violates lower joint limit for joint {i}: "
                f"{lower}"
            )

        if np.any(qpos[:, i] > upper + 1e-6):
            raise RuntimeError(
                f"Trajectory violates upper joint limit for joint {i}: "
                f"{upper}"
            )


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------

class CuMotionPlanner:
    """Generic planner with cuMotion resources loaded once."""

    def __init__(
        self,
        robot_name: str = "fr3",
        board_index: int = 1,
        resources: RobotResources | None = None,
    ) -> None:
        if resources is None:
            try:
                resources = ROBOT_RESOURCES[robot_name]
            except KeyError as exc:
                raise ValueError(
                    f"Robot not configured: {robot_name}"
                ) from exc


        self.resources = resources
        self.robot_name = resources.name
        self.board_index = board_index

        self.urdf_path = Path(resources.urdf)
        self.xrdf_path = Path(resources.xrdf)
        self.mujoco_xml = resources.mujoco_xml()

        for path in (
            self.urdf_path,
            self.xrdf_path,
            self.mujoco_xml,
        ):
            if not path.exists():
                raise FileNotFoundError(
                    f"File not Found: {path}"
                )

        self.robot = cumotion.load_robot_from_file(
            self.xrdf_path,
            self.urdf_path,
        )

        self.tool_frame = self.robot.tool_frame_names()[0]

        self.world = cumotion.create_world()
        self.world_view = self.world.add_world_view()

        self.optimizer_config = (
            cumotion.create_default_trajectory_optimizer_config(
                self.robot,
                self.tool_frame,
                self.world_view,
            )
        )

        self.optimizer = cumotion.create_trajectory_optimizer(
            self.optimizer_config
        )

        self.mj_model = mujoco.MjModel.from_xml_path(
            str(self.mujoco_xml)
        )
        self.mj_data = mujoco.MjData(self.mj_model)

        self.joint_ids = [
            mujoco.mj_name2id(
                self.mj_model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )
            for joint_name in resources.joint_names
        ]

        missing = [
            joint_name
            for joint_name, joint_id in zip(
                resources.joint_names,
                self.joint_ids,
            )
            if joint_id < 0
        ]

        if missing:
            raise RuntimeError(
                "It was not possible to find the following joints in the MuJoCo model: "
                + ", ".join(missing)
            )

        self.q_home = _as_qpos(
            self.robot.default_cspace_configuration(),
            "q_home",
        )

    def plan_joint_trajectory(
        self,
        q_start: Sequence[float],
        q_goal: Sequence[float],
        num_samples: int = 100,
        validate: bool = True,
    ) -> dict[str, Any]:
        """
        Plan a joint trajectory.

        Returns a cuMotion-independent dictionary containing:
          duration, times, qpos, qvel, qacc, qjerk.
        """
        q_start = _as_qpos(q_start, "q_start")
        q_goal = _as_qpos(q_goal, "q_goal")

        target = cumotion.TrajectoryOptimizer.CSpaceTarget(q_goal)

        result = self.optimizer.plan_to_cspace_target(
            q_start,
            target,
        )

        status = result.status()

        if status != cumotion.TrajectoryOptimizer.Results.Status.SUCCESS:
            raise RuntimeError(
                f"cuMotion failed to plan. Status: {status}"
            )

        trajectory = result.trajectory()

        samples = _sample_trajectory(
            trajectory,
            num_samples,
        )

        if validate:
            qpos_array = np.asarray(samples["qpos"])

            _validate_limits(
                qpos_array,
                self.mj_model,
                self.joint_ids,
            )

            # Confirm that the trajectory endpoints match the expected values.
            start_error = np.linalg.norm(qpos_array[0] - q_start)
            end_error = np.linalg.norm(qpos_array[-1] - q_goal)

            if start_error > 1e-4:
                raise RuntimeError(
                    f"Trajectory initial error: {start_error}"
                )

            if end_error > 1e-4:
                raise RuntimeError(
                    f"Trajectory final error: {end_error}"
                )

        samples["status"] = str(status)
        samples["num_cspace_coords"] = trajectory.num_cspace_coords()

        return samples


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Joint planning with cuMotion."
    )

    parser.add_argument(
        "--goal",
        nargs=7,
        type=float,
        required=True,
        metavar="Q",
        help="Target joint configuration.",
    )

    parser.add_argument(
        "--start",
        nargs=7,
        type=float,
        default=None,
        metavar="Q",
        help="Initial joint configuration. Default: home.",
    )

    parser.add_argument(
        "--samples",
        type=int,
        default=100,
        help="Number of trajectory samples.",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output JSON file.",
    )

    args = parser.parse_args()

    planner = CuMotionPlanner()

    result = planner.plan_joint_trajectory(
        q_start=args.start,
        q_goal=args.goal,
        num_samples=args.samples,
    )

    output = json.dumps(result, indent=2)

    if args.output is not None:
        args.output.write_text(output + "\n", encoding="utf-8")
        print(f"Trajectory saved to: {args.output}")
    else:
        print(output)

    return 0


if __name__ == "__main__":
    sys.exit(main())

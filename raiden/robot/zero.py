"""Zero-torque mode: release the follower arms so they can be moved by hand.

Unlike `rd teleop --control handguide`, this records nothing and needs no input
device — it just drops the position gains so the only motor torque left is the
model-based gravity compensation term, holds that state until you ask for it
back, then restores position control.
"""

import os
import sys

import numpy as np

from raiden.robot.controller import RobotController


def _restore(robot_controller: RobotController, home: bool) -> None:
    """Re-engage position control on every initialized arm.

    With `home`, the arms interpolate from wherever they were left to the home
    pose.  Without it, each arm is re-stiffened at its *current* pose, so it
    holds where you put it instead of moving on its own.
    """
    robot_controller.disable_gravity_compensation()

    if home:
        robot_controller.move_to_home_positions(simultaneous=True)
        return

    print("  - Holding arms at their current poses...")
    for robot in (
        robot_controller.follower_r,
        robot_controller.follower_l,
    ):
        if robot is not None:
            robot.command_joint_pos(np.asarray(robot.get_joint_pos(), dtype=float))
    print("✓ Position control restored at current poses")


def run_zero_torque(
    arms: str = "bimanual",
    damping: float = 0.0,
    home_on_exit: bool = True,
) -> None:
    """Put the follower arms into gravity-compensation-only mode.

    Args:
        arms: "bimanual" for both followers, "single" for the left follower only.
        damping: Fraction of each arm's nominal kd retained as velocity damping.
            0.0 releases the arms completely; a small value (~0.2) resists slow
            drift from cable tension without resisting deliberate motion.
        home_on_exit: Move the arms back to their home poses when the session
            ends, instead of re-stiffening them where they were left.
    """
    use_right = arms == "bimanual"

    robot_controller = RobotController(
        use_right_leader=False,
        use_left_leader=False,
        use_right_follower=use_right,
        use_left_follower=True,
    )

    try:
        print("Checking CAN interfaces...")
        robot_controller.check_can_interfaces()
        robot_controller.initialize_robots(gravity_comp_mode=False)

        print("\n" + "!" * 60)
        print("  Support both follower arms by hand BEFORE continuing.")
        print("  Zeroing the position gains removes all position holding —")
        print("  an unsupported arm will drop under its own remaining weight.")
        print("!" * 60)
        input("\n  Press Enter to release the arms...")

        robot_controller.enable_gravity_compensation(damping_scale=damping)

        print("\n" + "=" * 60)
        print("  ZERO-TORQUE MODE ACTIVE (gravity compensation only)")
        print("=" * 60)
        print("\n  The arms are free to move by hand.")
        if home_on_exit:
            print("  On exit the arms return to their home poses.")
        else:
            print("  On exit the arms stiffen up where you left them.")
        print("\n  Press Enter (or Ctrl+C) to re-engage position control.")
        print("=" * 60 + "\n")

        try:
            input()
        except KeyboardInterrupt:
            print()

        print("\n  !! Support the arms again before they re-engage !!")
        input("  Press Enter to restore position control...")
        _restore(robot_controller, home=home_on_exit)

        robot_controller.close()
        print("\nZero-torque session ended.")

    except KeyboardInterrupt:
        # Ctrl+C outside the release loop: the arms may still be limp, so bring
        # them back under control rather than dropping the CAN connections.
        print("\nInterrupted — restoring position control.")
        if robot_controller.has_robots():
            _restore(robot_controller, home=home_on_exit)
            robot_controller.close()
    except Exception as e:
        print(f"\nError: {e}")
        if robot_controller.has_robots():
            robot_controller.emergency_stop()
        sys.exit(1)
    finally:
        os._exit(0)

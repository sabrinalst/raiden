"""Direct hand-guiding of the follower arms via gravity compensation.

Intended for setups without leader arms: the followers are put into zero-stiffness
gravity-compensation mode so they can be posed by hand, and poses are recorded
from the keyboard (or the footpedal, if present).
"""

import threading

from raiden.control.base import TeleopInterface
from raiden.robot.footpedal import PEDAL_LEFT, PEDAL_MIDDLE, PEDAL_RIGHT, try_open_footpedal


class HandGuideInterface(TeleopInterface):
    """Followers are moved directly by hand under gravity compensation."""

    def __init__(self, damping_scale: float = 0.2):
        """
        Args:
            damping_scale: Fraction of the arm's nominal kd retained as velocity
                damping while hand-guiding.  Resists slow drift from cable
                tension; raise it if the arms still creep, lower it toward 0 if
                they feel sluggish to move.
        """
        self._damping_scale = damping_scale

    @property
    def name(self) -> str:
        return "handguide"

    @property
    def uses_leaders(self) -> bool:
        return False

    # ------------------------------------------------------------------
    # Session-level lifecycle (footpedal)
    # ------------------------------------------------------------------

    def open(self) -> None:
        self._pedal_trigger = threading.Event()
        self._pedal_success = threading.Event()
        self._pedal_failure = threading.Event()
        self._footpedal = try_open_footpedal()
        if self._footpedal is not None:

            def _cb(code: int) -> None:
                if code == PEDAL_LEFT:
                    rc = getattr(self, "_recording_controller", None)
                    if rc is not None:
                        rc.soft_pause()
                    else:
                        self._pedal_trigger.set()
                elif code == PEDAL_MIDDLE:
                    self._pedal_success.set()
                elif code == PEDAL_RIGHT:
                    self._pedal_failure.set()

            self._footpedal.on_press(_cb)
            self._footpedal.start()
            print(
                "  ✓ FootPedal ready: left=trigger/pause  middle=success  right=failure"
            )

    def close(self) -> None:
        if getattr(self, "_footpedal", None) is not None:
            self._footpedal.close()
            self._footpedal = None

    # ------------------------------------------------------------------
    # Episode-level lifecycle
    # ------------------------------------------------------------------

    def setup(self, robot_controller) -> None:
        pass  # nothing to attach — the arms themselves are the input device

    def start(self, robot_controller) -> None:
        """Release the followers so they can be moved by hand.

        Support the arms before this returns: zeroing the PD gains removes all
        position holding, leaving only the model-based gravity term.
        """
        print("\n  !! Support both follower arms by hand before continuing !!")
        robot_controller.enable_gravity_compensation(
            damping_scale=self._damping_scale
        )

    def stop(self, robot_controller) -> None:
        robot_controller.disable_gravity_compensation()

    # ------------------------------------------------------------------
    # Polling
    # ------------------------------------------------------------------

    def poll(self, robot_controller) -> bool:
        trigger = getattr(self, "_pedal_trigger", None)
        if trigger is not None and trigger.is_set():
            trigger.clear()
            return True
        return False

    def poll_success(self, robot_controller) -> bool:
        ev = getattr(self, "_pedal_success", None)
        if ev is not None and ev.is_set():
            ev.clear()
            return True
        return False

    def poll_failure(self, robot_controller) -> bool:
        ev = getattr(self, "_pedal_failure", None)
        if ev is not None and ev.is_set():
            ev.clear()
            return True
        return False

    @property
    def banner(self) -> str:
        return (
            "\n" + "=" * 60 + "\n"
            "  HAND-GUIDE MODE ACTIVE (gravity compensation)\n" + "=" * 60 + "\n\n"
            "  The follower arms are limp apart from gravity compensation.\n"
            "  Move them by hand, then type 'r' to record the pose.\n"
            "  Press Ctrl+C for EMERGENCY STOP (stops all motion immediately)\n\n"
            + "=" * 60
            + "\n"
        )

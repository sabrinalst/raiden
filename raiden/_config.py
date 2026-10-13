"""Default paths for user configuration files."""

import json
from pathlib import Path
from typing import Dict

CONFIG_DIR = Path.home() / ".config" / "raiden"
DB_DIR = CONFIG_DIR / "db"
WEIGHTS_DIR = CONFIG_DIR / "weights"

CAMERA_CONFIG = str(CONFIG_DIR / "camera.json")
CALIBRATION_FILE = str(CONFIG_DIR / "calibration_results.json")
CALIBRATION_POSES_FILE = str(CONFIG_DIR / "calibration_poses.json")
SPACEMOUSE_CONFIG = str(CONFIG_DIR / "spacemouse.json")
CAN_CONFIG = str(CONFIG_DIR / "can.json")

# Logical arm name → SocketCAN interface name.  Overridable per-workstation via
# can.json so that setups whose udev rules assign different names (or that have
# fewer than four adapters) do not need to rewrite their udev rules.
DEFAULT_CAN_INTERFACES: Dict[str, str] = {
    "follower_r": "can_follower_r",
    "follower_l": "can_follower_l",
    "leader_r": "can_leader_r",
    "leader_l": "can_leader_l",
}


def load_can_interfaces(config_file: str = CAN_CONFIG) -> Dict[str, str]:
    """Return the arm → CAN interface mapping, applying can.json overrides.

    Unknown keys are ignored so that a typo degrades to the default name (and
    then to a clear "missing CAN interface" error) rather than silently
    initialising the wrong arm.
    """
    interfaces = dict(DEFAULT_CAN_INTERFACES)
    path = Path(config_file)
    if not path.exists():
        return interfaces

    try:
        with open(path) as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError) as e:
        print(f"Warning: could not read {path} ({e}); using default CAN names")
        return interfaces

    for key, value in data.items():
        if key in interfaces:
            interfaces[key] = value
        else:
            print(f"Warning: unknown arm '{key}' in {path}; ignoring")
    return interfaces

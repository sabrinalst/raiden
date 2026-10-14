"""Live check of the camera calibration on the bimanual YAM rig.

Streams every RealSense camera as a coloured point cloud, placed using
calibration_results.json, together with both arms' meshes posed from their
measured joint positions — all in the left-arm base frame, the same frame and
the same transform chain that `rd convert` writes into the dataset:

    scene camera:  T_left_base_from_cam                    (static, from calibration)
    left wrist:    FK_left(q)  @ T_ee_from_cam             (hand-eye)
    right wrist:   T_left_base_from_right_base @ FK_right(q) @ T_ee_from_cam

If the calibration is right, the wrist cameras' clouds stay locked onto the
scene camera's cloud as the arms move, and the scene cloud of each gripper sits
on its mesh.
"""

import json
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import mujoco
import numpy as np
import rerun as rr

from raiden._xml_paths import get_yam_4310_linear_xml_path
from raiden.camera_config import CameraConfig

# A stalled RealSense sometimes needs its pipeline restarted to recover.
_RESTART_AFTER_FAILED_GRABS = 6

# Downscale applied to the images shown on the camera frustums (points are
# coloured from the full-resolution image).
_IMAGE_SCALE = 0.5

_ARM_COLORS = {"left": [60, 120, 220], "right": [220, 80, 60]}


# ---------------------------------------------------------------------------
# Robot model
# ---------------------------------------------------------------------------


class _ArmModel:
    """MuJoCo model of one YAM arm + linear-4310 gripper, posed from measured joints."""

    def __init__(self, xml_path: str):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self._site_id = self.model.site("grasp_site").id
        self.mesh_geoms = [
            g
            for g in range(self.model.ngeom)
            if self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
        ]

    def set_joints(self, q: np.ndarray) -> None:
        """Pose the model from 6 arm joints, optionally followed by the gripper in [0, 1]."""
        m, d = self.model, self.data
        d.qpos[:] = 0.0
        n = min(len(q), m.nq)
        d.qpos[:n] = q[:n]
        # The gripper is reported normalised to [0, 1]; scale slide joints to metres
        for j in range(m.njnt):
            adr = m.jnt_qposadr[j]
            if adr < n and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE:
                lo, hi = m.jnt_range[j]
                d.qpos[adr] = lo + d.qpos[adr] * (hi - lo)
        # Coupled fingers are joint-equality constraints
        for i in range(m.neq):
            if m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT:
                a1 = m.jnt_qposadr[m.eq_obj1id[i]]
                a2 = m.jnt_qposadr[m.eq_obj2id[i]]
                d.qpos[a2] = np.polyval(m.eq_data[i, :5][::-1], d.qpos[a1])
        mujoco.mj_kinematics(m, d)

    def ee_pose(self) -> np.ndarray:
        """grasp_site in the arm base frame — the frame the hand-eye calibration uses."""
        T = np.eye(4)
        T[:3, :3] = self.data.site_xmat[self._site_id].reshape(3, 3)
        T[:3, 3] = self.data.site_xpos[self._site_id]
        return T

    def geom_pose(self, g: int) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.data.geom_xmat[g].reshape(3, 3)
        T[:3, 3] = self.data.geom_xpos[g]
        return T

    def geom_mesh(self, g: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Vertices (in the geom frame), triangle indices and RGBA of a mesh geom."""
        m = self.model
        mesh = m.geom_dataid[g]
        v0, nv = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
        f0, nf = m.mesh_faceadr[mesh], m.mesh_facenum[mesh]
        return m.mesh_vert[v0 : v0 + nv], m.mesh_face[f0 : f0 + nf], m.geom_rgba[g]


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------


def _rt_to_T(entry: Dict) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.array(entry["rotation_matrix"], dtype=np.float64)
    T[:3, 3] = np.array(entry["translation_vector"], dtype=np.float64).flatten()
    return T


def _load_calibration(path: str) -> Dict:
    """Return {"static": {cam: T}, "hand_eye": {cam: T}, "right_to_left": T | None}."""
    with open(path) as f:
        calib = json.load(f)

    static, hand_eye = {}, {}
    for name, cam in calib.get("cameras", {}).items():
        he = cam.get("hand_eye_calibration")
        ext = cam.get("extrinsics")
        if he and he.get("success"):
            hand_eye[name] = _rt_to_T(he)
        elif ext and ext.get("success"):
            static[name] = _rt_to_T(ext)

    # "right_base_to_left_base" maps left_arm_base → right_arm_base (despite the
    # name), as in the converter and server. Invert to bring right_arm_base
    # points into left_arm_base.
    right_to_left = None
    bt = calib.get("bimanual_transform", {}).get("right_base_to_left_base")
    if bt is not None:
        right_to_left = np.linalg.inv(np.array(bt, dtype=np.float64))

    print(f"Calibration: {path} ({calib.get('timestamp', 'no timestamp')})")
    return {"static": static, "hand_eye": hand_eye, "right_to_left": right_to_left}


# ---------------------------------------------------------------------------
# Cameras
# ---------------------------------------------------------------------------


class _LiveCamera:
    """A RealSense camera streaming colour + depth aligned to colour."""

    def __init__(self, name: str, role: Optional[str], camera, fps: int):
        self.name = name
        self.role = role
        self.camera = camera
        self.fps = fps
        self._failed = 0
        self.K: Optional[np.ndarray] = None

    def start(self) -> None:
        import pyrealsense2 as rs

        self.camera.open()
        # Live pipelines don't align depth to colour by default; the point
        # cloud needs both on the colour camera's pixel grid and intrinsics.
        self.camera._align = rs.align(rs.stream.color)
        K, _, _ = self.camera.get_intrinsics()
        self.K = np.asarray(K, dtype=np.float64)

    def stop(self) -> None:
        try:
            self.camera.close()
        except RuntimeError:
            pass  # pipeline never started

    def read(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Latest (BGR colour, depth in metres), or None if no frame arrived."""
        if self.camera.grab():
            self._failed = 0
            frame = self.camera.get_frame()
            return frame.color.copy(), frame.depth.copy()

        self._failed += 1
        if self._failed >= _RESTART_AFTER_FAILED_GRABS:
            print(f"  {self.name}: no frames, restarting the camera")
            self._failed = 0
            self.stop()
            try:
                self.start()
            except RuntimeError as e:
                print(f"  {self.name}: restart failed: {e}")
        return None


def _open_cameras(camera_config_file: str, fps: int) -> List[_LiveCamera]:
    from raiden.cameras.realsense import RealSenseCamera

    cfg = CameraConfig(camera_config_file)
    cams = []
    for name in cfg.list_camera_names():
        if cfg.get_camera_type(name) != "realsense":
            print(f"  Skipping {name}: only RealSense cameras are supported")
            continue
        entry = cfg.cameras[name]

        def _size(key: str):
            v = entry.get(key)
            return (int(v[0]), int(v[1])) if v is not None else None

        camera = RealSenseCamera(
            name,
            str(cfg.get_serial_by_name(name)),
            fps=fps,
            color_size=_size("color_size"),
            depth_size=_size("depth_size"),
        )
        live = _LiveCamera(name, cfg.get_role(name), camera, fps)
        try:
            live.start()
        except RuntimeError as e:
            print(f"  ✗ {name}: could not start ({e}); skipping it")
            live.stop()
            continue
        cams.append(live)
    return cams


def _unproject(
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    K: np.ndarray,
    T_world_cam: np.ndarray,
    stride: int,
    max_depth: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Every stride-th pixel with valid depth, as world points and RGB colours."""
    H, W = depth_m.shape
    vv, uu = np.mgrid[0:H:stride, 0:W:stride]
    d = depth_m[vv, uu]
    valid = (d > 0.05) & (d < max_depth)
    u, v, d = uu[valid], vv[valid], d[valid]
    pts_cam = np.stack(
        [(u - K[0, 2]) * d / K[0, 0], (v - K[1, 2]) * d / K[1, 1], d], axis=1
    )
    pts = pts_cam @ T_world_cam[:3, :3].T + T_world_cam[:3, 3]
    rgb = color_bgr[v, u][:, ::-1]
    return pts.astype(np.float32), rgb


# ---------------------------------------------------------------------------
# Rerun logging
# ---------------------------------------------------------------------------


def _log_static_scene(arm_model: Optional[_ArmModel], sides: List[str]) -> None:
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/left_base", rr.TransformAxes3D(0.1), static=True)
    if arm_model is None:
        return
    for side in sides:
        for g in arm_model.mesh_geoms:
            verts, faces, rgba = arm_model.geom_mesh(g)
            rr.log(
                f"world/arms/{side}/geom_{g}",
                rr.Mesh3D(
                    vertex_positions=verts,
                    triangle_indices=faces,
                    albedo_factor=(rgba * 255).astype(np.uint8),
                ),
                static=True,
            )


def _log_transform(entity: str, T: np.ndarray) -> None:
    rr.log(entity, rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]))


def _log_camera(
    cam: _LiveCamera,
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    T_world_cam: np.ndarray,
    stride: int,
    max_depth: float,
) -> None:
    import cv2

    entity = f"world/cameras/{cam.name}"
    _log_transform(entity, T_world_cam)

    H, W = color_bgr.shape[:2]
    small = cv2.resize(color_bgr, (int(W * _IMAGE_SCALE), int(H * _IMAGE_SCALE)))
    K_small = cam.K.copy()
    K_small[:2] *= _IMAGE_SCALE
    rr.log(
        entity,
        rr.Pinhole(
            image_from_camera=K_small,
            resolution=[small.shape[1], small.shape[0]],
            image_plane_distance=0.1,
        ),
    )
    rr.log(entity, rr.Image(small[:, :, ::-1]))

    pts, rgb = _unproject(color_bgr, depth_m, cam.K, T_world_cam, stride, max_depth)
    rr.log(f"world/points/{cam.name}", rr.Points3D(pts, colors=rgb, radii=0.002))


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def _wrist_side(cam: _LiveCamera) -> Optional[str]:
    if cam.role == "left_wrist":
        return "left"
    if cam.role == "right_wrist":
        return "right"
    return None


def _visualize(
    cams: List[_LiveCamera],
    calib: Dict,
    robots: Dict[str, object],
    arm_model: Optional[_ArmModel],
    rate: float,
    stride: int,
    max_depth: float,
) -> None:
    """Log until Ctrl+C."""
    # Arm base frames in the left-arm base frame
    base_in_left = {"left": np.eye(4)}
    if calib["right_to_left"] is not None:
        base_in_left["right"] = calib["right_to_left"]
    elif "right" in robots:
        print("  ⚠ No bimanual_transform in calibration: the right arm can't be placed")

    period = 1.0 / rate
    tick = 0
    while True:
        t0 = time.monotonic()
        rr.set_time("tick", sequence=tick)

        frames = {cam.name: cam.read() for cam in cams}
        # Read the joints right after the frames so both describe the same moment
        ee_in_left = {}
        for side, robot in robots.items():
            if side not in base_in_left:
                continue
            arm_model.set_joints(np.asarray(robot.get_joint_pos(), dtype=np.float64))
            T_base = base_in_left[side]
            ee_in_left[side] = T_base @ arm_model.ee_pose()
            for g in arm_model.mesh_geoms:
                _log_transform(
                    f"world/arms/{side}/geom_{g}", T_base @ arm_model.geom_pose(g)
                )
            _log_transform(f"world/arms/{side}/ee", ee_in_left[side])
            rr.log(f"world/arms/{side}/ee", rr.TransformAxes3D(0.05))

        for cam in cams:
            frame = frames[cam.name]
            if frame is None:
                continue
            side = _wrist_side(cam)
            if side is None:
                T_world_cam = calib["static"].get(cam.name)
            elif side in ee_in_left and cam.name in calib["hand_eye"]:
                T_world_cam = ee_in_left[side] @ calib["hand_eye"][cam.name]
            else:
                T_world_cam = None
            if T_world_cam is not None:
                _log_camera(cam, *frame, T_world_cam, stride, max_depth)

        tick += 1
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


def run_viz_calibration(
    calibration_file: str,
    camera_config_file: str,
    arms: bool = True,
    release: bool = True,
    damping: float = 0.2,
    home_on_exit: bool = True,
    fps: int = 15,
    rate: float = 5.0,
    stride: int = 4,
    max_depth: float = 1.5,
    web: bool = False,
    web_port: int = 9090,
) -> None:
    """Show live point clouds from every camera and both arms, placed by the calibration.

    Args:
        arms: Connect to the follower arms. Needed to place the wrist cameras
            and draw the arms; without it only the scene camera is shown and
            the arms are not touched.
        release: Put the arms in gravity compensation so they can be moved by
            hand. Without it they hold the pose they were in.
        damping: Fraction of nominal kd kept while released (resists drift).
        home_on_exit: Move the arms home on exit instead of stiffening in place.
        fps: Camera frame rate for this session (does not change camera.json).
        rate: Viewer updates per second.
        stride: Use every stride-th pixel for the point clouds.
        max_depth: Drop points further than this from the camera, in metres.
        web: Serve the viewer over HTTP (for SSH) instead of spawning it.
    """
    from raiden.robot.zero import _restore

    calib = _load_calibration(calibration_file)
    for name, T in calib["static"].items():
        print(f"  {name}: static, camera at {np.round(T[:3, 3], 3)} m")
    for name in calib["hand_eye"]:
        print(f"  {name}: hand-eye")

    robot_controller = None
    released = False
    cams: List[_LiveCamera] = []
    try:
        print("\nStarting cameras...")
        cams = _open_cameras(camera_config_file, fps)
        if not cams:
            raise RuntimeError("No cameras could be started")

        robots: Dict[str, object] = {}
        arm_model = None
        if arms:
            from raiden.robot.controller import RobotController

            robot_controller = RobotController(
                use_right_leader=False, use_left_leader=False
            )
            print("\nChecking CAN interfaces...")
            robot_controller.check_can_interfaces()
            # Connects holding the current pose — the arms don't move
            robot_controller.initialize_robots(gravity_comp_mode=False)
            robots = {
                "left": robot_controller.follower_l,
                "right": robot_controller.follower_r,
            }
            arm_model = _ArmModel(get_yam_4310_linear_xml_path())

            if release:
                print("\n" + "!" * 60)
                print("  Support both follower arms by hand BEFORE continuing.")
                print("  Releasing removes position holding: an unsupported arm")
                print("  can sag under its own remaining weight.")
                print("!" * 60)
                input("\n  Press Enter to release the arms...")
                robot_controller.enable_gravity_compensation(damping_scale=damping)
                released = True
                print("\n  Arms released: move them by hand to check the calibration.")
            else:
                print("\n  Arms are holding their current pose (--no-release).")
        else:
            print("\n  --no-arms: only the scene camera is shown; arms untouched.")

        if web:
            from urllib.parse import quote

            rr.init("raiden_viz_calibration")
            grpc_port = web_port + 1
            server_uri = rr.serve_grpc(grpc_port=grpc_port)
            rr.serve_web_viewer(web_port=web_port, open_browser=False)
            print(
                f"\nOpen in browser: http://localhost:{web_port}?url={quote(server_uri, safe='')}"
            )
            print(
                f"SSH tunnel:      ssh -L {web_port}:localhost:{web_port} "
                f"-L {grpc_port}:localhost:{grpc_port} <host>"
            )
        else:
            rr.init("raiden_viz_calibration", spawn=True)

        # The right arm can only be placed through the bimanual transform
        sides = [
            s for s in robots if s == "left" or calib["right_to_left"] is not None
        ]
        _log_static_scene(arm_model, sides)
        print("\nStreaming. What to look for:")
        print("  - wrist clouds stay locked onto the scene cloud as the arms move")
        print("  - the scene camera's view of each gripper sits on its mesh")
        print("  Hold an arm still for a moment before judging — the wrist cloud")
        print("  lags slightly while it moves.")
        print("\n  Press Ctrl+C to stop.\n")

        try:
            _visualize(cams, calib, robots, arm_model, rate, stride, max_depth)
        except KeyboardInterrupt:
            print()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"\nError: {e}")
        if robot_controller is not None and robot_controller.has_robots():
            robot_controller.emergency_stop()
            robot_controller.close()
            robot_controller = None
    finally:
        for cam in cams:
            cam.stop()
        if robot_controller is not None and robot_controller.has_robots():
            if released:
                print("\n  !! Support the arms again before they re-engage !!")
                try:
                    input("  Press Enter to restore position control...")
                except KeyboardInterrupt:
                    print()
            _restore(robot_controller, home=home_on_exit)
            robot_controller.close()
        sys.stdout.flush()
        # i2rt's control threads can keep the interpreter alive
        os._exit(0)

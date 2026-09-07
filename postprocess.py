import argparse
import time
from pathlib import Path

import gtsam
#import matplotlib
#import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pymap3d as pm
import os
from gtsam.symbol_shorthand import P
#from matplotlib.gridspec import GridSpec
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp
from mcap.exceptions import EndOfFile
from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory
from plyfile import PlyData, PlyElement
from PIL import Image
from datetime import datetime


# ---------------- Parameters ----------------
BASEPATH = Path(__file__).resolve().parent / "data"

INITIAL_YAW_DEG = 60.0

STAT_TH = 1.0                       # 1: Fix, 2: Float
ARM = np.array([0.0, 0.09, 0.18])   # lever arm (GNSS antenna - IMU), m
SIGMA_GNSS = 0.2                    # GNSS position std (m)
SIGMA_POS = 0.01                    # LiDAR odometry position std (m)
SIGMA_ATT = np.deg2rad(0.01)        # LiDAR odometry attitude std (rad)
RELATIVE_DIST_TH = 30.0             # distance threshold for loop constraints (m)
RELATIVE_TIME_TH = 5.0              # time threshold for loop constraints (s)

ORGLLH = (35.89120474, 139.57004460, 47.6648)  # base station LLH

LIDAR_HZ = 10.0                     # used for the loop-closure time gap
LEAP_SEC = 18                       # GPST - UTC seconds

# 3D pointcloud construction time window (GPST, naive)
TS_GPST = pd.Timestamp("2026-01-29 04:45:23")
TE_GPST = pd.Timestamp("2026-01-29 04:45:51")
LEAP_SEC = 18  # GPST - UTC seconds

RNG_MIN = 1.0          # short-range rejection threshold (m)
RNG_MAX = 100.0        # long-range rejection threshold (m)
INTENSITY_TH = 1       # low-intensity rejection threshold
SKIP = 1               # take every SKIP-th lidar message

LIDAR_TOPIC = "/lidar/points"

# PointCloud2 PointField layout for this bag (point_step = 23 B).
# Packed (no alignment padding); intensity is uint8, ring is uint16.
POINT_DTYPE = np.dtype(
    {
        "names":   ["x",   "y",   "z",   "intensity", "ring",  "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "u1",         "<u2",   "<f8"],
        "offsets": [0,     4,     8,     12,           13,      15],
        "itemsize": 23,
    }
)

# IR application parameters

# Camera intrinsic values, standard opencv matrix form.
K = np.array([
    [774.98646364, 0.0, 309.00747709],
    [0.0, 770.55037689, 261.70299926],
    [0.0, 0.0, 1.0]
], dtype=np.float64)
# camera_q = qx qy qz qw rotation from lidar frame
q_camera = np.array([0.5, 0, 0, 0.866])
ROT_CAM = Rotation.from_quat(q_camera)


# ---------------- Time helpers ----------------

def _gpst_to_unix(ts: pd.Timestamp) -> float:
    """GPST-naive Timestamp -> UTC UNIX seconds."""
    return ts.value / 1e9 - LEAP_SEC


# ---------------- Custom GTSAM factor (GPSFactorArm equivalent) ----------------

def _skew(v):
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ]
    )


def make_gps_factor_arm(key, measurement, arm, noise):
    """Equivalent of gtsam.GPSFactorArm (not exposed in the PyPI gtsam wheel).

    error(pose) = pose.t + pose.R @ arm - measurement
    Jacobian w.r.t. Pose3 tangent [omega; v]:
      d_err/d_omega = -R @ skew(arm)
      d_err/d_v     =  R
    """
    measurement = np.asarray(measurement, dtype=float)
    arm = np.asarray(arm, dtype=float)
    skew_arm = _skew(arm)

    def error_func(this, values, jacobians):
        pose = values.atPose3(this.keys()[0])
        R = pose.rotation().matrix()
        t = pose.translation()
        err = t + R @ arm - measurement
        if jacobians is not None:
            H = np.empty((3, 6))
            H[:, :3] = -R @ skew_arm
            H[:, 3:] = R
            jacobians[0] = H
        return err

    return gtsam.CustomFactor(noise, [key], error_func)


# ---------------- Input ----------------

def read_glim_trajectory(path):
    """Read traj_lidar.txt -> (t_gpst_unix, pos, rot).

    Columns of the file: unix_utc, x, y, z, qx, qy, qz, qw.
    The returned time is `UTC + LEAP_SEC` so that it shares a numeric scale with
    GPST-as-seconds-since-epoch derived from the GNSS file.
    """
    lio = np.loadtxt(path)
    return (
        lio[:, 0] + LEAP_SEC,
        lio[:, 1:4],
        Rotation.from_quat(lio[:, [4, 5, 6, 7]]),
    )


def read_gnss_solution(path, orgllh):
    """Read RTKLib .pos -> (t_gpst_unix, enu, stat).

    The .pos timestamps are GPST already; parsing the naive datetime and treating
    the result as seconds-since-epoch keeps the same numeric scale as the GLIM
    file once the +LEAP_SEC offset is applied there.
    """
    gps = pd.read_csv(
        path, sep=r"\s+", skiprows=26, header=None, engine="python"
    )
    dt = pd.to_datetime(
        gps[0].astype(str) + " " + gps[1].astype(str),
        format="%Y/%m/%d %H:%M:%S.%f",
    )
    # pandas 3.x defaults to datetime64[us]; force ns for an unambiguous conversion.
    t = dt.astype("datetime64[ns]").astype("int64").to_numpy() / 1e9
    lla = gps.iloc[:, 2:5].to_numpy(dtype=float)
    e, n, u = pm.geodetic2enu(lla[:, 0], lla[:, 1], lla[:, 2], *orgllh)
    return t, np.column_stack([e, n, u]), gps.iloc[:, 5].to_numpy(dtype=float)

def read_pointcloud(mcap_path: Path, skip: int = 1) -> np.ndarray:
    """Stream PointCloud2 messages on LIDAR_TOPIC into one structured ndarray.

    Uses NonSeekingReader + log_time_order=False to bypass the file summary,
    which is malformed in this particular bag. The trailing record may also be
    truncated (EndOfFile); stop gracefully when that happens.
    """
    chunks = []
    n_msgs = 0
    with open(mcap_path, "rb") as f:
        reader = NonSeekingReader(f, decoder_factories=[DecoderFactory()])
        it = reader.iter_decoded_messages(
            topics=[LIDAR_TOPIC], log_time_order=False
        )
        try:
            for idx, (_s, _c, _r, msg) in enumerate(it):
                if idx % skip != 0:
                    continue
                buf = msg.data if isinstance(
                    msg.data, (bytes, bytearray, memoryview)
                ) else bytes(msg.data)
                chunks.append(
                    np.frombuffer(
                        buf, dtype=POINT_DTYPE, count=msg.width * msg.height
                    )
                )
                n_msgs += 1
        except EndOfFile:
            pass
    print(f"  decoded {n_msgs} messages")
    return np.concatenate(chunks)


# ---------------- Trajectory Processing ----------------

def interpolate_gnss(tgps, enugps, stat_gps, tlio):
    """Linearly resample GNSS ENU position and quality flag at LiDAR timestamps."""
    enu = interp1d(
        tgps, enugps, axis=0, kind="linear",
        bounds_error=False, fill_value=np.nan,
    )(tlio)
    stat = interp1d(
        tgps, stat_gps, kind="linear",
        bounds_error=False, fill_value=np.nan,
    )(tlio)
    return enu, stat


def apply_initial_yaw(pos, rot, yaw_deg, origin):
    """Rotate by `yaw_deg` around Z and translate so it starts at `origin`.

    Equivalent to MATLAB:
        q = quaternion([0 0 yaw_deg], "eulerd", "XYZ", "frame");
        tform = rigidtform3d(q.rotmat("point"), origin);
        pos_out = transformPointsForward(tform, pos);
        rot_out = q * rot;
    """
    q = Rotation.from_euler("z", yaw_deg, degrees=True)
    return q.apply(pos) + origin, q * rot


def _pose3(rot, translation):
    return gtsam.Pose3(gtsam.Rot3(rot.as_matrix()), translation)


def build_factor_graph(qlio_trans, enugpsi, stati, qlio, poslio):
    """Build the pose graph: per-node GNSS arm factor, sequential odometry
    BetweenFactor, and one loop-closure BetweenFactor per node when a spatially
    close but temporally distant neighbour exists.

    Returns (graph, initials, loop_pairs (M, 2)).
    """
    noise_pose = gtsam.noiseModel.Diagonal.Sigmas(
        np.concatenate([SIGMA_ATT * np.ones(3), SIGMA_POS * np.ones(3)])
    )
    noise_gnss = gtsam.noiseModel.Diagonal.Sigmas(SIGMA_GNSS * np.ones(3))

    graph = gtsam.NonlinearFactorGraph()
    initials = gtsam.Values()
    time_gap_samples = RELATIVE_TIME_TH * LIDAR_HZ
    loop_pairs = []
    n = len(qlio_trans)

    for i in range(n):
        # Initial pose: yaw-aligned LiDAR attitude, GNSS-interpolated position
        initials.insert(P(i), _pose3(qlio_trans[i], enugpsi[i]))

        # GNSS factor (only at fix quality and finite interp)
        if stati[i] <= STAT_TH and np.all(np.isfinite(enugpsi[i])):
            graph.add(make_gps_factor_arm(P(i), enugpsi[i], ARM, noise_gnss))

        # Sequential between-pose factor from raw LiDAR odometry
        if i > 0:
            rel = _pose3(qlio[i - 1], poslio[i - 1]).between(
                _pose3(qlio[i], poslio[i])
            )
            graph.add(gtsam.BetweenFactorPose3(P(i - 1), P(i), rel, noise_pose))

        # Loop-closure-style relative factor between distant-in-time nodes
        dist = np.linalg.norm(poslio - poslio[i], axis=1)
        order = np.argsort(dist)
        cand = np.where(
            (dist[order] < RELATIVE_DIST_TH)
            & (np.abs(order - i) > time_gap_samples)
        )[0]
        if len(cand):
            j = int(order[cand[0]])
            rel = _pose3(qlio[i], poslio[i]).between(
                _pose3(qlio[j], poslio[j])
            )
            graph.add(gtsam.BetweenFactorPose3(P(i), P(j), rel, noise_pose))
            loop_pairs.append((i, j))

    loop_pairs = (
        np.array(loop_pairs) if loop_pairs else np.empty((0, 2), dtype=int)
    )
    return graph, initials, loop_pairs


def optimize(graph, initials):
    """Run Levenberg-Marquardt, print stats, return optimized Values."""
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosity("TERMINATION")
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initials, params)

    print("optimization...")
    print(f"Initial Error: {optimizer.error():.2f}")
    t0 = time.perf_counter()
    results = optimizer.optimize()
    print(f"Error: {optimizer.error():.2f} Iter: {optimizer.iterations()}")
    print(f"Elapsed: {time.perf_counter() - t0:.3f} s")
    return results


def extract_estimates(results, n):
    """Pull (xyz, rpy_deg, quat_xyzw) for each P(i) from the optimized Values."""
    x = np.full((n, 3), np.nan)
    rpy = np.full((n, 3), np.nan)
    q = np.full((n, 4), np.nan)
    for i in range(n):
        pose = results.atPose3(P(i))
        x[i] = pose.translation()
        rpy[i] = np.rad2deg(pose.rotation().rpy())
        qq = pose.rotation().toQuaternion()
        # MATLAB ".coeffs()" follows the Eigen convention (x, y, z, w)
        q[i] = [qq.x(), qq.y(), qq.z(), qq.w()]
    return x, rpy, q


# ---------------- Mapping Processing ----------------

def filter_points(pc, tsunix, teunix, t_traj_min, t_traj_max):
    """Apply MATLAB-equivalent range / intensity / time / trajectory filters.

    Returns (xyz, intensity, tunix) — all masked to the surviving points.
    """
    xyz = np.column_stack([pc["x"], pc["y"], pc["z"]]).astype(np.float32)
    intensity = pc["intensity"]
    tunix = pc["timestamp"]
    rng = np.linalg.norm(xyz, axis=1)

    # MATLAB reject mask was rng<=min OR rng>=max OR int<=th OR t<=ts OR t>te;
    # the complement is the keep mask used here.
    keep = (
        (rng > RNG_MIN)
        & (rng < RNG_MAX)
        & (intensity > INTENSITY_TH)
        & (tunix > tsunix)
        & (tunix <= teunix)
    )
    # MATLAB interp1 returns NaN outside the trajectory range, and pcwrite
    # silently drops NaN points -> equivalent to clipping here.
    keep &= (tunix >= t_traj_min) & (tunix <= t_traj_max)
    return xyz[keep], intensity[keep], tunix[keep]


def interpolate_pose(tunix, timeunix, pos, quat_xyzw):
    """Linear-interp position and slerp orientation at each `tunix`.

    np.interp is used instead of scipy.interpolate.interp1d which has a known
    failure mode on huge query arrays in scipy 1.17.
    """
    pos_interp = np.empty((len(tunix), 3), dtype=np.float32)
    for k in range(3):
        pos_interp[:, k] = np.interp(tunix, timeunix, pos[:, k])
    quat_interp = Slerp(timeunix, Rotation.from_quat(quat_xyzw))(tunix)
    return pos_interp, quat_interp


def transform_to_world(xyz_sensor, pos_interp, quat_interp):
    """Sensor frame -> world frame, dropping any NaN/Inf-valued points.

    Equivalent to MATLAB `rotateframe(conj(q), xyz) + p` (active rotation by q).
    Returns (xyz_world, finite_mask) where the mask is needed by the caller to
    align other per-point arrays (e.g. intensity).
    """
    xyz_world = quat_interp.apply(xyz_sensor).astype(np.float32) + pos_interp
    finite = np.isfinite(xyz_world).all(axis=1)
    if not finite.all():
        print(f"  dropping {int((~finite).sum())} NaN/Inf points")
    return xyz_world[finite], finite


# ---------------- Colorization Processing ----------------


def apply_pixel_colors_to_vertices_vectorized(
    image,
    rot_image,
    pos,
    K,
    Verticies,
    Z
):
    """
    Assign pixel colors to vertices using simple depth-based occlusion.

    For each image pixel:
        1. Find all vertices projected into that pixel.
        2. Find the closest vertex in camera-space depth.
        3. Apply the pixel color to that closest vertex.
        4. Apply the same color to vertices within Z depth units
           behind the closest vertex.

    Parameters
    ----------
    image : np.ndarray
        Image of shape (height, width, 3).

    height : int
        Image height.

    width : int
        Image width.

    rot_image : np.ndarray
        Camera rotation matrix, shape (3, 3).

    pos : np.ndarray
        Camera position, shape (3,).

    K : np.ndarray
        OpenCV camera intrinsic matrix:

            [[fx,  0, cx],
             [ 0, fy, cy],
             [ 0,  0,  1]]

    Verticies : np.ndarray
        Vertex array. Columns 0:3 are XYZ coordinates and
        columns 3:6 are RGB colors.

    Z : float
        Depth thickness used to tolerate noisy geometry.

        Z = 0:
            Only the closest vertex in each pixel receives
            the pixel color.

        Z > 0:
            Vertices up to Z units behind the closest vertex
            also receive the pixel color.

    Returns
    -------
    np.ndarray
        Updated Verticies array.
    """

    # Image info
    height, width, _ = pixels.shape
    # ------------------------------------------------------------
    # 1. Camera intrinsics
    # ------------------------------------------------------------
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    # ------------------------------------------------------------
    # 2. Transform vertices into camera coordinates
    # ------------------------------------------------------------
    verts_world = Verticies[:, :3]

    verts_cam = (
        rot_image.T @ (verts_world - pos).T
    ).T

    x = verts_cam[:, 0]
    y = verts_cam[:, 1]
    depth = verts_cam[:, 2]

    # Only vertices in front of camera
    valid = depth > 0

    idx_valid = np.where(valid)[0]

    x = x[valid]
    y = y[valid]
    depth = depth[valid]

    if len(idx_valid) == 0:
        return Verticies

    # ------------------------------------------------------------
    # 3. Project vertices using OpenCV intrinsics
    # ------------------------------------------------------------
    u = fx * (x / depth) + cx
    v = fy * (y / depth) + cy

    # ------------------------------------------------------------
    # 4. Keep vertices inside image
    # ------------------------------------------------------------
    in_bounds = (
        (u >= 0)
        & (u < width)
        & (v >= 0)
        & (v < height)
    )

    u = u[in_bounds]
    v = v[in_bounds]
    depth = depth[in_bounds]
    idx_valid = idx_valid[in_bounds]

    if len(idx_valid) == 0:
        return Verticies

    # ------------------------------------------------------------
    # 5. Determine which pixel each vertex belongs to
    # ------------------------------------------------------------
    px = np.floor(u).astype(int)
    py = np.floor(v).astype(int)

    pixel_indices = py * width + px

    # ------------------------------------------------------------
    # 6. Sort vertices by pixel and then by depth
    # ------------------------------------------------------------
    #
    # The first vertex in each pixel group will therefore be
    # the closest vertex to the camera.
    #
    order = np.lexsort((depth, pixel_indices))

    pixel_indices = pixel_indices[order]
    depth = depth[order]
    idx_valid = idx_valid[order]

    # ------------------------------------------------------------
    # 7. Find the closest depth for every pixel
    # ------------------------------------------------------------
    unique_pixels, start_idx, counts = np.unique(
        pixel_indices,
        return_index=True,
        return_counts=True
    )

    # Since the vertices are sorted by depth within each pixel,
    # the first entry of each group is the closest vertex.
    closest_depth = depth[start_idx]

    # ------------------------------------------------------------
    # 8. Determine which vertices receive the pixel color
    # ------------------------------------------------------------
    #
    # Map each vertex to the closest depth of its pixel.
    #
    group_ids = np.repeat(
        np.arange(len(unique_pixels)),
        counts
    )

    closest_depth_per_vertex = closest_depth[group_ids]

    # A vertex is visible/colorable if it is no more than Z
    # behind the closest vertex.
    #
    # Because the vertices are sorted by depth, there cannot be
    # be a vertex in front of closest_depth_per_vertex.
    keep_mask = (
        depth <= closest_depth_per_vertex + Z
    )

    selected_vertices = idx_valid[keep_mask]
    selected_pixels = pixel_indices[keep_mask]

    # ------------------------------------------------------------
    # 9. Assign pixel colors
    # ------------------------------------------------------------
    colors = Verticies[:, 3:6].copy()

    pixel_colors = image.reshape(-1, 3)[selected_pixels]

    colors[selected_vertices] = pixel_colors

    Verticies[:, 3:6] = colors

    return Verticies

def filename_to_unix(filename):
    # Remove extension
    name = os.path.splitext(filename)[0]
    
    # Split parts
    parts = name.split('_')
    
    # Extract date, time, milliseconds
    date_part = parts[0]        # YYYYMMDD
    time_part = parts[1]        # HHMMSS
    millis_part = parts[2]      # milliseconds
    
    # Combine into datetime string
    dt_str = f"{date_part}{time_part}{millis_part}"
    
    # Parse into datetime object
    dt = datetime.strptime(dt_str, "%Y%m%d%H%M%S%f")
    
    # Convert to Unix timestamp (seconds)
    unix_time = dt.timestamp()
    
    return unix_time

def indexfromtime(times, timestamp):
    # Find the closest index
    times_dist = np.absolute(times - timestamp)
    # Get least deviation
    return np.argmin(times_dist)


# ---------------- Output ----------------

def write_ply(path: Path, xyz: np.ndarray, intensity: np.ndarray):
    """Binary little-endian PLY with `float x, y, z + uchar intensity`."""
    vertex = np.empty(
        len(xyz),
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "u1")],
    )
    vertex["x"] = xyz[:, 0]
    vertex["y"] = xyz[:, 1]
    vertex["z"] = xyz[:, 2]
    vertex["intensity"] = intensity
    PlyData(
        [PlyElement.describe(vertex, "vertex")],
        text=False,
        byte_order="<",
    ).write(path)

def save_ply_rgb(vertices, intensity, path):
    """
    Save XYZ + grayscale as standard RGB .ply (appears gray in viewers).
    vertices: (N, 7) array [x, y, z, r, g, b, intensity]
    """

    vertex_dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                    ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
                    ('intensity', 'u1')]

    rgb = vertices[:, 3:6].astype(np.uint8)

    data['red'] = rgb[:, 0]
    data['green'] = rgb[:, 1]
    data['blue'] = rgb[:, 2]
    data['x'], data['y'], data['z'] = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    data['intensity'] = intensity
    #data['red'] = data['green'] = data['blue'] = gray

    el = PlyElement.describe(data, 'vertex')
    PlyData([el], text=False, byte_order="<",).write(path)
    print(f"Saved {vertices.shape[0]} vertices with grayscale (RGB) color to {path}")


# ---------------- Orchestration ----------------

def main(args):
    tlio, poslio, qlio = read_glim_trajectory(args.glim)
    tgps, enugps, stat_gps = read_gnss_solution(args.gnss, ORGLLH)
    enugpsi, stati = interpolate_gnss(tgps, enugps, stat_gps, tlio)
    poslio_trans, qlio_trans = apply_initial_yaw(
        poslio, qlio, INITIAL_YAW_DEG, enugpsi[0]
    )
    tsunix = _gpst_to_unix(TS_GPST)
    teunix = _gpst_to_unix(TE_GPST)

    graph, initials, loop_pairs = build_factor_graph(
        qlio_trans, enugpsi, stati, qlio, poslio
    )
    results = optimize(graph, initials)
    x_est, rpy_est, q_est = extract_estimates(results, len(tlio))
    
    print("Reading rosbag...")
    t0 = time.perf_counter()
    pc = read_pointcloud(args.bag, skip=SKIP)
    print(f"  total points: {len(pc):,}  ({time.perf_counter()-t0:.2f}s)")

    print("Selecting points...")
    xyz, intensity, tunix = filter_points(
        pc, tsunix, teunix, tlio[0], tlio[-1]
    )
    print(f"  after filter: {len(xyz):,} / {len(pc):,}")

    print("Pose interpolation...")
    pos_interp, quat_interp = interpolate_pose(tunix, tlio, x_est, q_est)

    print("Transformation...")
    xyz_world, finite = transform_to_world(xyz, pos_interp, quat_interp)
    intensity = intensity[finite]

    print("Colorization...")
    vertex_stack = np.stack([
        xyz_world[:, 0],
        xyz_world[:, 1],
        xyz_world[:, 2],
        np.zeros(ply_data['vertex'].data['x'].size),  # R
        np.zeros(ply_data['vertex'].data['x'].size),  # G
        np.zeros(ply_data['vertex'].data['x'].size)   # B
    ], axis=-1)

    #times = tunix
    #pos = pos_interp
    #quat = quat_interp

    for filename in os.listdir(args.img):
        if filename.lower().endswith((".jpg", ".jpeg")):
            filepath = os.path.join(args.img, filename)
            print(f"\nProcessing file: {filename}")

            # Load image (RGB)
            image = Image.open(filepath).convert("RGB")
            pixels = np.array(image)  # shape (H, W, 3)
            

            # NOTE: Generate timestamp from filename and convert to unix time
            timestamp = filename_to_unix(filename)

            if (timestamp < tunix[0]) or (timestamp > tunix[-1])
                #image is outside of the timestamp range.
                continue

            position_index = indexfromtime(tunix, timestamp)

            pageposition = pos_interp[position_index]

            q_lidar = quat_interp[position_index]
            rot_world = Rotation.from_quat(q_lidar)
            rot_image = np.matmul(rot_world,ROT_CAM)

            vertex_stack = apply_pixel_colors_to_vertices_vectorized(
                pixels,
                rot_image, pageposition,
                K,
                vertex_stack, 0.05
            )

    print("Writing pointcloud...")
    write_ply(vertex_stack, intensity, args.out)
    print(f"Wrote {args.out}  ({len(xyz_world):,} points)")


def parse_args():
    default_out = (
        "pointcloud_merge.ply" if SKIP == 1 else f"pointcloud_skip{SKIP}.ply"
    )
    p = argparse.ArgumentParser(
        description="Fuse GLIM LiDAR odometry with GNSS via a GTSAM pose graph."
    )
    p.add_argument(
        "--glim", type=Path, default=BASEPATH / "traj_lidar.txt",
        help="GLIM trajectory input (default: data/traj_lidar.txt)",
    )
    p.add_argument(
        "--gnss", type=Path, default=BASEPATH / "asterx.pos",
        help="RTKLib .pos GNSS solution input (default: data/asterx.pos)",
    )
    p.add_argument(
        "--bag", type=Path, default=BASEPATH / "lidar_0.mcap",
        help="LiDAR rosbag (mcap) input (default: data/lidar/lidar_0.mcap)",
    )
    p.add_argument(
        "--img", type=Path, default=BASEPATH / "img",
        help="Image directory input (default: data/img/(images go here)",
    )
    p.add_argument(
        "--out", type=Path, default=BASEPATH / default_out,
        help=f"output PLY (default: data/{default_out})",
    )
    return p.parse_args()


if __name__ == "__main__":
    main(parse_args())

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for headless environments
import matplotlib.pyplot as plt
import signal

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# ----------------------------------------------------------------------
# Constants
# ----------------------------------------------------------------------
TOPIC_GT  = "/vrpn/H4_PX1/pose"
TOPIC_EKF = "/mavros/local_position/pose"
TOPIC_SP  = "/mavros/setpoint_raw/target_local"
TOPIC_VIS = "/mavros/vision_pose/pose"       

DT = 0.02
THRESHOLD_SP_VALID = 0.1

OUTPUT_DIR = os.path.expanduser("~/lidar_perception_analyse")

# ----------------------------------------------------------------------
# Safe exit on Ctrl+C
# ----------------------------------------------------------------------
def sigint_handler(sig, frame):
    print("\n[Interrupted] Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

# ----------------------------------------------------------------------
# Quaternion utilities
# ----------------------------------------------------------------------
def qnp(q):
    """Convert ROS quaternion message to numpy array."""
    return np.array([q.x, q.y, q.z, q.w])

def qnorm(q):
    """Normalize a quaternion."""
    return q / (np.linalg.norm(q) + 1e-12)

def q_to_euler_xyz(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    Uses ZYX convention (intrinsic).
    """
    q = qnorm(q)
    x, y, z, w = q

    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr, cosr)

    sinp = 2 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny, cosy)

    return np.array([roll, pitch, yaw])

def quat_array_to_euler(q_array):
    """Apply quaternion to Euler conversion over an array."""
    return np.array([q_to_euler_xyz(q) for q in q_array])

# ----------------------------------------------------------------------
# Rosbag reader
# ----------------------------------------------------------------------
def read_bag_full(path):
    """Read all position and orientation data from a rosbag."""
    storage = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")
    conv = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = {TOPIC_GT: [], TOPIC_EKF: [], TOPIC_SP: [], TOPIC_VIS: []}

    while reader.has_next():
        topic, raw, t = reader.read_next()

        if topic not in data:
            continue

        msg_type = get_message(types[topic])
        msg = deserialize_message(raw, msg_type)

        stamp = t * 1e-9  # nanoseconds to seconds

        if topic == TOPIC_SP:
            p = np.array([msg.position.x, msg.position.y, msg.position.z])
            q = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            p = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            q = qnp(msg.pose.orientation)

        # Skip entries containing NaNs
        if np.any(np.isnan(p)) or np.any(np.isnan(q)):
            continue

        data[topic].append((stamp, p, q))

    return data

def to_numpy_full(data_list):
    """Convert list of (time, position, quaternion) to numpy arrays."""
    t = np.array([x[0] for x in data_list])
    p = np.array([x[1] for x in data_list])
    q = np.array([x[2] for x in data_list])
    return t, p, q

# ----------------------------------------------------------------------
# Interpolation
# ----------------------------------------------------------------------
def interp_pos(t, x, tn):
    """Linear interpolation for 3D positions."""
    out = np.zeros((len(tn), 3))
    for i in range(3):
        out[:, i] = np.interp(tn, t, x[:, i])
    return out

def interp_quat(t, q, tn):
    """
    Spherical linear interpolation (slerp) for quaternion arrays.
    """
    out = []
    for tt in tn:
        i = np.searchsorted(t, tt) - 1
        i = np.clip(i, 0, len(t) - 2)

        t0, t1 = t[i], t[i + 1]
        q0, q1 = q[i], q[i + 1]

        a = (tt - t0) / (t1 - t0 + 1e-12)

        dot = np.clip(np.dot(q0, q1), -1.0, 1.0)
        if dot < 0:
            q1 = -q1
            dot = -dot

        theta = np.arccos(dot)
        if theta < 1e-6:
            q_interp = (1 - a) * q0 + a * q1
        else:
            q_interp = (
                np.sin((1 - a) * theta) / np.sin(theta) * q0 +
                np.sin(a * theta) / np.sin(theta) * q1
            )

        out.append(qnorm(q_interp))

    return np.array(out)

# ----------------------------------------------------------------------
# SE(3) alignment (position only)
# ----------------------------------------------------------------------
def align_se3(A, B):
    """
    Find rigid transformation (R, t) that maps A to B minimizing ||B - (R*A + t)||.
    """
    muA = A.mean(axis=0)
    muB = B.mean(axis=0)

    H = (A - muA).T @ (B - muB)
    U, _, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2] *= -1
        R = Vt.T @ U.T

    t = muB - R @ muA
    return R, t

# ----------------------------------------------------------------------
# Main analysis
# ----------------------------------------------------------------------
def main(bag_path):
    # Create output directory (root-level)
    try:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
    except PermissionError:
        print(f"[Error] Cannot create/write to {OUTPUT_DIR}. Check permissions.")
        sys.exit(1)

    print("[1/4] Reading rosbag ...")
    data = read_bag_full(bag_path)

    # Check if visual odometry topic exists
    if not data[TOPIC_VIS]:
        print(f"[Warning] No messages on {TOPIC_VIS}. Visual odometry analysis will be skipped.")
        has_vis = False
    else:
        has_vis = True

    t_g, p_g, q_g = to_numpy_full(data[TOPIC_GT])
    t_e, p_e, q_e = to_numpy_full(data[TOPIC_EKF])
    t_s, p_s, _   = to_numpy_full(data[TOPIC_SP])
    if has_vis:
        t_v, p_v, q_v = to_numpy_full(data[TOPIC_VIS])
    else:
        # Dummy arrays to avoid NameError later
        t_v, p_v, q_v = np.array([]), np.empty((0,3)), np.empty((0,4))

    # Common time interval
    all_start = [t_g[0], t_e[0], t_s[0]]
    all_end   = [t_g[-1], t_e[-1], t_s[-1]]
    if has_vis:
        all_start.append(t_v[0])
        all_end.append(t_v[-1])

    t0 = max(all_start)
    t1 = min(all_end)

    if t0 >= t1:
        print("[Error] No overlapping time range. Exiting.")
        return

    t_common = np.arange(t0, t1, DT)
    print(f"      Common time span: {t0:.2f} to {t1:.2f} s")

    # Interpolate all signals onto common time grid
    pg = interp_pos(t_g, p_g, t_common)
    pe = interp_pos(t_e, p_e, t_common)
    ps = interp_pos(t_s, p_s, t_common)

    qg = interp_quat(t_g, q_g, t_common)
    qe = interp_quat(t_e, q_e, t_common)

    if has_vis:
        pv = interp_pos(t_v, p_v, t_common)
        qv = interp_quat(t_v, q_v, t_common)

    # Extract segment where setpoint magnitude is above threshold
    idx = np.where(np.linalg.norm(ps, axis=1) > THRESHOLD_SP_VALID)[0]
    if len(idx) == 0:
        print("[Error] No valid setpoint segment found. Exiting.")
        return

    s, e = idx[0], idx[-1]
    print(f"      Valid segment indices: {s} to {e}")

    t_common = t_common[s:e]
    pg = pg[s:e]
    pe = pe[s:e]
    ps = ps[s:e]
    qg = qg[s:e]
    qe = qe[s:e]
    if has_vis:
        pv = pv[s:e]
        qv = qv[s:e]

    # ------------------------------------------------------------------
    # Attitude error analysis
    # ------------------------------------------------------------------
    print("\n[2/4] Analyzing attitude errors (Euler angles) ...")

    for i in range(len(qg)):
        if np.dot(qg[i], qe[i]) < 0:
            qe[i] = -qe[i]
        if has_vis and np.dot(qg[i], qv[i]) < 0:
            qv[i] = -qv[i]
            
    euler_gt  = quat_array_to_euler(qg)
    euler_ekf = quat_array_to_euler(qe)

    def wrap_angle_error(euler_est):
        err = euler_est - euler_gt
        err[:, 0] = (err[:, 0] + np.pi) % (2 * np.pi) - np.pi
        err[:, 1] = (err[:, 1] + np.pi) % (2 * np.pi) - np.pi
        yaw = err[:, 2]
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        yaw_alt = yaw - np.sign(yaw) * np.pi
        yaw = np.where(np.abs(yaw_alt) < np.abs(yaw), yaw_alt, yaw)
        err[:, 2] = yaw
        return err

    err_ekf = wrap_angle_error(euler_ekf)
    err_ekf_deg = err_ekf * 180.0 / np.pi
    mean_ekf_deg = np.mean(err_ekf_deg, axis=0)
    print(f"      EKF  Signed mean (bias):   Roll={mean_ekf_deg[0]:.2f}°, "
          f"Pitch={mean_ekf_deg[1]:.2f}°, Yaw={mean_ekf_deg[2]:.2f}°")

    if has_vis:
        euler_vis = quat_array_to_euler(qv)
        err_vis = wrap_angle_error(euler_vis)
        err_vis_deg = err_vis * 180.0 / np.pi
        mean_vis_deg = np.mean(err_vis_deg, axis=0)
        print(f"      VIS  Signed mean (bias):   Roll={mean_vis_deg[0]:.2f}°, "
              f"Pitch={mean_vis_deg[1]:.2f}°, Yaw={mean_vis_deg[2]:.2f}°")

    # Plot 1: Attitude error
    fig1 = plt.figure("Attitude Error", figsize=(12, 8))
    labels = ["Roll (X)", "Pitch (Y)", "Yaw (Z)"]
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(t_common, err_ekf_deg[:, i], label="EKF error")
        if has_vis:
            plt.plot(t_common, err_vis_deg[:, i], label="VIS error")
        plt.axhline(0, color="black", linewidth=0.8)
        plt.axhline(mean_ekf_deg[i], linestyle="--", linewidth=2,
                    label=f"EKF mean = {mean_ekf_deg[i]:.2f}°")
        if has_vis:
            plt.axhline(mean_vis_deg[i], linestyle="--", linewidth=2,
                        label=f"VIS mean = {mean_vis_deg[i]:.2f}°")
        plt.grid(True)
        plt.legend()
        plt.ylabel("deg")
    plt.xlabel("Time [s]")
    plt.suptitle("Euler Angle Error (EKF and VIS vs GT)")
    plt.tight_layout()
    out1 = os.path.join(OUTPUT_DIR, "attitude_error.png")
    plt.savefig(out1, dpi=150)
    plt.close(fig1)
    print(f"      Saved plot -> {out1}")

    # ------------------------------------------------------------------
    # Position error analysis
    # ------------------------------------------------------------------
    print("\n[3/4] Analyzing position errors ...")
    # Align EKF and SP to GT
    R_ekf, t_ekf_off = align_se3(pe, pg)
    pe_aligned = (R_ekf @ pe.T).T + t_ekf_off

    R_sp, t_sp_off = align_se3(ps, pg)
    ps_aligned = (R_sp @ ps.T).T + t_sp_off

    if has_vis:
        R_vis, t_vis_off = align_se3(pv, pg)
        pv_aligned = (R_vis @ pv.T).T + t_vis_off

    # Errors
    err_ekf = np.linalg.norm(pe_aligned - pg, axis=1)
    err_sp  = np.linalg.norm(ps_aligned - pg, axis=1)
    if has_vis:
        err_vis_pos = np.linalg.norm(pv_aligned - pg, axis=1)

    ekf_rmse = np.sqrt(np.mean(err_ekf ** 2))
    sp_rmse  = np.sqrt(np.mean(err_sp ** 2))
    ekf_mean = np.mean(err_ekf)
    sp_mean  = np.mean(err_sp)

    print(f"      EKF  RMSE: {ekf_rmse:.3f} m,  Mean: {ekf_mean:.3f} m")
    print(f"      SP   RMSE: {sp_rmse:.3f} m,  Mean: {sp_mean:.3f} m")
    if has_vis:
        vis_rmse = np.sqrt(np.mean(err_vis_pos ** 2))
        vis_mean = np.mean(err_vis_pos)
        print(f"      VIS  RMSE: {vis_rmse:.3f} m,  Mean: {vis_mean:.3f} m")

    # Plot 2: XYZ trajectories
    fig2 = plt.figure("XYZ Trajectories", figsize=(10, 8))
    axis_labels = ['X', 'Y', 'Z']
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(t_common, pg[:, i], label="GT")
        plt.plot(t_common, pe_aligned[:, i], label="EKF")
        plt.plot(t_common, ps_aligned[:, i], label="SP")
        if has_vis:
            plt.plot(t_common, pv_aligned[:, i], label="VIS")
        plt.ylabel(axis_labels[i])
        plt.legend()
        plt.grid(True)
    plt.xlabel("Time (s)")
    plt.suptitle("XYZ Position vs Time (after SE(3) alignment)")
    plt.tight_layout()
    out2 = os.path.join(OUTPUT_DIR, "xyz_trajectories.png")
    plt.savefig(out2, dpi=150)
    plt.close(fig2)
    print(f"      Saved plot -> {out2}")

    # Plot 3: Position error magnitude
    fig3 = plt.figure("Position Error", figsize=(10, 6))
    plt.plot(t_common, err_ekf, label="EKF Error")
    plt.plot(t_common, err_sp,  label="SP Error")
    if has_vis:
        plt.plot(t_common, err_vis_pos, label="VIS Error")
    plt.axhline(ekf_mean, linestyle="--", linewidth=2,
                label=f"EKF Mean = {ekf_mean:.3f} m")
    plt.axhline(sp_mean, linestyle="--", linewidth=2,
                label=f"SP Mean = {sp_mean:.3f} m")
    if has_vis:
        plt.axhline(vis_mean, linestyle="--", linewidth=2,
                    label=f"VIS Mean = {vis_mean:.3f} m")
    plt.legend()
    plt.grid(True)
    plt.title("Tracking Error with Mean Reference Lines")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.tight_layout()
    out3 = os.path.join(OUTPUT_DIR, "position_error.png")
    plt.savefig(out3, dpi=150)
    plt.close(fig3)
    print(f"      Saved plot -> {out3}")

    print(f"\n[4/4] Analysis complete. All figures saved in '{OUTPUT_DIR}'.")
    print("       Existing files were overwritten.")


# ----------------------------------------------------------------------
# Script entry point
# ----------------------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_rosbag_merged.py <bag_path>")
        sys.exit(1)
    main(sys.argv[1])
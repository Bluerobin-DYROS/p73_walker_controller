import numpy as np
import matplotlib.pyplot as plt

# =========================
# Load log data
# =========================
joint_desired = np.loadtxt("data/joint_desired_log.txt")   # [T, 13]
joint_position = np.loadtxt("data/joint_position_log.txt") # [T, 13]
joint_velocity = np.loadtxt("data/joint_velocity_log.txt") # [T, 13]
torque_sum = np.loadtxt("data/torque_sum_log.txt")         # [T, 26]
foot_traj = np.loadtxt("data/foot_traj_log.txt")           # [T, 12]

# =========================
# Basic checks
# =========================
num_dof = 13

if joint_desired.ndim == 1:
    joint_desired = joint_desired.reshape(1, -1)
if joint_position.ndim == 1:
    joint_position = joint_position.reshape(1, -1)
if joint_velocity.ndim == 1:
    joint_velocity = joint_velocity.reshape(1, -1)
if torque_sum.ndim == 1:
    torque_sum = torque_sum.reshape(1, -1)
if foot_traj.ndim == 1:
    foot_traj = foot_traj.reshape(1, -1)

assert joint_desired.shape[1] == num_dof, f"joint_desired_log must have {num_dof} columns"
assert joint_position.shape[1] == num_dof, f"joint_position_log must have {num_dof} columns"
assert joint_velocity.shape[1] == num_dof, f"joint_velocity_log must have {num_dof} columns"
assert torque_sum.shape[1] == 2 * num_dof, f"torque_sum_log must have {2*num_dof} columns"
assert foot_traj.shape[1] == 12, "foot_traj_log must have 12 columns"

torque_desired = torque_sum[:, :num_dof]
torque_actual = torque_sum[:, num_dof:]

# =========================
# Foot trajectory split
# =========================
lf_traj = foot_traj[:, 0:3]    # Left foot desired x, y, z
rf_traj = foot_traj[:, 3:6]    # Right foot desired x, y, z
lf_pos  = foot_traj[:, 6:9]    # Left foot actual x, y, z
rf_pos  = foot_traj[:, 9:12]   # Right foot actual x, y, z

# =========================
# Time axis
# =========================
T = joint_desired.shape[0]
time = np.arange(T)

# =========================
# Joint names
# =========================
joint_names = [
    "J1", "J2", "J3", "J4", "J5", "J6", "J7",
    "J8", "J9", "J10", "J11", "J12", "J13"
]

# =========================
# Plot joint logs
# =========================
fig, axes = plt.subplots(num_dof, 3, figsize=(20, 3 * num_dof), sharex=True)
fig.suptitle("Joint Logs (13 DOF)", fontsize=16)

for i in range(num_dof):
    # Position
    axes[i, 0].plot(time, joint_desired[:, i], label="q_des")
    axes[i, 0].plot(time, joint_position[:, i], label="q")
    axes[i, 0].set_ylabel(joint_names[i])
    axes[i, 0].grid(True)
    if i == 0:
        axes[i, 0].set_title("Position")
        axes[i, 0].legend()

    # Velocity
    axes[i, 1].plot(time, joint_velocity[:, i], label="qdot")
    axes[i, 1].grid(True)
    if i == 0:
        axes[i, 1].set_title("Velocity")

    # Torque
    axes[i, 2].plot(time, torque_desired[:, i], label="tau_des")
    axes[i, 2].plot(time, torque_actual[:, i], label="tau_meas")
    axes[i, 2].grid(True)
    if i == 0:
        axes[i, 2].set_title("Torque")
        axes[i, 2].legend()

for j in range(3):
    axes[-1, j].set_xlabel("Time step")

plt.tight_layout()
plt.show()

# =========================
# Plot foot trajectory logs
# =========================
coord_names = ["x", "y", "z"]

fig2, axes2 = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
fig2.suptitle("Foot Trajectory", fontsize=16)

for i in range(3):
    # Left foot
    axes2[i, 0].plot(time, lf_traj[:, i], label="traj")
    axes2[i, 0].plot(time, lf_pos[:, i], label="pos")
    axes2[i, 0].set_ylabel(coord_names[i])
    axes2[i, 0].grid(True)
    if i == 0:
        axes2[i, 0].set_title("Left Foot")
        axes2[i, 0].legend()

    # Right foot
    axes2[i, 1].plot(time, rf_traj[:, i], label="traj")
    axes2[i, 1].plot(time, rf_pos[:, i], label="pos")
    axes2[i, 1].set_ylabel(coord_names[i])
    axes2[i, 1].grid(True)
    if i == 0:
        axes2[i, 1].set_title("Right Foot")
        axes2[i, 1].legend()

axes2[-1, 0].set_xlabel("Time step")
axes2[-1, 1].set_xlabel("Time step")

plt.tight_layout()
plt.show()
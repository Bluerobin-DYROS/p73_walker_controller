import numpy as np
import matplotlib.pyplot as plt

# =========================
# Load log data
# =========================
joint_desired = np.loadtxt("data/joint_desired_log.txt")   # [T, 13]
joint_position = np.loadtxt("data/joint_position_log.txt") # [T, 13]
joint_velocity = np.loadtxt("data/joint_velocity_log.txt") # [T, 13]
torque_sum = np.loadtxt("data/torque_joint_log.txt")         # [T, 26]

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

assert joint_desired.shape[1] == num_dof, f"joint_desired_log must have {num_dof} columns"
assert joint_position.shape[1] == num_dof, f"joint_position_log must have {num_dof} columns"
assert joint_velocity.shape[1] == num_dof, f"joint_velocity_log must have {num_dof} columns"
assert torque_sum.shape[1] == 2 * num_dof, f"torque_sum_log must have {2*num_dof} columns"

torque_desired = torque_sum[:, :num_dof]
torque_actual = torque_sum[:, num_dof:]

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
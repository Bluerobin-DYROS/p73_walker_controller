import numpy as np
import matplotlib.pyplot as plt

# =========================
# Load log data
# =========================
joint_desired = np.loadtxt("data/joint_desired_log.txt")       # [T, 13]
joint_position = np.loadtxt("data/joint_position_log.txt")     # [T, 13]
joint_velocity = np.loadtxt("data/joint_velocity_log.txt")     # [T, 13]
torque_sum = np.loadtxt("data/torque_joint_log.txt")           # [T, 26]

joint_desired_005 = np.loadtxt("data_amplitude_0.05/joint_desired_log.txt")   # [T, 13]
joint_position_005 = np.loadtxt("data_amplitude_0.05/joint_position_log.txt") # [T, 13]
joint_velocity_005 = np.loadtxt("data_amplitude_0.05/joint_velocity_log.txt") # [T, 13]
torque_sum_005 = np.loadtxt("data_amplitude_0.05/torque_joint_log.txt")       # [T, 26]

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
if joint_desired_005.ndim == 1:
    joint_desired_005 = joint_desired_005.reshape(1, -1)
if joint_position_005.ndim == 1:
    joint_position_005 = joint_position_005.reshape(1, -1)
if joint_velocity_005.ndim == 1:
    joint_velocity_005 = joint_velocity_005.reshape(1, -1)
if torque_sum_005.ndim == 1:
    torque_sum_005 = torque_sum_005.reshape(1, -1)

assert joint_desired.shape[1] == num_dof, f"joint_desired_log must have {num_dof} columns"
assert joint_position.shape[1] == num_dof, f"joint_position_log must have {num_dof} columns"
assert joint_velocity.shape[1] == num_dof, f"joint_velocity_log must have {num_dof} columns"
assert torque_sum.shape[1] == 2 * num_dof, f"torque_sum_log must have {2*num_dof} columns"
assert joint_desired_005.shape[1] == num_dof, f"joint_desired_log(0.05) must have {num_dof} columns"
assert joint_position_005.shape[1] == num_dof, f"joint_position_log(0.05) must have {num_dof} columns"
assert joint_velocity_005.shape[1] == num_dof, f"joint_velocity_log(0.05) must have {num_dof} columns"
assert torque_sum_005.shape[1] == 2 * num_dof, f"torque_sum_log(0.05) must have {2*num_dof} columns"

torque_desired = torque_sum[:, :num_dof]
torque_actual = torque_sum[:, num_dof:]
torque_desired_005 = torque_sum_005[:, :num_dof]
torque_actual_005 = torque_sum_005[:, num_dof:]

# Align lengths for overlay plotting
T = min(
    joint_desired.shape[0],
    joint_position.shape[0],
    joint_velocity.shape[0],
    torque_sum.shape[0],
    joint_desired_005.shape[0],
    joint_position_005.shape[0],
    joint_velocity_005.shape[0],
    torque_sum_005.shape[0],
)

joint_desired = joint_desired[:T]
joint_position = joint_position[:T]
joint_velocity = joint_velocity[:T]
torque_desired = torque_desired[:T]
torque_actual = torque_actual[:T]

joint_desired_005 = joint_desired_005[:T]
joint_position_005 = joint_position_005[:T]
joint_velocity_005 = joint_velocity_005[:T]
torque_desired_005 = torque_desired_005[:T]
torque_actual_005 = torque_actual_005[:T]

# =========================
# Time axis
# =========================
time = np.arange(T)

# =========================
# Joint names
# =========================
joint_names = [
    "J1", "J2", "J3", "J4", "J5", "J6", "J7",
    "J8", "J9", "J10", "J11", "J12", "J13"
]

# Set the joint index you want to inspect (0-based)
joint_index = 5
assert 0 <= joint_index < num_dof, f"joint_index must be between 0 and {num_dof - 1}"

# Align initial point of "data" to "data_amplitude_0.05" for easier comparison
align_initial_point = False

if align_initial_point:
    pos_offset = joint_position_005[0, joint_index] - joint_position[0, joint_index]
    pos_des_offset = joint_desired_005[0, joint_index] - joint_desired[0, joint_index]

    joint_position[:, joint_index] += pos_offset
    joint_desired[:, joint_index] += pos_des_offset

# =========================
# Plot joint logs
# =========================
fig, axes = plt.subplots(1, 2, figsize=(18, 5), sharex=True)
fig.suptitle(f"Joint Logs Overlay: {joint_names[joint_index]} (index {joint_index})", fontsize=16)

# Position
axes[0].plot(time, joint_desired[:, joint_index], label="q_des (data)")
axes[0].plot(time, joint_position[:, joint_index], label="q (data)")
axes[0].plot(time, joint_desired_005[:, joint_index], "--", label="q_des (amp0.05)")
axes[0].plot(time, joint_position_005[:, joint_index], "--", label="q (amp0.05)")
axes[0].set_title("Position")
axes[0].set_ylabel(joint_names[joint_index])
axes[0].set_xlabel("Time step")
axes[0].grid(True)
axes[0].legend()

# Velocity
axes[1].plot(time, joint_velocity[:, joint_index], label="qdot (data)")
axes[1].plot(time, joint_velocity_005[:, joint_index], "--", label="qdot (amp0.05)")
axes[1].set_title("Velocity")
axes[1].set_xlabel("Time step")
axes[1].grid(True)
axes[1].legend()


plt.tight_layout()
plt.show()
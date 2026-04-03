import numpy as np
import matplotlib.pyplot as plt
import os

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

JOINT_NAMES = [
    "L_Hip_Roll", "L_Hip_Pitch", "L_Hip_Yaw", "L_Knee_Pitch", "L_Ankle_Pitch", "L_Ankle_Roll",
    "R_Hip_Roll", "R_Hip_Pitch", "R_Hip_Yaw", "R_Knee_Pitch", "R_Ankle_Pitch", "R_Ankle_Roll",
]

def load(filename):
    return np.loadtxt(os.path.join(DATA_DIR, filename))

# torque_joint_log: [N, 26] = desired_torque(12) | actual_torque(12)
torque_joint  = load("torque_joint_log.txt")
torque_net    = load("torque_net_log.txt")       # [N, 12] — actuator net output

desired_torque = torque_joint[:, :12]   # PD torque, 12 joints
actual_torque  = torque_joint[:, 12:24] # measured torque, 12 joints

N = min(len(torque_joint), len(torque_net))
desired_torque = desired_torque[:N]
actual_torque  = actual_torque[:N]
torque_net     = torque_net[:N]

t = np.arange(N) * 0.001  # 1000Hz → seconds

fig, axes = plt.subplots(4, 3, figsize=(18, 12))
axes = axes.flatten()

for i, ax in enumerate(axes):
    ax.plot(t, actual_torque[:, i],  label="Actual (target)", linewidth=1.0)
    ax.plot(t, torque_net[:, i],     label="ActuatorNet",     linewidth=1.0, linestyle="--")
    ax.plot(t, desired_torque[:, i], label="PD desired",      linewidth=0.8, alpha=0.5)
    ax.set_title(JOINT_NAMES[i])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Torque [Nm]")
    ax.legend(fontsize=7)
    ax.grid(True)

plt.suptitle("Desired vs ActuatorNet vs Actual Torque", fontsize=14)
plt.tight_layout()
out_path = os.path.join(DATA_DIR, "actuatornet_comparison.png")
plt.savefig(out_path, dpi=150)
plt.show()
print(f"Saved: {out_path}")

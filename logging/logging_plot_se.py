import numpy as np
import matplotlib.pyplot as plt

# =========================
# Load log data
# =========================
state_estimate = np.loadtxt("data/state_estimate_log.txt")  # [T, 12]

# =========================
# Basic checks
# =========================
if state_estimate.ndim == 1:
    state_estimate = state_estimate.reshape(1, -1)

assert state_estimate.shape[1] == 12, "state_estimate_log must have 12 columns"

# Logged order:
# [x_hat(0:6), q_virtual_mjc(0:3), q_dot_virtual_mjc(0:3)]
x_hat = state_estimate[:, 0:6]
q_virtual = state_estimate[:, 6:9]
qdot_virtual = state_estimate[:, 9:12]

# =========================
# Time axis
# =========================
T = state_estimate.shape[0]
time = np.arange(T)

# =========================
# Signal names
# =========================
x_hat_names = [
    "x_hat[0]", "x_hat[1]", "x_hat[2]",
    "x_hat[3]", "x_hat[4]", "x_hat[5]"
]
q_virtual_names = ["q_virtual[0]", "q_virtual[1]", "q_virtual[2]"]
qdot_virtual_names = ["qdot_virtual[0]", "qdot_virtual[1]", "qdot_virtual[2]"]

# =========================
# Plot paired comparisons
# =========================
fig1, axes1 = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig1.suptitle("x_hat[0:3] vs q_virtual[0:3]", fontsize=16)

for i in range(3):
    axes1[i].plot(time, x_hat[:, i], label=x_hat_names[i])
    axes1[i].plot(time, q_virtual[:, i], label=q_virtual_names[i])
    axes1[i].set_ylabel(f"idx {i}")
    axes1[i].grid(True)
    axes1[i].legend(loc="upper right")

axes1[-1].set_xlabel("Time step")
plt.tight_layout()
plt.show()

fig2, axes2 = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig2.suptitle("x_hat[3:6] vs qdot_virtual[0:3]", fontsize=16)

for i in range(3):
    axes2[i].plot(time, x_hat[:, i + 3], label=x_hat_names[i + 3])
    axes2[i].plot(time, qdot_virtual[:, i], label=qdot_virtual_names[i])
    axes2[i].set_ylabel(f"idx {i + 3}")
    axes2[i].grid(True)
    axes2[i].legend(loc="upper right")

axes2[-1].set_xlabel("Time step")
plt.tight_layout()
plt.show()
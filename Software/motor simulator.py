import numpy as np
from scipy.signal import StateSpace, lsim
import matplotlib.pyplot as plt

# Motor characteristics
MomentOfInertia = 0.0000487216 #kg*m^2
ViscusFrictionConstant = 2.981147e-06 #N*m*s
emfConstant = 0.002272 #V/rad/sec
TorqueConstant = emfConstant #N*m*amp
electricResistance = 1.0 #Ohm
electricInductance = 0.5 #H

J = MomentOfInertia
b = ViscusFrictionConstant
Ke = emfConstant
Kt = TorqueConstant
R = electricResistance
L = electricInductance

##
Vcc = 16.0 #V
Current_limit=0.6 #A

# Defining the SS system
# state = [theta*;i]
A = np.array([[-b/J,Kt/J],[-Kt/L,-R/L]])
B = np.array([[0],[1/L]])
C = np.array([[1,0]])
D = np.array([[0]])

sys = StateSpace(A,B,C,D)

# Simulating the system
t = np.linspace(0,20,1000)
u = np.ones(t.shape)
x0 = np.array([0,0])

_,yout,xout = lsim(sys,u,t,x0)

# ── PD Torque Controller (state feedback u = -K*(x-setpoint)) ──────────────────────────
# Gain matrix K: shape (1, 2) — [K_omega, K_i]
# Tune these gains to achieve the desired closed-loop response.
# K[0]: proportional gain on angular velocity (rad/s)  → derivative action
# K[1]: proportional gain on current (A)               → proportional action

K = np.array([[0, 3]])
 
# Control frequency and discrete time step
f_control = 10_000          # Hz
dt        = 1.0 / f_control # s
 
# Simulation duration and reference
t_end    = 20.0             # s
t_cl     = np.arange(0, t_end, dt)
N        = len(t_cl)

# Pre-allocate storage
x_cl = np.zeros((N, 2))    # states  [omega, i]
u_cl = np.zeros(N)          # control input (voltage)
 
x_cl[0] = np.array([0,0])

Torque_Setpoint = 0.0005 #N/m
Setpoint = np.array([0,Torque_Setpoint/Kt])

## Simulation
for k in range(N-1):
    x_k = x_cl[k]
    u_k  = float(-K @ (x_k - Setpoint))          # state feedback: u = -K*x

    # Input voltage saturation
    u_k  = float(np.clip(u_k, -Vcc, Vcc))

    u_cl[k] = u_k
    _, _, xout = lsim(sys, [u_k, u_k], [0.0, dt], X0=x_k)

    x_next = xout[-1].copy()# take only the final state

    # current limit 
    x_next[1] = float(np.clip(x_next[1], -Current_limit, Current_limit))

    x_cl[k+1] = x_next
 
u_cl[-1] = float(-K @ (x_cl[-1] - Setpoint))

omega_cl = x_cl[:, 0]   # angular velocity output
current_cl = x_cl[:,1]  # current output
torque_cl = x_cl[:,1]*Kt   # torque velocity output
 
# ── Plots ───────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(10, 10))
fig.suptitle("DC Motor Simulation", fontsize=14)
 
# Open-loop
axes[0].plot(t, yout, color="steelblue")
axes[0].set_title("Open-Loop Response (unit step voltage)")
axes[0].set_xlabel("Time (s)")
axes[0].set_ylabel("ω (rad/s)")
axes[0].grid(True)
 
# Closed-loop angular velocity
axes[1].plot(t_cl, torque_cl, color="darkorange",
             label=f"K = {K.flatten().tolist()}")
axes[1].plot(t_cl, np.ones_like(t_cl)*Torque_Setpoint, color="blue",
             label="torque setpoint")
axes[1].set_title(f"Closed-Loop Response — PD Controller (u = −K·(x - Setpoint), f = {f_control/1000:.0f} kHz)")
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Torque (N/m)")
axes[1].legend()
axes[1].grid(True)
 
plt.tight_layout()
plt.show()
 

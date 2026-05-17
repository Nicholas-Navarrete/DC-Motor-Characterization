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

# Defining the SS system
A = np.array([[-b/J,Kt/J],[-Kt/L,-R/L]])
B = np.array([[0],[1/L]])
C = np.array([[1,0]])
D = np.array([[0]])

sys = StateSpace(A,B,C,D)

# Simulating the system
t = np.linspace(0,10,1000)
u = np.ones(t.shape)
x0 = np.array([0,0])

_,yout,xout = lsim(sys,u,t,x0)

# Plot the system
plt.plot(t,yout)
plt.show()



import numpy as np
from math import *
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from plotWindow import plotWindow
import matplotlib.pyplot as plt

np.set_printoptions(linewidth=150)
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

UNI_STATE = 5
LOG_WIDTH = 1 + UNI_STATE

data = np.reshape(np.fromfile("/tmp/unicycle_sim.bin", dtype=np.float64), (-1, LOG_WIDTH)).T

# check for NAns
if (np.any(np.isnan(data))):
    print("Uh Oh.... We've got NaNs")

t = data[0,:]
uni_state = data[1:6,:]

pw = plotWindow()

f = plt.figure(dpi=150)
plt.plot()
plt.plot(uni_state[0,:], uni_state[1,:], label="True $x$")
# plt.plot(xc[1,:], xc[0,:], label=r"UAV $x_c$")
# plt.plot(x_veh[1,:], x_veh[0,:], label=r"Veh $x$")
plt.legend()
pw.addPlot("2D Position", f)

state_label = [r'$x$', r'$y$', r'$\theta$']
f = plt.figure(dpi=150)
plt.plot()
for i in range(3):
    plt.suptitle("State")
    plt.subplot(3, 1, i+1)
    plt.plot(t, uni_state[i,:], label="x")
    # plt.plot(t, xc[i,:], label=r"$x_c$")
    plt.ylabel(state_label[i])
    if i == 0:
        plt.legend()
pw.addPlot("State", f)

vel_label = [r'$v$', r'$\omega$']
f = plt.figure(dpi=150)
plt.plot()
for i in range(2):
    plt.suptitle("Velocities")
    plt.subplot(2, 1, i+1)
    plt.plot(t, uni_state[3 + i,:], label="x")
    # plt.plot(t, xc[i,:], label=r"$x_c$")
    plt.ylabel(state_label[i])
    if i == 0:
        plt.legend()
pw.addPlot("Velocities", f)

pw.show()



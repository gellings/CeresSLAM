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

data = np.fromfile("/tmp/online_1d_slam.bin", dtype=np.float64)
NUM_LANDMARKS = int(data[0])
# LANDMARKS = NUM_LANDMARKS
# landmarks = np.reshape(data[1 : 1 + LANDMARKS], (1, NUM_LANDMARKS)).T

UNI_STATE = 2
LOG_WIDTH = 1 + 2 * UNI_STATE + 2 * NUM_LANDMARKS

data = np.reshape(data[1:], (-1, LOG_WIDTH)).T

# check for NAns
if (np.any(np.isnan(data))):
    print("Uh Oh.... We've got NaNs")

t = data[0,:]
x = data[1:3,:]
xhat = data[3:5, :]
lms = data[5 : 5 + NUM_LANDMARKS, :]
lms_hat = data[5 + NUM_LANDMARKS : 5 + 2 * NUM_LANDMARKS, :]

print("x shape: ", x.shape)
print("xhat shape: ", xhat.shape)
print("lms shape: ", lms.shape)
print("lms_hat shape: ", lms_hat.shape)

pw = plotWindow()

f = plt.figure(dpi=150)
plt.plot()
plt.plot(x[0,:], np.zeros(t.shape), label="True $x$")
plt.plot(lms[:, 0], np.zeros((NUM_LANDMARKS)), "rs", label="Landmarks")
plt.legend()
pw.addPlot("Position", f)

state_label = [r'$x$', r'$v$']
f = plt.figure(dpi=150)
plt.plot()
for i in range(2):
    plt.suptitle("State")
    plt.subplot(2, 1, i+1)
    plt.plot(t, x[i,:], label="x")
    plt.plot(t, xhat[i,:], label="$\hat{x}$")
    plt.ylabel(state_label[i])
    if i == 0:
        plt.legend()
pw.addPlot("State", f)

# state_label = [r'$x$', r'$v$']
f = plt.figure(dpi=150)
plt.plot()
for i in range(NUM_LANDMARKS):
    plt.suptitle("Landmarks")
    plt.subplot(NUM_LANDMARKS, 1, i+1)
    plt.plot(t, lms[i,:], label="x")
    plt.plot(t, lms_hat[i,:], label="$\hat{x}$")
    # plt.ylabel(state_label[i])
    if i == 0:
        plt.legend()
pw.addPlot("Landmarks", f)

pw.show()



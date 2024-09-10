import numpy as np
import matplotlib.pyplot as plt

LA, LB, LC = 1.0, 0.8, 0.7
x_max = LA+LB-LC/2.0
y_max = np.sqrt((LA+LB)**2 - LC**2/4.0)

def draw_configuration(fig, data):
  load_xy = data[7:9]
  ang = data[[3, 14]]
  r1 = LB * np.r_[np.cos(ang[0]), np.sin(ang[0])] + np.r_[-LC/2.0, 0.0]
  r2 = LB * np.r_[np.cos(ang[1]), np.sin(ang[1])] + np.r_[LC/2.0, 0.0]
  points = np.array([[-LC/2.0, 0.0], r1, load_xy, r2, [LC/2.0, 0.0]]).transpose()
  fig.plot(points[0], points[1])

# data = np.genfromtxt('out.csv', delimiter=',', dtype=float)
data = np.fromfile('out_mode_switching.bin', dtype='f4', count=-1).reshape([-1,33])

plt.subplots()
# plt.plot(data[:,0], np.linalg.norm(data[:,-4:-2], axis=1)) # mag torque
# plt.plot(data[:,0], np.linalg.norm(data[:,-2:], axis=1)) # mag accel

# plt.plot(data[:,0], np.linalg.norm(data[:,21:23], axis=1)) # mag speed
ws_bg = plt.imread('whole_workspace.png')
plt.imshow(ws_bg, extent=[-x_max, x_max, -y_max, y_max])
plt.plot(data[:,7], data[:,8])
draw_configuration(plt, data[0])
draw_configuration(plt, data[2600])
draw_configuration(plt, data[7999])
plt.show()
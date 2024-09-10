import numpy as np
import matplotlib.pyplot as plt

LA, LB, LC = 1.0, 0.8, 0.7
x_max = LA+LB-LC/2.0
y_max = np.sqrt((LA+LB)**2 - LC**2/4.0)

def draw_configuration(axis, data, color):
  load_xy = data[7:9]
  ang = data[[3, 14]]
  r1 = LB * np.r_[np.cos(ang[0]), np.sin(ang[0])] + np.r_[-LC/2.0, 0.0]
  r2 = LB * np.r_[np.cos(ang[1]), np.sin(ang[1])] + np.r_[LC/2.0, 0.0]
  points = np.array([[-LC/2.0, 0.0], r1, load_xy, r2, [LC/2.0, 0.0]]).transpose()
  axis.plot(points[0], points[1], color=color)

data = np.fromfile('out_2dof_pd_vlim.bin', dtype='f4', count=-1).reshape([-1,33])

fig_cs, ax_cs = plt.subplots()
fig_cs.set_size_inches([5,3])
cs_bg = plt.imread('positive.png')
ax_cs.imshow(cs_bg, extent=[-np.pi, np.pi, -np.pi, np.pi])
ax_cs.imshow(cs_bg, extent=[-3*np.pi, -np.pi, -np.pi, np.pi])
ax_cs.imshow(cs_bg, extent=[-3*np.pi, -np.pi, -3*np.pi, -np.pi])
ax_cs.imshow(cs_bg, extent=[-np.pi, np.pi, -3*np.pi, -np.pi])
ax_cs.plot(data[:,3], data[:, 14], 'yellow')
# ax_cs.set_ylim([0,np.pi])
ax_cs.set_xlabel(r'$\theta_1$')
ax_cs.set_ylabel(r'$\theta_2$')

ws_bg = plt.imread('whole_workspace.png')
fig_ws, ax_ws = plt.subplots()
fig_ws.set_size_inches([5,6])
ax_ws.imshow(ws_bg, extent=[-x_max, x_max, -y_max, y_max])
ax_ws.set_xlabel('x')
ax_ws.set_ylabel('y')
ax_ws.plot(data[:, 7], data[:, 8], 'brown')

plt.show()
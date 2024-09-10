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

# data = np.genfromtxt('out.csv', delimiter=',', dtype=float)
data = np.fromfile('out_mode_switching.bin', dtype='f4', count=-1).reshape([-1,33])

fig_cs, ax_cs = plt.subplots()
fig_cs.set_size_inches([5,3])
cs_bg = plt.imread('positive.png')
ax_cs.imshow(cs_bg, extent=[-np.pi, np.pi, -np.pi, np.pi])
ax_cs.plot(data[0:2199,3], data[0:2199, 14], '0.8')
ax_cs.plot(data[2200:3450,3], data[2200:3450, 14], 'brown')
ax_cs.set_ylim([0,np.pi])
ax_cs.set_xlabel(r'$\theta_1$')
ax_cs.set_ylabel(r'$\theta_2$')
fig_cs.savefig('wm_switching_ex1_3_cs.png', format='png', transparent=True, dpi=200)

ws_bg = plt.imread('whole_workspace.png')
fig_ws, ax_ws = plt.subplots()
fig_ws.set_size_inches([5,6])
ax_ws.imshow(ws_bg, extent=[-x_max, x_max, -y_max, y_max])
ax_ws.set_xlabel('x')
ax_ws.set_ylabel('y')
# plt.plot(data[:,7], data[:,8])
# ax.plot(data[0, 7], data[0, 8], 'bo')
# ax.plot(data[7999, 7], data[7999, 8], 'bo')
# draw_configuration(ax_ws, data[0], 'cyan')
ax_ws.plot(data[0:2199, 7], data[0:2199, 8], '0.8')
# fig.savefig('ex1_1.png', dpi=200, format='png', transparent=True)
draw_configuration(ax_ws, data[2200], '0.9')
draw_configuration(ax_ws, data[2380], '0.75')
draw_configuration(ax_ws, data[2570], '0.625')
draw_configuration(ax_ws, data[2750], '0.5')
draw_configuration(ax_ws, data[3000], '0.375')
draw_configuration(ax_ws, data[3217], '0.25')
draw_configuration(ax_ws, data[3450], '0.125')
ax_ws.plot(data[2200:3450, 7], data[2200:3450, 8], 'brown')
ax_ws.set_xlim([-0.592, 0.437])
ax_ws.set_ylim([-0.868, 0.873])
# draw_configuration(plt, data[7999], 'blue')
fig_ws.savefig('wm_switching_ex1_3_ws.png', format='png', transparent=True, dpi=200)
plt.show()
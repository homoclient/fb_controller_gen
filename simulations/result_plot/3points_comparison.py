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

data_labels = ['PD', '2DoF PD', '2DoF PD with velocity limit']

data_pd = np.fromfile('out_basic_pd.bin', dtype='f4', count=-1).reshape([-1,33])
data_2dof_pd = np.fromfile('out_2dof_pd.bin', dtype='f4', count=-1).reshape([-1,33])
data_2dof_pd_vlim = np.fromfile('out_2dof_pd_vlim.bin', dtype='f4', count=-1).reshape([-1,33])

fig_cs, ax_cs = plt.subplots()
fig_cs.set_size_inches([5,5])
cs_bg = plt.imread('positive.png')
ax_cs.imshow(cs_bg, extent=[-np.pi, np.pi, -np.pi, np.pi])
ax_cs.imshow(cs_bg, extent=[-3*np.pi, -np.pi, -np.pi, np.pi])
ax_cs.imshow(cs_bg, extent=[-3*np.pi, -np.pi, -3*np.pi, -np.pi])
ax_cs.imshow(cs_bg, extent=[-np.pi, np.pi, -3*np.pi, -np.pi])
ax_cs.plot(data_pd[:,3], data_pd[:, 14], 'green', label=data_labels[0])
ax_cs.plot(data_2dof_pd[:,3], data_2dof_pd[:, 14], 'gold', label=data_labels[1])
ax_cs.plot(data_2dof_pd_vlim[:,3], data_2dof_pd_vlim[:, 14], 'brown', label=data_labels[2])
ax_cs.set_xlim([-2*np.pi, np.pi])
ax_cs.set_ylim([-2*np.pi, np.pi])
ax_cs.set_xlabel(r'$\theta_1$')
ax_cs.set_ylabel(r'$\theta_2$')
ax_cs.legend(loc='upper left')

ws_bg = plt.imread('whole_workspace.png')
ws_mode6 = plt.imread('6_colored.png')
fig_ws, ax_ws = plt.subplots()
fig_ws.set_size_inches([5,6])
ax_ws.imshow(ws_bg, extent=[-x_max, x_max, -y_max, y_max])
ax_ws.imshow(ws_mode6, extent=[-x_max, x_max, -y_max, y_max])
ax_ws.set_xlabel('x')
ax_ws.set_ylabel('y')
ax_ws.plot(data_pd[:, 7], data_pd[:, 8], 'green', label=data_labels[0])
ax_ws.plot(data_2dof_pd[:, 7], data_2dof_pd[:, 8], 'gold', label=data_labels[1])
ax_ws.plot(data_2dof_pd_vlim[:, 7], data_2dof_pd_vlim[:, 8], 'brown', label=data_labels[2])
# ax_ws.legend()
# fig_ws.savefig('ex2_comparison_ws.png', format='png', transparent=True, dpi=200)

fig_torque, axs_torque = plt.subplots(3,1, sharex=True)
fig_torque.set_size_inches([6.0, 4.0])
axs_torque[0].plot(data_pd[:,0], data_pd[:,29], c='green', ls='dotted')
axs_torque[0].plot(data_pd[:,0], data_pd[:,30], c='green', ls='dashed')
axs_torque[0].grid(True)


axs_torque[1].plot(data_2dof_pd[:,0], data_2dof_pd[:,29], c='gold', ls='dotted')
axs_torque[1].plot(data_2dof_pd[:,0], data_2dof_pd[:,30], c='gold', ls='dashed')
axs_torque[1].grid(True)
axs_torque[1].set_ylabel('Torque (N m)')

axs_torque[2].plot(data_2dof_pd_vlim[:,0], data_2dof_pd_vlim[:,29], c='brown', ls='dotted')
axs_torque[2].plot(data_2dof_pd_vlim[:,0], data_2dof_pd_vlim[:,30], c='brown', ls='dashed')
axs_torque[2].grid(True)
axs_torque[2].set_xlabel('Time (s)')

fig_v, ax_v = plt.subplots()
fig_v.set_size_inches([6.0, 4.0])
ax_v.plot(data_pd[:,0], np.linalg.norm(data_pd[:,21:23], axis=1), c='green', label=data_labels[0])
ax_v.plot(data_2dof_pd[:,0], np.linalg.norm(data_2dof_pd[:,21:23], axis=1), c='gold', label=data_labels[1])
ax_v.plot(data_2dof_pd_vlim[:,0], np.linalg.norm(data_2dof_pd_vlim[:,21:23], axis=1), c='brown', label=data_labels[2])
ax_v.set_ylabel('Load Velocity (m/s)')
ax_v.set_xlabel('Time (s)')
ax_v.legend(loc='upper right')
ax_v.grid(True)

fig_cs.savefig('ex2_comparison_cs.png', format='png', transparent=True, dpi=200)
fig_ws.savefig('ex2_comparison_ws.png', format='png', transparent=True, dpi=200)
fig_torque.savefig('ex2_comparison_torque.png', format='png', transparent=True, dpi=200)
fig_v.savefig('ex2_comparison_v.png', format='png', transparent=True, dpi=200)
# plt.show()
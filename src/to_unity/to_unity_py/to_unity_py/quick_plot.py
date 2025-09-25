import matplotlib.pyplot as plt
import numpy as np

import torch


loaded_pos = torch.load('pos_data.pt')
fig = plt.figure()
title = ['d_x', 'd_y', 'd_z', 'd_rx', 'd_ry', 'd_rz']
for i in range(6): 
    ax = fig.add_subplot(3, 3, i+1)
    # plt.subplot(3, 3, i+1)
    ax.plot(loaded_pos[:, 0], loaded_pos[:, i+1])
    ax.set_title(title[i])

ax_x = fig.add_subplot(3, 3, 7)
ax_x.plot(loaded_pos[:, 0], loaded_pos[:, 7])
ax_x.plot(loaded_pos[:, 0], loaded_pos[:, 8])
ax_x.set_title("x")

ax_y = fig.add_subplot(3, 3, 8)
ax_y.plot(loaded_pos[:, 0], loaded_pos[:, 9])
ax_y.plot(loaded_pos[:, 0], loaded_pos[:, 10])
ax_y.set_title("y")

ax_z = fig.add_subplot(3, 3, 9)
ax_z.plot(loaded_pos[:, 0], loaded_pos[:, 11])
ax_z.plot(loaded_pos[:, 0], loaded_pos[:, 12])
ax_z.set_title("z")

loaded_vel = torch.load('vel_data.pt')
fig2 = plt.figure() 
for i in range(7): 
    ax = fig2.add_subplot(7, 1, i+1)
    ax.plot(loaded_pos[:len(loaded_vel[:, i]), 0], loaded_vel[:, i])

loaded_u = torch.load('u_data.pt')
fig3 = plt.figure() 
for i in range(7): 
    ax = fig3.add_subplot(7, 1, i+1)
    ax.plot(loaded_pos[:len(loaded_u[:, i]), 0], loaded_u[:, i])
    ax.plot(loaded_pos[:len(loaded_u[:, i]), 0], loaded_u[:, 7])
    

fig4 = plt.figure()  
ax = fig4.add_subplot(1, 1, 1)
ax.plot(loaded_pos[:, 7], loaded_pos[:, 9])
ax.plot(loaded_pos[:, 8], loaded_pos[:, 10])

# fig5 = plt.figure()  
# for i in range(7): 
#     ax = fig5.add_subplot(7, 1, i+1)
#     ax.plot(loaded_pos[:len(loaded_u[:, i]), 0], loaded_vel[:len(loaded_u[:, i]), i]/loaded_u[:, i])


plt.show() 

# msg.positions = listnp.array([-43.2, -26.43, 4.19, 115.65, -22.72, 127.31, 46.15]) / 180 * np.pi 


# x = np.array([i * 0.0002 for i in range(0, 1000)]) 
# y = np.zeros(x.shape)
# for i, num in enumerate(x):
#     if (num < 0.02):
#         y[i] = 0.03
#     elif (num > 0.1):
#         y[i] = 0.005
#     else:
#         y[i] = -(0.03 - 0.005) / (0.1 - 0.02) * num + 0.03625
# plt.plot(x, y)

# y2 = 0.0125 * np.tanh(-(x-0.03)*30) + 0.035 / 2.0
# plt.plot(x, y2)

# plt.show()
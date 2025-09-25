import matplotlib.pyplot as plt
import numpy as np

import torch


loaded_pos = torch.load('issac_pos.pt').cpu()
print(loaded_pos)
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

# loaded_vel = torch.load('vel_data.pt')
# fig2 = plt.figure() 
# for i in range(7): 
#     ax = fig2.add_subplot(7, 1, i+1)
#     ax.plot(loaded_pos[:len(loaded_vel[:, i]), 0], loaded_vel[:, i])

# loaded_u = torch.load('u_data.pt')
# fig3 = plt.figure() 
# for i in range(7): 
#     ax = fig3.add_subplot(7, 1, i+1)
#     ax.plot(loaded_pos[:len(loaded_u[:, i]), 0], loaded_u[:, i])
#     ax.plot(loaded_pos[:len(loaded_u[:, i]), 0], loaded_u[:, 7])
    

# fig4 = plt.figure()  
# ax = fig4.add_subplot(1, 1, 1)
# ax.plot(loaded_pos[:, 7], loaded_pos[:, 9])
# ax.plot(loaded_pos[:, 8], loaded_pos[:, 10])



plt.show() 

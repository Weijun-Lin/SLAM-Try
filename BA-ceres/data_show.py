'''
@author weijun-lin
@brief 优化三维点云可视化
@date 2021-12-22
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#定义坐标轴
fig = plt.figure()
ax1 = plt.axes(projection='3d')

# 读取数据
data_src = np.loadtxt("./3d_data_src.txt", delimiter=",")
data_noise = np.loadtxt("./3d_data_noise.txt", delimiter=",")
data_opt = np.loadtxt("./3d_data_opt.txt", delimiter=",")

# 显示数据
# ax1.scatter3D(data_src[..., 0], data_src[..., 1], data_src[..., 2], c='b')
l1 = ax1.scatter(data_noise[..., 0], data_noise[..., 1], data_noise[..., 2], c='g')
l2 = ax1.scatter(data_opt[..., 0], data_opt[..., 1], data_opt[..., 2], c='r', marker='x')
ax1.legend(handles=[l1, l2], labels=['Data With Noise', 'Optimized'])

plt.show()
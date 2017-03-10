
# 从txt文档获取speed，绘制稳定性曲线

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

data = np.loadtxt('lane.txt')
using = data[90000::10]

spd = using[:, 4]
num = using[:, 1].astype(int)
time = using[:, 0].astype(int)

fig = plt.figure()
ax = plt.subplot(111,projection='3d')

ax.scatter(spd, time, num)

# plt.xlim(2, 3)


plt.show()

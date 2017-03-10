
# 从txt文档获取data，绘制稳定性曲线

f = open('lane.txt','r')
lines = f.readlines()
f.close()

import matplotlib.pyplot as plt

fig = plt.figure(figsize=(8,4))
pic = fig.add_subplot(1, 1, 1)

lines = lines[14801:] # 精度可能有问题
for line in lines:
    line = line.split()

position = [line.split()[2] for line in lines]
time = [line.split()[0] for line in lines]

pic.plot(position,time,'b')


fig.show()

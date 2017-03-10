
# 绘制模型的特性曲线

import numpy as np
import matplotlib.pyplot as plt


# 绘图调整参数
col = ['b', 'g', 'r', 'k']
lam = 0.6
tao = [0.8,1,1.2,1.4]

# 权重系数
ay = 0.8
aq = 0.2

# 优化速度函数参数
v00=15
beta=1.5
deltas=8

# 线性化系数
def A(x):
    return v00*ay/deltas/(1+np.tanh(beta))/(np.cosh(x/deltas-beta))**2
    
def Af(x):
    return v00*aq/deltas/(1+np.tanh(beta))/(np.cosh(x/deltas-beta))**2

def almax(lam,tao,headway):
    return np.array([(3-3*lam*tao**2)/(lam*tao**3)]*dim)

def almin(lam,tao,headway):
    return 2*(A(ay*headway+aq*headway/2.0)+Af(ay*headway+aq*headway/2.0)-lam*tao)



#绘图精度
dim = 1000
headway = np.linspace(0, 30, dim)


fig = plt.figure(figsize=(6,8))
pic = fig.add_subplot(1, 1, 1)

for i in range(len(tao)):
    pic.plot(headway,almax(lam, tao[i], headway), col[i])
    pic.plot(headway,almin(lam, tao[i], headway), col[i]+'--')

fig.show()


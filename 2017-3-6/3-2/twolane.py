
from math import tanh

# 横轴Headway
hdw = 15

# 延时
tao = 100
# 反馈系数
lambda1 = 0.4
lambda2 = 0.4

# 时间步长
t = 0.01
# 循环次数
circle = 10000

# 敏感系数
a1 = 2.0
a2 = 2.0

# 车道长度
L = 1500
# 车数
carsum = 100

# 安全车距
xfc = 2
ay = 0.8
aq = 0.2

# 优化速度函数1
def OV1(ay, aq, headway, distf, hd = 2):
    return tanh(ay * headway + aq * distf - hd) + tanh(hd)
# 优化速度函数2
def OV2(ay, aq, headway, distf):
    delta = 8
    beta = 1.5
    v0 = 15
    dx = ay * headway + aq * distf
    return v0 * (1/(1+tanh(beta))) * (tanh(dx/delta-beta)) + tanh(beta)

# 车辆类
class Vehicle:
    # 初始化车辆信息
    def __init__(self, num, lanenum, pos, spd, acc, headway, distf):
        self.Num = num
        self.Lanenum = lanenum
        self.Pos = [float(pos)] * (tao+1)
        self.Spd = [float(spd)] * (tao+1)
        self.Acc = [float(acc)] * (tao+1)
        self.Headway = [float(headway)] * (tao+1)
        self.Distf = [float(distf)] * (tao+1)

# 初始化道路上车辆信息
def LaneGen(carsum, length, carheadway, lanenum, OV, distf):
    Car = []
    if lanenum == 0:
        for i in range(1, 1 + carsum):
            newCar = Vehicle(i, lanenum, length - i * carheadway, OV, 0, carheadway, distf)
            Car.append(newCar)
    else:
        for i in range(1, 1 + carsum):
            newCar = Vehicle(i + 100, lanenum, length - i * carheadway + carheadway/2, OV, 0, carheadway, distf)
            Car.append(newCar)
    return Car

# 干扰函数
def Interation(Car1, Car2, ay, aq, i):
    # 对车进行遍历
    for car in Car1:
        # 找车
        thecar = car
        precar = Car1[Car1.index(thecar)-1]
        # 找临道前车 可能有问题
        sortlist = [thecar] + Car2
        sortlist.sort(key=lambda x: x.Pos[tao], reverse=True) # 按位置排序
        neacar = sortlist[sortlist.index(thecar) - 1]
        neacar = Car2[Car2.index(neacar)]
        del sortlist

        # 加入扰动
        if i == 2000:
            if thecar.Num == 25:
                thecar.Pos[tao] += 0.5 * ((precar.Pos[tao] - thecar.Pos[tao] + L) % L)
                if thecar.Pos[tao] >= L:
                    thecar.Pos[tao] -= L

        thecar.Headway[tao] = (precar.Pos[tao] - thecar.Pos[tao] + L) % L
        thecar.Distf[tao] = neacar.Pos[tao] - thecar.Pos[tao]
        if thecar.Distf[tao] < -L/2:
            thecar.Distf[tao] += L

        # 优化速度
        OptV = OV2(ay, aq, thecar.Headway[tao], thecar.Distf[tao])

        # 龙格库塔法算速度
        if thecar.Lanenum == 0:
            k1 = t * (a1 * (OptV - thecar.Spd[tao-1]) + lambda1 * (thecar.Distf[tao] - thecar.Distf[1]))
            k2 = t * (a1 * (OptV - thecar.Spd[tao-1] - 0.5 * k1) + lambda1 * (thecar.Distf[tao] - thecar.Distf[1]))
            k3 = t * (a1 * (OptV - thecar.Spd[tao-1] - 0.5 * k2) + lambda1 * (thecar.Distf[tao] - thecar.Distf[1]))
            k4 = t * (a1 * (OptV - thecar.Spd[tao-1] - k3) + lambda1 * (thecar.Distf[tao] - thecar.Distf[1]))
            thecar.Spd[tao] = thecar.Spd[tao-1] + (1/6) * (k1 + 2 * k2 + 2 * k3 + k4)
        else:
            k1 = t * (a2 * (OptV - thecar.Spd[tao-1]) + lambda2 * (thecar.Distf[tao] - thecar.Distf[1]))
            k2 = t * (a2 * (OptV - thecar.Spd[tao-1] - 0.5 * k1) + lambda2 * (thecar.Distf[tao] - thecar.Distf[1]))
            k3 = t * (a2 * (OptV - thecar.Spd[tao-1] - 0.5 * k2) + lambda2 * (thecar.Distf[tao] - thecar.Distf[1]))
            k4 = t * (a2 * (OptV - thecar.Spd[tao-1] - k3) + lambda2 * (thecar.Distf[tao] - thecar.Distf[1]))
            thecar.Spd[tao] = thecar.Spd[tao-1] + (1/6) * (k1 + 2 * k2 + 2 * k3 + k4)
        # 加速度计算
        if thecar.Lanenum == 0:
            thecar.Acc[tao] = a1 * (OptV - thecar.Spd[tao]) + lambda1 * (thecar.Distf[tao] - thecar.Distf[1])
        else:
            thecar.Acc[tao] = a2 * (OptV - thecar.Spd[tao]) + lambda2 * (thecar.Distf[tao] - thecar.Distf[1])

        # 新位置计算
        thecar.Pos[0] = thecar.Pos[tao] + thecar.Spd[tao] * t + 0.5 * thecar.Acc[tao] * t * t
        if thecar.Pos[0] >= L:
            thecar.Pos[0] -= L

        # 输出文件到txt文件中 全部为tao


# 换道函数 未修改
def LaneChange(lane1, lane2, car, time):
    thecar = car

    if len(lane2.Car) == 0:
        car.Lanenum = (car.Lanenum + 1) % 2
        lane2.Car.append(car)
        lane1.Car.remove(car)
    else:
        # 找临道前车 可能有问题
        sortlist = [car] + lane2.Car
        sortlist.sort(key=lambda x: x.Pos[tao], reverse=True) # 按位置排序
        k = sortlist.index(car)
        neacar = sortlist[k - 1]
        neacar = lane2.Car[lane2.Car.index(neacar)]
        folcar = lane2.Car[(lane2.Car.index(neacar) + 1) % len(lane2.Car)]
        del sortlist

        if k == 0:
            distf = neacar.Pos[tao] - thecar.Pos[tao] + L
            distb = thecar.Pos[tao] - folcar.Pos[tao]
        elif k == len(lane2.Car):
            distf = neacar.Pos[tao] - thecar.Pos[tao]
            distb = thecar.Pos[tao] - folcar.Pos[tao] + L
        else:
            distf = neacar.Pos[tao] - thecar.Pos[tao]
            distb = thecar.Pos[tao] - folcar.Pos[tao]


        if (thecar.Headway[tao] < 2 * xfc) and (distf > thecar.Headway[tao]) and (distb > xfc):
            thecar.Lanenum = (thecar.Lanenum + 1) % 2
            thecar.Distf[tao] = thecar.Headway[tao]
            thecar.Headway[tao] = distf
            lane2.Car.insert(lane2.Car.index(folcar), thecar)
            lane1.Car.remove(thecar)


# 道路更新函数
def Update(Car, time):
    for car in Car:
        newpos = car.Pos[0]
        car.Acc = car.Acc[1:] + [0]
        car.Distf = car.Distf[1:] + [0]
        car.Headway = car.Headway[1:] + [0]
        car.Spd = car.Spd[1:] + [0]
        car.Pos = car.Pos[1:] + [newpos]


ov = OV2(ay, aq, hdw, hdw/2)
Car1 = LaneGen(carsum, L, hdw, 0, ov, hdw/2)
Car2 = LaneGen(carsum, L, hdw, 1, ov, hdw/2)


f1 = open("lane1.txt",'w')
f2 = open("lane2.txt",'w')

for i in range(circle):
    Interation(Car1, Car2, ay, aq, i)
    Interation(Car2, Car1, ay, aq, i)

    if i % 100 == 0:
        print(i)
        for car in Car1:
            f1.write(str(car.Num) + ' ' + str(car.Headway[tao])+ ' ' + str(car.Spd[tao]) + '\n')
            # lane1.LaneChange(i, lane1, lane2, car)
        for car in Car2:
            f2.write(str(car.Num) + ' ' + str(car.Headway[tao])+ ' ' + str(car.Spd[tao]) + '\n')
            # lane2.LaneChange(i, lane2, lane1, car)

    Update(Car1,i)
    Update(Car2,i)


f1.close()
f2.close()




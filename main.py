import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import MultipleLocator
from pylab import *
import random



k = 0.1  # 前视距离系数
Lfc = 0.03  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.05  # 时间间隔，单位：s
L = 0.5  # 车辆轴距，单位：m
wmax = 2.0 #4.4 舵轮最大角速度
target_speed = 0.1  # 移动速度[m/s]
watch = True#True False
filename = 'Lfc0_5_noise_neg_yonly_1hz.mp4'
e = 5 #每秒偏移误差幅值  cm/s
a = 10000/e
Tn=int(1/dt)

fig = plt.figure()
trackpath = fig.add_subplot(1,1,1)
# noise = fig.add_subplot(2,1,2)


ln, = trackpath.plot([], [], '-b',linewidth=2.5)
tar, = trackpath.plot([], [], 'go')
# noise_t, = noise.plot([], [], '-b')
# lyaw, = plt.plot([], [], 'go')
x = []
y = []
target = []
cx = []
cy = []
cyaw = []
noise_datax = []
noise_datay = []



for i in range(50):
    cx.append(0.01*i)
    cy.append(0)
for i in range(50):
    cx.append(0.49)
    cy.append(0.01 + 0.01*i)
for i in range(50):
    cx.append(0.48-0.01*i)
    cy.append(0.5)

class VehicleState:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class contralVal:
    def __init__(self, v = [0.0,0.0,0.0], th=[0.0,0.0,0.0]):
        self.v = v
        self.th = th
    def setv(self, v):
        self.v = v
    def setth(self, th):
        self.th = th

def update(state, aa, ctr):
    global gr
    a = 0.244
    state.v += aa*dt
    sumv = [0.0, 0.0]
    glwheelth = [i+state.yaw for i in ctr.th]#每个轮子相对于世界坐标系的角
    glwheelv = [[0.0, 0.0] for i in range(3)]#每个轮子速度的x，y分量
    for i in range(3):
        sumv[0] += cos(glwheelth[i])*ctr.v[i]
        sumv[1] += sin(glwheelth[i])*ctr.v[i]
        glwheelv[i][0] = cos(glwheelth[i])*ctr.v[i]
        glwheelv[i][1] = sin(glwheelth[i])*ctr.v[i]
    testv = [state.v*cos(state.yaw), state.v*sin(state.yaw)]#理论速度
    testth = state.v/gr

    state.x += sumv[0] * dt
    state.y += sumv[1] * dt
    #计算角速度
    locth = [0, math.pi*2/3, -math.pi*2/3]#中心指向每个轮子的角度
    glth = [i + state.yaw for i in locth]#相对世界坐标系中心指向每个轮子的角度
    vetth = [[cos(glth[i]), sin(glth[i])] for i in range(3)]#相对世界坐标系中心指向每个轮子的单位方向向量
    radialCompenent = [0.0, 0.0, 0.0]
    for i in range(3):
        radialCompenent[i] = vetth[i][0]*glwheelv[i][1]-vetth[i][1]*glwheelv[i][0]
    w = sum(radialCompenent)/a
    state.yaw += w*dt
    return state

def calc_target_index(state, cx, cy):
    # 搜索最临近的路点
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = d[ind]

    Lf = k * state.v + Lfc

    while L < Lf and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


def PControl(target, current):
    a = Kp * (target - current)
    return a

def getCircle(state, tx, ty):
    global gr
    k1 = tan(state.yaw + math.pi/2)
    tth = math.atan2(ty-state.y, tx - state.x)
    k2 = tan(tth + math.pi/2)
    x1 = state.x
    y1 = state.y
    xm = (x1+tx)/2
    ym = (y1+ty)/2
    a = np.array([[-k1, 1], [-k2, 1]])
    b = np.array([y1-k1*x1, ym-k2*xm]).T
    c = np.dot(np.linalg.inv(a),b)
    cirx,ciry = c[0], c[1]
    r = math.sqrt(pow(cirx-x1,2)+pow(ciry-y1,2))
    gr = r
    return cirx, ciry, r

def pure_pursuit_control(state, cx, cy, pind):
    a = 0.244
    L = 1.5 * a
    D = sqrt(3) / 2 * a
    ind = calc_target_index(state, cx, cy)

    if ind < pind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    cirx,ciry,r = getCircle(state, tx, ty)#圆心的x，y与半径
    wheel = [[0.0, 0.0] for i in range(3)]#轮子世界坐标系坐标
    wheellocal = [[a, 0], [-a/2, D], [-a/2, -D]];#轮子自身坐标系坐标
    locwheelth = [0.0, 0.0, 0.0]#每个轮子相对于车子主轴（前进方向）的夹角
    wheelr = [0.0, 0.0, 0.0]#每个轮子的转弯半径
    wheelv = [0.0, 0.0, 0.0]#每个轮子的转弯速度
    for i in range(3):
        #求轮子在世界坐标系的坐标
        temp1 = np.array([state.x, state.y])
        temp2 = np.array(wheellocal[i])
        temp1 = temp1.T
        temp2 = temp2.T
        spth = -state.yaw
        spinTrans = np.array([[cos(spth), sin(spth)],[-sin(spth), cos(spth)]])
        temp3 = np.dot(spinTrans,temp2) + temp1
        wheel[i] = [temp3[0], temp3[1]]
        #求每个轮子应该转的角度
        locwheelth[i] = math.atan2(wheel[i][1]-ciry, wheel[i][0]-cirx) - math.atan2(state.y-ciry, state.x-cirx)
        # 求每个轮子的转弯半径
        wheelr[i] = sqrt(pow(wheel[i][1]-ciry,2)+pow(wheel[i][0]-cirx,2))
        #求每个轮子应有的速度
        wheelv[i] = wheelr[i]/r*state.v
    return contralVal(wheelv, locwheelth), ind

def init():
    # ax.set_xlim(0, 1.1)
    # ax.set_ylim(-3.14, 3.14)


    trackpath.axis('equal')
    trackpath.set_ylim(-0.2, 0.6)
    trackpath.set_xlim(0, 0.6)
    # noise.set_ylim(-10, 10)
    # noise.set_xlim(0, 15)
    trackpath.grid(True)
    # noise.grid(True)

    return ln,

tt = []
def update_points(num):
    '''
    更新数据点
    '''

    trackpath.figure.canvas.draw()
    # noise.figure.canvas.draw()
    ln.set_data(x[:num], y[:num])
    tar.set_data(cx[target[num]], cy[target[num]])
    # temp = [100*i for i in noise_datay[:num]]
    # t=int(num*dt)
    # tt = [i for i in range(t)]
    # noise_t.set_data(tt, temp[:t])
    # lyaw.set_data(num*0.001,cyaw[num])
    # ln.set_data([10,11],[0,1] )

    return ln,tar,#noise_t,

def main():
     #  设置目标路点
    global x,y
    T = 100.0  # 最大模拟时间

    # 设置车辆的出事状态
    state = VehicleState(x=0.0, y=0.0 , yaw=0.52, v=0.0)
    ctr = contralVal()

    lastIndex = len(cx) - 1
    time = 0.0

    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)
    i = 1
    while time <= T and target_ind < lastIndex:
        ai = PControl(target_speed, state.v)
        ctr, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        state = update(state, ai, ctr)
        # if i<Tn:
        #     i += 1
        # else:
        #     i = 1
        #     noise_datax.append(random.randint(-100,100)/a)
        #     noise_datay.append(random.randint(-200,0)/a)
        #     state.y += noise_datay[-1]
        #     # state.x += noise_datax[-1]

        time = time + dt
        # tt.append(time)
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        cyaw.append(state.yaw)
        target.append(target_ind)


    trackpath.plot(cx, cy, ".r")

    # plt.grid(True)
    ani = animation.FuncAnimation(fig, update_points, frames=np.arange(1, 450), init_func=init, interval=50, blit=True)
    if watch:
         plt.show()
    else:
        try:
            ani.save(filename,writer='ffmpeg')
        except:
            print('mp4 done')




if __name__ == '__main__':
    main()

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import MultipleLocator
from pylab import *



k = 0.1  # 前视距离系数
Lfc = 0.03  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.05  # 时间间隔，单位：s
L = 0.5  # 车辆轴距，单位：m
wmax = 2.0 #4.4 舵轮最大角速度
target_speed = 0.1  # 移动速度[m/s]
watch = False#True False
filename = 'Lfc0_3.mp4'

fig, ax = plt.subplots()
ln, = plt.plot([], [], '-b',linewidth=2.5)
tar, = plt.plot([], [], 'go')
lyaw, = plt.plot([], [], 'go')
x = []
y = []
target = []
cx = []
cy = []
cyaw = []


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

def update(state, a, delta):
    dth=0
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    if(delta-state.yaw>3.14):
        dth = delta-state.yaw - 2*math.pi
    elif(delta-state.yaw<-3.14):
        dth = delta - state.yaw + 2 * math.pi
    if(math.fabs(dth)<wmax*dt):
        state.yaw = delta
    else:
        if dth > 0:
            state.yaw += dt*wmax
        else:
            state.yaw -= dt*wmax
    # state.yaw = delta
    state.v = state.v + a * dt

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

def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x)

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    # delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return alpha, ind

def init():
    ax.set_xlim(0, 1.1)
    ax.set_ylim(-3.14, 3.14)


    plt.axis('equal')
    return ln,

tt = []
def update_points(num):
    '''
    更新数据点
    '''
    tt.append(num*0.01)
    ax.figure.canvas.draw()
    ln.set_data(x[:num], y[:num])
    tar.set_data(cx[target[num]], cy[target[num]])
    # lyaw.set_data(num*0.001,cyaw[num])
    # ln.set_data([10,11],[0,1] )

    return ln,tar,

def main():
     #  设置目标路点




    T = 100.0  # 最大模拟时间

    # 设置车辆的出事状态
    state = VehicleState(x=0.0, y=0.0 , yaw=0.52, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0

    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    while time <= T and target_ind < lastIndex:
        ai = PControl(target_speed, state.v)
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        state = update(state, ai, di)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        cyaw.append(state.yaw)
        target.append(target_ind)


    plt.plot(cx, cy, ".r")

    # plt.grid(True)



    ani = animation.FuncAnimation(fig, update_points, frames=np.arange(1, 1000), init_func=init, interval=50, blit=True)
    if watch:
         plt.show()
    else:
        try:
            ani.save(filename,writer='ffmpeg')
        except:
            print('mp4 done')




if __name__ == '__main__':
    main()

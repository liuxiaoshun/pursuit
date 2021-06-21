import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import MultipleLocator


k = 0.1  # 前视距离系数
Lfc = 0.1  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.1  # 时间间隔，单位：s
L = 0.5  # 车辆轴距，单位：m

fig, ax = plt.subplots()
ln, = plt.plot([], [], '-b')
tar, = plt.plot([], [], 'go')
x = []
y = []
target = []
cx = []
cy = []
for i in range(100):
    cx.append(0.01*i)
    cy.append(0)
for i in range(10):
    cx.append(0.99)
    cy.append(0.01 + 0.01*i)
for i in range(100):
    cx.append(0.98-0.01*i)
    cy.append(0.1)

class VehicleState:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state

def calc_target_index(state, cx, cy):
    # 搜索最临近的路点
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * state.v + Lfc

    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
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

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def init():
    ax.set_xlim(0, 1.1)
    ax.set_ylim(-0.05, 0.15)


    plt.axis('equal')
    return ln,


def update_points(num):
    '''
    更新数据点
    '''
    ax.figure.canvas.draw()
    ln.set_data(x[:num], y[:num])
    tar.set_data(cx[target[num]], cy[target[num]])
    # ln.set_data([10,11],[0,1] )

    return ln,tar,

def main():
     #  设置目标路点


    target_speed = 0.1  # [m/s]

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
        target.append(target_ind)


    plt.plot(cx, cy, ".r")

    # plt.grid(True)



    ani = animation.FuncAnimation(fig, update_points, frames=np.arange(1, 1000), init_func=init, interval=100, blit=True)
    plt.show()



if __name__ == '__main__':
    main()

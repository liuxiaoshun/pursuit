import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import MultipleLocator
from pylab import *

colors=['blue', 'red', 'black', 'blue', 'purple']
subplot_list = [1,3,2,2,6]
leng = len(subplot_list)
ax, bars = list(), list()

def barlistdef(n): 
    return [1/float(n*k) for k in range(1,6)]

fig = plt.figure()
n = 20  # Number of frames
x = range(1,6)
for i in range(leng):
    if leng == 1:
        axis = fig.add_subplot(leng, 1, (i+1))
    elif leng > 1:
        if leng%2 == 0:
            axis = fig.add_subplot(leng/2, 2, (i+1))
        else:
            axis = fig.add_subplot(int(leng/2+0.5), 2, (i+1))
    ax.append(axis)
    barcollection = ax[i].bar(x,barlistdef(1), color=colors[i])
    bars.append(barcollection)

def animate(i):
    y = barlistdef(i+1)
    for j in range(len(bars)):
        for k, bar in enumerate(bars[j]):
            bar.set_height(y[k])
    return barcollection

ani = animation.FuncAnimation(fig, animate, interval=75, frames=n, blit=False, repeat=True)
plt.show()
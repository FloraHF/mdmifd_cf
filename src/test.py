import matplotlib.pyplot as plt 
import numpy as np
from math import pi, sin, cos

x0 = -2.2
y0 = -4.
R = 3.9

tht = np.linspace(0, 2*pi, 50)

x = [x0+R*sin(t) for t in tht]
y = [y0+R*cos(t) for t in tht]

plt.plot(x, y)
plt.axis('equal')
plt.xlim(-2, 2)
plt.ylim(-1.6, 2)
plt.grid()


plt.show()
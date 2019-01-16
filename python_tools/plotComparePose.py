import matplotlib.pyplot as plt
import matplotlib.pyplot as plt2
import matplotlib.pyplot as plt3
import numpy as np
import numpy as np1
import numpy as np2
import numpy as np3
import numpy as np4
import matplotlib as mpl
import matplotlib as mpl2
mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['font.sans-serif'] = 'NSimSun,Times New Roman'

(t1, x1, y1, z1 ) = np1.loadtxt('../build/oldpose/footsingle.txt', unpack=True)
(t2, x2, y2, z2 ) = np1.loadtxt('../build/newpose/footsingle.txt', unpack=True)

fig1 = plt.figure()
fig1.suptitle('compare pose: plane from singe frame',fontsize=15,color='r')
ax1 = fig1.add_subplot(131)
ax1.plot(t1, x1, '-',color='black',label='old')
ax1.plot(t2, x2, '-',color='red',label='new')
ax1.legend()
plt.xlabel("frame ID")
plt.ylabel("foot x")
ax1.set_title('x',fontsize=12,color='b')
ax2 = fig1.add_subplot(132)
ax2.plot(t1, y1, '-',color='black',label='old')
ax2.plot(t2, y2, '-',color='red',label='new')
ax2.legend()
plt.xlabel("frame ID")
plt.ylabel("foot y")
ax2.set_title('y',fontsize=12,color='b')
ax3 = fig1.add_subplot(133)
ax3.plot(t1, z1, '-',color='black',label='old')
ax3.plot(t2, z2, '-',color='red',label='new')
ax3.legend()
plt.xlabel("frame ID")
plt.ylabel("foot z")
ax3.set_title('z',fontsize=12,color='b')

 
plt.show()


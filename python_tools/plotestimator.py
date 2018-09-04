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

(t3, x3, y3, z3) = np3.loadtxt('/home/abner/UFO/ecl/EKF/build/position_estimator.txt', unpack=True)

(t4, roll, pitch, yaw) = np.loadtxt('/home/abner/UFO/ecl/EKF/build/euler_estimator.txt', unpack=True)



fig1 = plt.figure()

bx4 = fig1.add_subplot(161)
bx4.plot(t3, x3, '-',color='black')
bx4.set_title('x',fontsize=12,color='b')
bx5 = fig1.add_subplot(162)
bx5.plot(t3, y3, '-',color='black')
bx5.set_title('y',fontsize=12,color='b')
bx6 = fig1.add_subplot(163)
bx6.plot(t3, z3, '-',color='black')
bx6.set_title('z',fontsize=12,color='b')

bx4 = fig1.add_subplot(164)
bx4.plot(t3, roll , '-',color='black')
bx4.set_title('roll',fontsize=12,color='b')
bx5 = fig1.add_subplot(165)
bx5.plot(t3, pitch, '-',color='black')
bx5.set_title('pitch',fontsize=12,color='b')
bx6 = fig1.add_subplot(166)
bx6.plot(t3, yaw, '-',color='black')
bx6.set_title('yaw',fontsize=12,color='b')

plt.show()


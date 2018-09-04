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

(t1, yaw1, pitch1, roll1 ) = np1.loadtxt('/home/abner/UFO/ecl/EKF/build/euler_estimator.txt', unpack=True)
#(t2, yaw2, pitch2, roll2 ) = np2.loadtxt('/home/abner/UFO/ecl/EKF/build/euler_data.txt', unpack=True)
(t3, x3, y3, z3) = np3.loadtxt('/home/abner/UFO/ecl/EKF/build/position_estimator.txt', unpack=True)
(t4, x4, y4, z4, yaw2, pitch2, roll2) = np.loadtxt('/home/abner/UFO/ecl/EKF/build/data/pose_real.txt', unpack=True)
(t, test_x, test_y) = np4.loadtxt('/home/abner/UFO/ecl/EKF/build/data/gps_test.txt', unpack=True)


fig1 = plt.figure()
ax1 = fig1.add_subplot(161)
ax1.plot(t1, roll1, '-',color='black')
ax1.plot(t4, roll2, '-',color='red')
ax1.set_title('yaw',fontsize=12,color='b')
ax2 = fig1.add_subplot(162)
ax2.plot(t1, pitch1, '-',color='black')
ax2.plot(t4, pitch2, '-',color='red')
ax2.set_title('pitch',fontsize=12,color='b')
ax3 = fig1.add_subplot(163)
ax3.plot(t1, yaw1, '-',color='black')
ax3.plot(t4, yaw2, '-',color='red')
ax3.set_title('roll',fontsize=12,color='b')


bx4 = fig1.add_subplot(164)
bx4.plot(t3, x3, '-',color='black')
bx4.plot(t4, x4, '-',color='red')
bx4.set_title('x',fontsize=12,color='b')
bx5 = fig1.add_subplot(165)
bx5.plot(t3, y3, '-',color='black')
bx5.plot(t4, y4, '-',color='red')
bx5.set_title('y',fontsize=12,color='b')
bx6 = fig1.add_subplot(166)
bx6.plot(t3, z3, '-',color='black')
bx6.plot(t4, -z4, '-',color='red')
bx6.set_title('z',fontsize=12,color='b')
plt.show()


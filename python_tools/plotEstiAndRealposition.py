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


#(t3, x3, y3, z3, x32, y32, z32) = np3.loadtxt('C:/UFO/AirSim-master/HelloDrone/p_estimator.txt', unpack=True)
(t3, x3, y3, z3) = np3.loadtxt('/home/abner/UFO/ecl/EKF/build/position_estimator.txt', unpack=True)
(t4, x4, y4, z4, roll ,pitch ,yaw) = np.loadtxt('/home/abner/UFO/ecl/EKF/build/data/pose_real.txt', unpack=True)




fig2 = plt2.figure()
bx1 = fig2.add_subplot(131)
bx1.plot(t3, x3, '-',color='black')
bx1.plot(t4, x4, '-',color='red')
bx1.set_title('x',fontsize=12,color='b')
bx2 = fig2.add_subplot(132)
bx2.plot(t3, y3, '-',color='black')
bx2.plot(t4, y4, '-',color='red')
bx2.set_title('y',fontsize=12,color='b')
bx3 = fig2.add_subplot(133)
bx3.plot(t3, z3, '-',color='black')
bx3.plot(t4, z4, '-',color='red')
bx3.set_title('z',fontsize=12,color='b')
plt2.show()
 

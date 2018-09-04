import matplotlib.pyplot as plt2
import numpy as np4
import matplotlib as mpl
import matplotlib as mpl2
mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['font.sans-serif'] = 'NSimSun,Times New Roman'

(t, test_x, test_y) = np4.loadtxt('/home/abner/UFO/ecl/EKF/build/data/gps_test.txt', unpack=True)

 
fig2 = plt2.figure()
cx1 = fig2.add_subplot(131)
cx1.plot(t, test_x)
cx2 = fig2.add_subplot(132)
cx2.plot(t, test_y)
cx3 = fig2.add_subplot(133)
cx3.plot(test_x, test_y)
plt2.show()

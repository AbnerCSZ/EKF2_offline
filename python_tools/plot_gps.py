import matplotlib.pyplot as plt2
import numpy as np4
import matplotlib as mpl
import matplotlib as mpl2
mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['font.sans-serif'] = 'NSimSun,Times New Roman'

(t, a, b, c, d, e, lat, lon, hgt, f, g, h, i, j) = np4.loadtxt('/home/abner/UFO/ecl/EKF/build/data/gps_data.txt', unpack=True)

 
fig2 = plt2.figure()
cx1 = fig2.add_subplot(131)
cx1.plot(t, lat)
cx2 = fig2.add_subplot(132)
cx2.plot(t, lon)
cx3 = fig2.add_subplot(133)
cx3.plot(t, hgt)
plt2.show()

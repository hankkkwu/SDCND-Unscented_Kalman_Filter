import matplotlib.pyplot as plt
import numpy as np

lidar_nis = []
baseline_l = []
radar_nis = []
baseline_r = []
f_lidar = open("build/lidar_output.txt", "r")
f_radar = open("build/radar_output.txt", "r")
f1 = f_lidar.readlines()
f2 = f_radar.readlines()
for l in f1:
    lidar_nis.append(float(l[:-2]))
    baseline_l.append(5.991)
for r in f2:
    radar_nis.append(float(r[:-2]))
    baseline_r.append(7.815)

plt.plot(radar_nis, label="radar_nis")
plt.plot(baseline_r, label="95%")
# plt.plot(lidar_nis, label="radar_nis")
# plt.plot(baseline_l, label="95%")
plt.legend()   # show lines name
plt.xlim(0, 250)
plt.ylim(0, 16)
plt.ylabel("NIS")
plt.show()

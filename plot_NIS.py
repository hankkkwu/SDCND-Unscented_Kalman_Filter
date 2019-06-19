import matplotlib.pyplot as plt
import numpy as np

nis = []
f=open("build/output.txt", "r")
f1 = f.readlines()
for i in f1:
    nis.append(float(i[:-2]))
# t = np.arange(0, len(nis), 1)
plt.plot(nis)
plt.ylabel("NIS")
plt.show()

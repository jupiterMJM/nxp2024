import json
from matplotlib import pyplot as plt
import matplotlib.markers
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


with open("trajectory.json", "r") as f:
    data = json.load(f)
x, y, z = data["north"], data["east"], data["down"]  # for show
c = data["speed"]  # create some colours



p = ax.scatter(x, y, z, c=c, cmap=plt.cm.magma)
ax.set_xlabel("North")
ax.set_ylabel("East")
ax.set_zlabel("Down")

try:
    aim = data["Info"]
    print(aim)
    ax.scatter(*aim[:-1], c="red")
except:
    pass

fig.colorbar(p, ax=ax)
plt.show()
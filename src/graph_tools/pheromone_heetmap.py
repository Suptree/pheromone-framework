import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import seaborn as sns

parent = Path(__file__).resolve().parent

print(parent)
with open(parent.joinpath
          ('../pheromone/pheromone_saved/experients_01.pheromone'), 'rb') as f:
    data = np.load(f)
    # print("The pheromone matrix {} is successfully loaded".
    #           format(file_name))
    # data = np.arange(0, 100).reshape(10, 10)
    # print(data)

    # output:
    # [[ 0  1  2  3  4  5  6  7  8  9]
    #  [10 11 12 13 14 15 16 17 18 19]
    #  [20 21 22 23 24 25 26 27 28 29]
    #  [30 31 32 33 34 35 36 37 38 39]
    #  [40 41 42 43 44 45 46 47 48 49]
    #  [50 51 52 53 54 55 56 57 58 59]
    #  [60 61 62 63 64 65 66 67 68 69]
    #  [70 71 72 73 74 75 76 77 78 79]
    #  [80 81 82 83 84 85 86 87 88 89]
    #  [90 91 92 93 94 95 96 97 98 99]]

# fig, ax = plt.subplots()
# im = ax.imshow(data)
# plt.colorbar(im)
sns.heatmap(data, linewidth = 0.01)
# plt.xticks([-200,-150,-100,-50,0,50,100,150,200])
plt.show()

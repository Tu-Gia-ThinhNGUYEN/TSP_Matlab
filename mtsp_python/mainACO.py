import numpy as np
import math
from ACO_function import *

# Input xy position
xy = [[2, 1],
        [2,7],
        [6,3],
        [8,3],
        [7,4],
        [8,7],
        [9,8],
        [5,9],
        [4,4],
        [1,5]]
x = []
y = []
for i in range(len(xy)):
    x.append(xy[i][0])
    y.append(xy[i][1])

# Calculate distance
d = np.zeros((len(x),len(y)))
for i in range(len(xy)):
    for j in range(len(xy)):
        d[i,j] = math.sqrt((x[i] - x[j])**2 + (y[i]-y[j])**2)


# Parameters of ACO
miter = 100
m = 100

v = 3
depot = 9

p = 0.15
alpha = 2
beta = 4

n = len(x)
sales = round((n-1)/v)
if (n-1)%v != 0:
    sales = sales + 1

print(sales)

pathOfmTSP,minDistance = mainACO(miter,m,sales,depot,p,alpha,beta,d,v)
print(pathOfmTSP)
print(minDistance)
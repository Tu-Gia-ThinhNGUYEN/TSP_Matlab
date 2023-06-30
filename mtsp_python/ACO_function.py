import random
import numpy as np
import time

def antTour(city,m,n,h,t,alpha,beta,sales):
    for i in range(0,m):
        mh = h.copy()
        stationObtained = round((n-1)/sales)
        # if i==1:
        #     print(h)
        #     time.sleep(30000)
        for j in range(0, n-1+sales-1):
            # if j==2:
            #     print(h)
            #     time.sleep(30000)
            if j == stationObtained:
                c = city[i,j].copy()
                mh[:,c-1]=0.0
                city[i,j+1]=city[i,0].copy()
                stationObtained = stationObtained + j+1
                continue
            c = city[i,j].copy()
            # print(mh[c-1,:])
            mh[:,c-1]=0.0
            temp = (t[c-1,:]**alpha)*(mh[c-1,:]**beta)
            s = sum(temp)
            p = (1/s)*temp
            r = random.random()
            s = 0.0
            for k in range(0,n):
                s = s+p[k]
                if r<=s:
                    city[i,j+1] = k+1
                    break
        # print(s)
    tour = city
    return tour

def calculateDistance(m, n, d, tour,sales):
    distance = np.zeros((m,1))
    for i in range(0,m):
        s=0
        for j in range(0,n+sales-1):
            s = s+d[tour[i,j].copy()-1,tour[i,j+1].copy()-1].copy()
        distance[i,0] = s
    return distance

def pheromoneUpdate(m, n, t, tour, distance, p,sales):
    t_temp = t.copy()
    for i in range(0,m):
        for j in range(0, n+sales-1):
            dt = 1/distance[i].copy()
            t_temp[tour[i,j].copy()-1,tour[i,j+1].copy()-1]=(1-p)*t[tour[i,j].copy()-1,tour[i,j+1].copy()-1].copy()+dt
    return t_temp

def mainACO(miter,m,sales,depot,p,alpha,beta,d):
    n = len(d)
    t = 0.0001*np.ones((len(d),len(d)))
    # Create the heuristic matrix
    h = np.zeros((len(d),len(d)))
    for i in range(len(d)):
        for j in range(len(d)):
            if d[i,j]==0:
                h[i,j] = 0
            else:
                h[i,j] = 1/d[i,j]
    # Ant colony Optimization Algorithm
    city = np.zeros((m,n+sales))
    # print(h[0,:])
    bestSolution = np.zeros((miter,1))
    besttour = np.zeros((miter,n+sales))
    besttour = besttour.astype(int)
    for i in range(0,miter):
        # print(h[0,:])
        # Step 1: Initializing the ants: For each ant, a random starting city is selected.
        for j in range(0,m):
            city[j,0] = depot
            city[j,-1] = depot
        city = city.astype(int)
        # Step 2:  Constructing solutions: Each ant constructs a solution by moving from its current city to the next city 
        # based on the pheromone level and the heuristic information.
        # print(h)
        tour = antTour(city, m, n, h, t, alpha, beta,sales)
        # Step 3: Updating pheromone levels: After all ants have constructed a solution, 
        # the pheromone level on each edge is updated based on the distance of the tour.
        distance = calculateDistance(m, n, d, tour,sales)
        t = pheromoneUpdate(m, n, t, tour, distance, p,sales)
        # Step 4: Evaluating the best solution: The best solution found in the current iteration is recorded.
        bestSolution[i,0] = float(np.amin(distance,axis=0))
        best_index = int(np.argmin(distance,axis=0))
        besttour[i,:] = tour[best_index,:]
   
    print(besttour)
    minDistance = float(np.amin(bestSolution,axis=0))
    minIndex = int(np.argmin(bestSolution,axis=0))
    generalPath = besttour[minIndex,:]
    print(generalPath)
    pathOfmTSP = np.zeros((sales, round((n-1)/sales)+2))
    index = 0
    j = 0
    stationObtained = round((n-1)/sales)
    for i in range(0,n+sales-1):
        pathOfmTSP[index,i-j] = generalPath[i]
        if i == stationObtained+1:
            stationObtained = stationObtained + i
            index = index + 1
            j = i
    for i in range(0,sales):
        pathOfmTSP[i,-1] = depot
    pathOfmTSP = pathOfmTSP.astype(int)
    print(pathOfmTSP)
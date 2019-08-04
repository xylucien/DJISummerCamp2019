import math
import os
import time

Mymap = [[0 for i in range (9)] for i in range (7)]
Territorymap = [[0 for i in range (9)] for i in range (7)]
visited = [[0 for i in range (9)] for i in range (7)]
step_limit = 15
time_limit = 40
x = 0
y = 0
t = 0
sentryPost = 0
end = False
total_territory = []
total_sentryPost = []
areaVisited = [(x,y)]

# Define Sentry Posts
# 1 -> Sentry Post, 0 -> 1 red / 2 blue, 00 -> Red Energy Points, 00 -> Blue Energy Points
Mymap[0][2] = 100000
Mymap[0][6] = 100000
Mymap[3][0] = 100000
Mymap[3][4] = 100000
Mymap[3][8] = 100000
Mymap[6][2] = 100000
Mymap[6][6] = 100000

Territorymap[0][2] = 100000
Territorymap[0][6] = 100000
Territorymap[3][0] = 100000
Territorymap[3][4] = 100000
Territorymap[3][8] = 100000
Territorymap[6][2] = 100000
Territorymap[6][6] = 100000

# Define Starting Zones
# -1 -> Starting Zone
Mymap[0][0] = -1
Mymap[0][8] = -1
Mymap[6][0] = -1
Mymap[6][8] = -1

# Test
Mymap[0][1] = 1
Mymap[1][1] = 1
Mymap[2][1] = 1
Mymap[3][1] = 1
Mymap[3][2] = 1
Mymap[3][3] = 1
Mymap[2][3] = 1

Mymap[6][8] = 1
Mymap[6][7] = 1
Mymap[5][7] = 1
Mymap[4][7] = 1
Mymap[3][7] = 1
Mymap[2][7] = 1
Mymap[1][7] = 1
Mymap[0][7] = 1

def printMymap():
    for i in range(7):
        print(Mymap[i])

def printTerritory():
    for i in range(7):
        print(Territorymap[i])

def search (x,y):
    global end, t
    #areaVisited.append((x,y))
    if visited[x][y]:
        return
    visited[x][y] = True

    if x<6 and Mymap[x+1][y] == 1 and (not visited[x+1][y]):         # South
        total_territory.append((x+1,y,t))
        Territorymap[x+1][y] = t
        search(x+1, y)

    if y<8 and Mymap[x][y+1] == 1 and (not visited[x][y+1]):         # East
        total_territory.append((x,y+1,t))
        Territorymap[x][y+1] = t
        search(x, y+1)

    if x>0 and Mymap[x-1][y] == 1 and (not visited[x-1][y]):         # North
        total_territory.append((x-1,y,t))
        Territorymap[x-1][y] = t
        search(x-1, y)

    if y > 0 and Mymap[x][y-1] == 1 and (not visited[x][y-1]):         # West
        total_territory.append((x,y-1,t))
        Territorymap[x][y-1] = t
        search(x, y-1)


printMymap()
print(''
      ''
      '')

for x in range(6):
    for y in range(8):
        if Mymap[x][y] == 1:
            flag = False
            if not visited[x][y]:
                t+=1
                total_territory.append((x,y,t))
                Territorymap[x][y] = t
                search(x,y)



print(areaVisited)
printTerritory()
print (total_territory)

for i in range(len(total_territory)):
    x1 = total_territory[i][0]
    y1 = total_territory[i][1]
    t1 = total_territory[i][2]
    if x1<6 and Mymap[int(x1+1)][y1] >= 100000:
        sentryPost += 1
    if y1 < 8 and Mymap[x1][int(y1+1)] >= 100000:
        sentryPost += 1
    if x1 > 0 and Mymap[int(x1-1)][y1] >= 100000:
        sentryPost += 1
    if y1 > 0 and Mymap[x1][int(y1-1)] >= 100000:
        sentryPost += 1

print(t1, sentryPost)




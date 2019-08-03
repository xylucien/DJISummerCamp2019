import math
import os
import time
import queue

Mymap = [[0 for i in range(9)] for i in range(7)]
step_limit = 15
time_limit = 40
action = {'n':1, 'e':1, 's':1, 'w':1, 'stay!': 1, 'turn&put': 10}
search_queue = queue.PriorityQueue() 

#best_path = ""
#max_point = -1

Mymap[0][2] = 100000
Mymap[0][6] = 100000
Mymap[3][0] = 100000
Mymap[3][4] = 100000
Mymap[3][8] = 100000
Mymap[6][6] = 100000

initR = 0
initC = 0

oppoR = 6
oppoC = 8

#temps
pcount = 0
flag = False
class state():
	def __init__(self, cs, tl):
		self.curState = cs
		self.timeLeft = tl
'''
def printMymap():
	for i in range(7):
		print(Mymap[i])
'''
#printMymap()
#记录连接信息用一个新的bool 每次从队列挤出时刷新bool状态 初始状态根据开始时是否连接路径、城堡确定 连接到城堡时刷新
def check_sp(r,c):
	if r == 0 or r == 6:
		if c == 2 or c == 6:
			return True
	if r == 3:
		if c == 0 or c == 4 or c == 8:
			return True
	return False

def check_put(r, c):
	return check_sp(r-1, c) or check_sp(r+1, c) or check_sp(r, c-1) or check_sp(r, c+1)

def check_oppo(r,c):
	if r == oppoR and c == oppoC:
		return True
	return False

def near_secured(r,c):
	#await implemented
	return False

def search():
	global pcount, flag
	if pcount > 100:
		flag = True
		return
	(pts, (state, tleft, r, c, step)) = search_queue.get()
	if check_sp(r,c) or check_oppo(r,c):
		return
	#if pcount%100000 == 0:
	#	print(pcount)
	if tleft == 0 or step == step_limit:
		pcount+=1
		print(state, r, c, step, -pts)
		return
	if tleft >=10 and check_put(r, c):
		search_queue.put((pts-50, (state+'&', tleft - 10, r, c, step)))
	if tleft >=1:
		if state[-1:] != 's' and r > 0:
			search_queue.put((pts, (state+'n', tleft - 1, r-1, c, step+1)))
		if state[-1:] != 'n' and r < 6:	
			search_queue.put((pts, (state+'s', tleft - 1, r+1, c, step+1)))	
		if state[-1:] != 'w' and c < 8:
			search_queue.put((pts, (state+'e', tleft - 1, r, c+1, step+1)))
		if state[-1:] != 'e' and c > 0:	
			search_queue.put((pts, (state+'w', tleft - 1, r, c-1, step+1)))
		if state[-1:] != '!' and (state[-2:-1] == '!' or near_secured(r,c) or check_put(r,c)):
			search_queue.put((pts - 15, (state+'!', tleft - 1, r, c, step)))
#a = 'asdfghjkl'
#print(a[-2:-1])
search_queue.put((0,(' ', 40, initR, initC, 0)) )
while not flag:
	search()
#print(check_put(0,0))
print(pcount)

#print(search_queue.get()[1][1])
'''
search_queue.put((-15, (1,2,3,4,5)))
search_queue.put((-145, (1,244,3,4,5)))
search_queue.put((-515, (1,23,3,4,5)))
search_queue.put((-155, (1,2,3,4,5)))
a=0
b=0
c=0
d=0
e=0
f=0
(a, (b, c, d, e, f)) = search_queue.get()
print(a,b,c,d,e,f)
'''
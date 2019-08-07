import math
import os
import time
import queue

Mymap = [[0 for i in range(9)] for i in range(7)]
step_limit = 15
time_limit = 40
action = {'n':1, 'e':1, 's':1, 'w':1, 'stay!': 1, 'turn&put': 10}
search_queue = queue.PriorityQueue() 

best_path = []
max_point = -1

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

'''
def printMymap():
	for i in range(7):
		print(Mymap[i])
'''
#printMymap()

def calculate_sp(r, c):
	pass

def check_sp_coor(r,c):
	if r == 0 or r == 6:
		if c == 2 or c == 6:
			return r,c
	if r == 3:
		if c == 0 or c == 4 or c == 8:
			return r,c
	return -1,-1

def check_sp(r,c):
	a, b = check_sp_coor(r,c)
	return False if (a==-1 and b==-1) else True

def check_put(r, c):
	return check_sp(r-1, c) or check_sp(r+1, c) or check_sp(r, c-1) or check_sp(r, c+1)

def check_oppo(r,c):
	if r == oppoR and c == oppoC:
		return True
	return False

def get_near_sp(r,c):
	sp_x, sp_y = check_sp_coor(r-1, c)
	if sp_x == -1:
		sp_x, sp_y = check_sp_coor(r+1, c)
	if sp_x == -1:
		sp_x, sp_y = check_sp_coor(r, c-1)
	if sp_x == -1:
		sp_x, sp_y = check_sp_coor(r, c+1)
	return Mymap[sp_x][sp_y], sp_x, sp_y

def near_secured(r,c):
	if r > 0:
		if Mymap[r-1][c] < 100000 and Mymap[r-1][c]%10 == 1:
			return True
		elif int((Mymap[r-1][c]-100000)/10000) == 1:
			return True
	if r < 6:
		if Mymap[r+1][c] < 100000 and Mymap[r+1][c]%10 == 1:
			return True
		elif int((Mymap[r+1][c]-100000)/10000) == 1:
			return True
	if c < 8:
		if Mymap[r][c+1] < 100000 and Mymap[r][c+1]%10 == 1:
			return True
		elif int((Mymap[r][c+1]-100000)/10000) == 1:
			return True
	if c > 0:	
		if Mymap[r][c-1] < 100000 and Mymap[r][c-1]%10 == 1:
			return True
		elif int((Mymap[r][c-1]-100000)/10000) == 1:
			return True			
	return False

def check_virtual_spinfo(spInfo, r, c):
	if r == 0:
		if c == 2:
			return spInfo&1, 1
		elif c == 6:
			return (spInfo>>1)&1, 2
	if r == 3:
		if c == 0:
			return (spInfo>>2)&1, 4
		elif c == 4:
			return (spInfo>>3)&1, 8
		elif c == 8:	
			return (spInfo>>4)&1, 16
	if r == 6:
		if c == 2:
			return (spInfo>>5)&1, 32
		elif c == 6:
			return (spInfo>>6)&1, 64
	return -1, -1

def search():
	global pcount, flag, max_point, best_path
	if pcount%100000 == 0:
		print(str(int(pcount/100000)) + r'% finished')
	if pcount >= 10000000:
		flag = True
		return
	
	(pts, (state, tleft, r, c, step, connected), spInfo) = search_queue.get()
	
	last_step = state[-1:]
	stayed = (not (last_step == 's' or last_step == 'w' or last_step == 'e' or last_step == 'n'))
	if check_sp(r,c) or check_oppo(r,c):
		return

	if tleft == 0 or step == step_limit:
		pcount+=1
		#print(state, r, c, step, -pts)
		if -pts > max_point:
			best_path = []
			max_point = -pts
		if -pts == max_point:
			best_path.append(state)
		return

	if tleft >=10 and check_put(r, c):
		info, sp_r, sp_c = get_near_sp(r,c)
		has_put, put_id = check_virtual_spinfo(spInfo, sp_r, sp_c)
		if int((info-100000)/10000) == 1 or (has_put!=0):
			pass
		else:
			search_queue.put((pts-50, (state + '&', tleft - 10, r, c, step, True), spInfo+put_id))

	if tleft >=1:
		if last_step != 's' and r > 0:
			search_queue.put((pts + 5 if (Mymap[r-1][c]%10==2) else pts, (state + 'n', tleft - 1, r-1, c, step+1, stayed), spInfo))
		if last_step != 'n' and r < 6:	
			search_queue.put((pts + 5 if (Mymap[r+1][c]%10==2) else pts, (state + 's', tleft - 1, r+1, c, step+1, stayed), spInfo))	
		if last_step != 'w' and c < 8:
			search_queue.put((pts + 5 if (Mymap[r][c+1]%10==2) else pts, (state + 'e', tleft - 1, r, c+1, step+1, stayed), spInfo))
		if last_step != 'e' and c > 0:	
			search_queue.put((pts + 5 if (Mymap[r][c-1]%10==2) else pts, (state + 'w', tleft - 1, r, c-1, step+1, stayed), spInfo))
		if last_step != '!' and (connected or near_secured(r,c)):
			search_queue.put((pts - 15, (state + '!', tleft - 1, r, c, step, True), spInfo))

search_queue.put((0,('', 40, initR, initC, 0, False), 0))

'''
for i in range(7):
	for j in range(9):
		print(get_near_sp(i,j), end = ' & ')
	print()
	print()
'''
while not flag:
	search()
print(pcount)
print(best_path, len(best_path), max_point)
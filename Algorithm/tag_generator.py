a = '<node pkg="tf2_ros" type="static_transform_publisher" name="map_tag_'
  
b = '" args="'

c = ' 0 1.57 0 0 map tag_'

d = '" />'

aa = '{id: ' 

bb = ', size: 0.144, name: tag_'

cc = '},'


file = 'C:\\Users\\Lucien\\Desktop\\test.txt'
with open(file, 'w') as f:
	for i in range(7):
		for j in range(9):
			#f.write(a + str(i*9 + j) + b + str(round(0.465 + 0.93*i,3)) + ' ' + str(round(0.465 + 0.93*j,3))+ c + str(i*9 + j) + d)
			#f.write(aa+str(i*9+j)+bb+str(i*9+j)cc)
			print(aa+str(i*9+j)+bb+str(i*9+j)+cc)
			#f.write('\n')
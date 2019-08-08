#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import pygame
from pygame.locals import *
from sys import exit

ARM_MOVE_SPD=0.005

pygame.init()
screen=pygame.display.set_mode((200,200),0,32)
BG=pygame.image.load("D:\\mydata\\Programs\\git_try\\DJISummerCamp2019\\Algorithm\\ui\\uiPic\\CHTMTR.jpg").convert_alpha()
screen.blit(BG,(0,0))
pygame.display.update()
ArmMotor=0.0
LeftRotor=0
RightRotor=0

def talker():
	pub_l = rospy.Publisher('left_ball', Int16, queue_size=10)
	pub_r = rospy.Publisher('right_ball', Int16, queue_size=10)
	pub_a = rospy.Publisher('arm', Float64, queue_size=10)
	
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		for event in pygame.event.get():
			if event.type==QUIT:
				pygame.quit()
				exit()
			if event.type==KEYDOWN:
				if event.key==K_o:
					LeftRotor+=1
				if event.key==K_p:
					RightRotor+=1
		KEY_STATE=pygame.key.get_pressed()
	
		if KEY_STATE[K_UP] and ArmMotor<1:
			ArmMotor+=ARM_MOVE_SPD
		if KEY_STATE[K_DOWN] and ArmMotor>0:
			ArmMotor-=ARM_MOVE_SPD

		ArmMotor=round(ArmMotor,5)

		pygame.time.delay(5)
		rospy.loginfo(LeftRotor, RightRotor, ArmMotor)
		pub_l.publish(LeftRotor)
		pub_r.publish(RightRotor)
		pub_a.publish(ArmMotor)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
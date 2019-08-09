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

while True:
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
    print(ArmMotor,'\t\t',LeftRotor,'\t\t',RightRotor)

    pygame.time.delay(5)
        

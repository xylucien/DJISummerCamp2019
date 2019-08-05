#!/usr/bin/python3

import time
import os
import threading
import pygame
from pygame.locals import *
from enum import Enum
from sys import exit

TEST=True
SCREEN_MODE=0
KEYBOARD_DRIVE_ROBOT=True
CONNECTION_PASS_CASTLE=True
RAISE_ERROR_WHEN_HIT=False
MOVE_TIME=0
TURN_TIME=0
CAPTURE_TIME=0
PUT_TIME=0

FILE_ROUTE="D:\\mydata\\pics\\RoboMaster\\"

def pause(*args):
    pass #在这个pass设置断点


if TEST:
    pygame.init()
    screen=pygame.display.set_mode((1920,1080),SCREEN_MODE,32)

    def ImageLoad(filename):
        global FILE_ROUTE
        try:
            PIC=pygame.image.load(FILE_ROUTE+filename+".jpg").convert_alpha()
        except Exception:
            PIC=pygame.image.load(FILE_ROUTE+filename+".png").convert_alpha()
        assert type(PIC)==pygame.Surface
        return PIC
        
    BACKGROUNDPIC=ImageLoad("BackGround")
    REDBK=ImageLoad("RedBlock")
    BLUEBK=ImageLoad("BlueBlock")
    ORANGEBK=ImageLoad("OrangeBlock")
    CASTLEBK=ImageLoad("Castle")
    ROBOTPICS=[ImageLoad("S1")]


    class CaptureState(Enum):
        free=0
        blue=1
        red=2


    class Block(object):
        def __init__(self,NO):
            assert type(NO)==int

            self.no=NO
            self.state=CaptureState["free"]
            self.strong=False
            self.poz=(self.no%9,self.no//9)
            self.crd=((510+self.poz[0]*100),(190+self.poz[1]*100))
            self.castle_connected={CaptureState["blue"]:[],CaptureState["red"]:[]}
            self.energy=0

            if self.poz in[(0,0),(8,6)]:
                self.tp='b'
                self.pic=BLUEBK
            elif self.poz in[(0,6),(8,0)]:
                self.tp='r'
                self.pic=REDBK
            elif self.poz in [(2,0),(6,0),(0,3),(4,3),(8,3),(2,6),(6,6)]:
                self.tp='c'
                self.pic=CASTLEBK
            else:
                self.tp='o'
                self.pic=ORANGEBK

        def energy_update(self):
            if self.state!=CaptureState["free"]:
                enbuf=0
                for castle in self.castle_connected[self.state]:
                    if castle.state==self.state:
                        enbuf+=1
                self.energy=enbuf
                if self.energy>=3:
                    self.strong=True
                else:
                    self.strong=False

        def display_to(self,screen,facing=0):
            assert type(screen)==pygame.Surface


            picBuf=self.pic.copy()
            picBuf=pygame.transform.rotate(picBuf,facing*(-90))

            if self.state==CaptureState["free"]:
                pass
            elif self.state==CaptureState["red"]:
                if not self.strong:
                    pygame.draw.rect(picBuf,(255,0,0),(25,40,51,20))
                    pygame.draw.rect(picBuf,(255,0,0),(40,25,20,51))
                else:
                    pygame.draw.rect(picBuf,(255,0,0),(2,40,96,20))
                    pygame.draw.rect(picBuf,(255,0,0),(40,2,20,96))
            elif self.state==CaptureState["blue"]:
                if not self.strong:
                    pygame.draw.rect(picBuf,(0,0,255),(25,40,51,20))
                    pygame.draw.rect(picBuf,(0,0,255),(40,25,20,51))
                else:
                    pygame.draw.rect(picBuf,(0,0,255),(2,40,96,20))
                    pygame.draw.rect(picBuf,(0,0,255),(40,2,20,96))
            else:
                raise ValueError
            screen.blit(picBuf,self.crd)


    class Mp(object):
        def __init__(self):
            self.blocks=[]
            self.castles=[]
            for i in range(0,63):
                if i in [2,6,27,31,35,56,60]:
                    c=Castle(i)
                    self.blocks.append(c)
                    self.castles.append(c)
                else:
                    self.blocks.append(Block(i))


        def BCU(self,blk,num,blst):
            if 0<=num<63:
                if blk.state!=CaptureState["free"] and blst[num].state==blk.state:
                    for cast in blst[num].castle_connected[blk.state]:
                        if not cast in blk.castle_connected[blk.state]:
                            blk.castle_connected[blk.state].append(cast)
            return


        def connection_update(self):
            for team in [CaptureState["red"],CaptureState["blue"]]:
                for block in self.blocks:
                    if type(block)==Block:
                        block.castle_connected={CaptureState["blue"]:[],CaptureState["red"]:[]}

                for i in range(0,14):
                    for block in self.blocks:
                        if type(block)==Block or CONNECTION_PASS_CASTLE:
                            self.BCU(block,block.no+1,self.blocks)
                            self.BCU(block,block.no-1,self.blocks)
                            self.BCU(block,block.no+9,self.blocks)
                            self.BCU(block,block.no-9,self.blocks)
            for block in self.blocks:
                block.energy_update()

        def capture(self,state,poz=None,NO=None):
            assert state==CaptureState["red"] or state==CaptureState["blue"]
            if poz==None and NO==None:
                raise ValueError
            elif poz==None:
                assert type(NO)==int and 0<NO<63
                poz=(NO%9,NO//9)
            NO=poz[0]+poz[1]*9
            if self.blocks[NO].strong==False:
                for pz in [(poz[0],poz[1]+1),(poz[0],poz[1]-1),(poz[0]+1,poz[1]),(poz[0]-1,poz[1])]:
                    if 0<=pz[0]<9 and 0<=pz[1]<7 and self.blocks[pz[0]+pz[1]*9].state==state:
                        self.blocks[NO].state=state

            
        def display(self,screen):
            assert type(screen)==pygame.Surface

            for block in self.blocks:
                block.display_to(screen)


    class Robot(object):
        def __init__(self,NO,team,init_poz=(0,0),init_direction='N'):
            assert type(NO)==int

            self.no=NO
            self.team=CaptureState[team]
            self.poz=init_poz
            self.direction=init_direction
            self.pic=ROBOTPICS[0].copy().convert_alpha()
            if self.team==CaptureState["red"]:
                pygame.draw.rect(self.pic,(255,0,0),(40,40,20,20))
            elif self.team==CaptureState["blue"]:
                pygame.draw.rect(self.pic,(0,0,255),(40,40,20,20))
        
        @staticmethod
        def DFC(direction):     #direction format change
            if type(direction)==str and direction in "NSWEFBLR":
                if direction=='N' or direction=='F':
                    return 0
                elif direction=='S' or direction=='B':
                    return 2
                elif direction=="W" or direction=='L':
                    return 3
                elif direction=='E' or direction=='R':
                    return 1
                else:
                    raise ValueError
            else:
                assert type(direction)==int

                if direction%4==0:
                    return 'N'
                elif direction%4==1:
                    return 'E'
                elif direction%4==2:
                    return 'S'
                elif direction%4==3:
                    return 'W'

        def crd_calu(self):
            self.crd=((510+self.poz[0]*100),(190+self.poz[1]*100))

        def setpoz(self,poz):
            assert type(poz)==tuple
            assert type(poz[0])==type(poz[1])==int
            assert 0<=poz[0]<9 and 0<=poz[1]<7

            self.poz=poz

        def movpoz(self,direction,lth,detect=True,opppoz=None):
            assert type(lth)==int

            pozbuf=list(self.poz)
            if type(direction)==str and direction in "NSWE" and len(direction)==1:
                pass
            elif type(direction)==str and direction in "FBLR":
                direction=self.DFC((self.DFC(direction)+self.DFC(self.direction))%4)
            elif type(direction)==int:
                direction=self.DFC(direction)
            else:
                raise ValueError

            if direction=='N':
                pozbuf[1]-=lth
            elif direction=='S':
                pozbuf[1]+=lth
            elif direction=='W':
                pozbuf[0]-=lth
            elif direction=='E':
                pozbuf[0]+=lth

            if not detect:
                self.poz=tuple(pozbuf)

            if 0<=pozbuf[0]<9 and 0<=pozbuf[1]<7 and not\
                    tuple(pozbuf) in [(2,0),(6,0),(0,3),(4,3),(8,3),(2,6),(6,6)] and opppoz!=None and tuple(pozbuf)!=opppoz:
                self.poz=tuple(pozbuf)
            elif RAISE_ERROR_WHEN_HIT:
                raise ValueError
        
        def set_facing(self,direction):
            if type(direction)==str and direction in "NSWE" and len(direction)==1:
                self.direction=direction
            elif type(direction)==int and 0<=direction<4:
                self.direction=self.DFC(direction)

        def rotate(self,rotation):
            if type(rotation)==int:
                self.direction=self.DFC((self.DFC(self.direction)+rotation)%4)
            elif rotation in "FBLR":
                self.direction=self.DFC((self.DFC(self.direction)+self.DFC(rotation))%4)
            elif rotation in "NSWE":
                self.set_facing(rotation)
            else: raise ValueError
        
        def display_to(self,screen,crd=None):
            assert type(screen)==type(self.pic)==pygame.Surface
            if crd==None:
                self.crd_calu()
                crd=self.crd
            screen.blit(pygame.transform.rotate(self.pic,(self.DFC(self.direction))*(-90)),crd)

    class Castle(Block,Robot):
        def __init__(self,NO):
            super().__init__(NO)

            castledict={2:0,6:1,27:2,31:3,35:4,56:5,60:6}
            self.castleNO=castledict[NO]
            self.castle_connected={CaptureState["blue"]:[self],CaptureState["red"]:[self]}

            if NO in [27,31]:
                self.direction='N'
            elif NO in [56,60]:
                self.direction='E'
            elif NO==35:
                self.direction='S'
            elif NO in [2,6]:
                self.direction='W'
            else:
                raise ValueError
            
            self.capture_point={}
            self.movpoz('F',1,False)
            self.capture_point.update({CaptureState["blue"]:self.poz})
            self.movpoz('B',2,False)
            self.capture_point.update({CaptureState["red"]:self.poz})
            self.movpoz('F',1,False)

            self.ballamount = { CaptureState["red"]:0 , CaptureState["blue"]:0 }

        def capture_update(self):
            if self.ballamount[CaptureState["red"]]>self.ballamount[CaptureState["blue"]]:
                self.state=CaptureState["red"]
            elif self.ballamount[CaptureState["red"]]<self.ballamount[CaptureState["blue"]]:
                self.state=CaptureState["blue"]
            else:
                self.state=CaptureState["free"]

        def display_to(self, screen):
                super().display_to(screen,self.DFC(self.direction))

    mp=Mp()

    robots=[Robot(0,"blue",(0,0),'S'),Robot(1,"red",(8,0),'S')]
    
    def get_robot_info(opp=False):
        if opp: return get_opponent_info()
        return (robots[0].poz[0],robots[0].poz[1],robots[0].direction)

    def get_opponent_info():
        return (robots[1].poz[0],robots[1].poz[1])

    def get_sp_info(no):
        assert type(no)==int and 0<=no<7
        return mp.castles[no]
    
    def get_road_info(no):
        assert type(no)==int and 0<=no<63 and type(mp.blocks[no])==Block
        return mp.blocks[no]

    def move(direction,lth,team=CaptureState["blue"]):
        time.sleep(MOVE_TIME)
        robots[team.value-1].movpoz(direction,lth,detect=True,opppoz=robots[{CaptureState["blue"]:1,CaptureState["red"]:0}[team]].poz)

    def turn(direction,team=CaptureState["blue"]):
        time.sleep(TURN_TIME)
        robots[team.value-1].rotate(direction)

    def stay(s=CAPTURE_TIME,team=CaptureState["blue"]):
        if type(s)==int:
            s=float(s)
        elif type(s)==float:
            pass
        else:
            raise TypeError
        if s>CAPTURE_TIME-0.01:
            mp.capture(robots[team.value-1].team,robots[team.value-1].poz)
            mp.connection_update()
        time.sleep(s)
    
    def put(cup=False,team=CaptureState["blue"]):
        for cst in mp.castles:
            if robots[team.value-1].poz==cst.capture_point[team]:
                a=Robot.DFC(robots[team.value-1].direction)
                b=(Robot.DFC(cst.direction)+2)%4
                if Robot.DFC(robots[team.value-1].direction)==(Robot.DFC(cst.direction)+(team.value*2))%4:
                    if cup: cst.ballamount[team] += 4
                    else: cst.ballamount[team] += 1
                    time.sleep(PUT_TIME)
                    cst.capture_update()
                    mp.connection_update()
                    break

    def OS(str):
        os.system(str)


    print("")
    print("RoboMaster 2019 Summen Camp    <<Robot Tester>>     By:AAA_shuibiao   QQ:1308499101   ---open source---   free to copy")
    print("RoboMaster 2019 夏令营         <<机器人测试器>>     制作:             QQ:             ------开源-------     允许复制")
    print("")
    print("输入 help() 也不会给你帮助")
    print("...>>>:")
    print("")


    def terminal_func():
        while True:
            try:
                print(">>>",end='')
                exec(input())
            except Exception as error:
                print(error)

    terminal=threading.Thread(target=terminal_func,name='terminal')
    terminal.start()


    def main():
        while True:
            time.sleep(1000)








    main=threading.Thread(target=main,name='main')
    main.start()


    while True:
        screen.blit(BACKGROUNDPIC,(0,0))

        for event in pygame.event.get():
            if event.type==QUIT and TEST:
                pygame.quit()
                exit()
            if event.type==KEYDOWN:
                if event.key==K_ESCAPE:
                    pygame.quit()
                    exit()
                if KEYBOARD_DRIVE_ROBOT:
                    if event.key==K_w: move('F',1)
                    if event.key==K_d: move('R',1)
                    if event.key==K_s: move('B',1)
                    if event.key==K_a: move('L',1)
                    if event.key==K_q: turn(-1)
                    if event.key==K_e: turn(1)
                    if event.key==K_r: put()
                    if event.key==K_f: stay(CAPTURE_TIME)
                    if event.key==K_u: move('F',1,CaptureState["red"])
                    if event.key==K_k: move('R',1,CaptureState["red"])
                    if event.key==K_j: move('B',1,CaptureState["red"])
                    if event.key==K_h: move('L',1,CaptureState["red"])
                    if event.key==K_y: turn(-1,CaptureState["red"])
                    if event.key==K_i: turn(1,CaptureState["red"])
                    if event.key==K_o: put(cup=False,team=CaptureState["red"])
                    if event.key==K_l: stay(CAPTURE_TIME,CaptureState["red"])

        mp.display(screen)

        for robot in robots:
            robot.display_to(screen)

        pygame.display.update()
        pygame.time.delay(30)


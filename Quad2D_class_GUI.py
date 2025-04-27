# -*- coding: utf-8 -*-
"""
Created on Sun Apr 27 16:10:32 2025

@author: theov
"""
import math as m
import tkinter as tk
import matplotlib.pyplot as plt
from PIL import Image, ImageTk

class Quad2D_GUI(tk.Tk):
    
    def __init__(self, main, physics, flightController, hitbox = True):
        tk.Tk.__init__(self)
        
        self.hitbox = hitbox
        
        self.title("Quad2D Simulation / Théo Vialis")
        
        self.main = main
        
        self.phys = physics
        
        self.FC = flightController
        
        self.Cv = tk.Canvas(self, width = self.main.XMax, height = self.main.YMax, background = "lightblue")
        self.Cv.pack()        
                
        self.background_setup()
        self.quad_im_setup()
        self.target_setup()
        
        ## Mode Human
        self.Cv.bind_all("<Key>", self.FC.ManControl_KP)   
        self.Cv.bind("p", self.plot_display) 
        self.Cv.bind_all("<KeyRelease>", self.FC.ManControl_KR)

    
    def background_setup(self):
        
        self.im_sun = Image.open("./assets/sun.png").resize((100,100))
        self.im_clouds = Image.open("./assets/clouds-moving.png").resize((700,300))
        
        self.Sun_PI = ImageTk.PhotoImage(self.im_sun)
        self.Clouds_PI = ImageTk.PhotoImage(self.im_clouds)
        
        self.sun = self.Cv.create_image(50,50,image = self.Sun_PI)
        self.clouds = self.Cv.create_image(400,250,image = self.Clouds_PI)
        
        self.floor =  self.Cv.create_rectangle(-10, self.main.YMax-5, self.main.XMax+10, self.main.YMax+10, fill ="brown")
        
        self.Tl_display = self.Cv.create_text(150, 20, text= f"Tl : {self.phys.Tl:.1f}")
        self.Tr_display = self.Cv.create_text(200, 20, text= f"Tr : {self.phys.Tr:.1f}")
        self.omega_display = self.Cv.create_text(300, 20, text= f"Omega : {self.phys.omega:.1f}")
        self.omegaT_display = self.Cv.create_text(375, 20, text= f"Omega_t : {self.FC.omega_target:.1f}")
        self.theta_display = self.Cv.create_text(450, 20, text= f"theta : {self.phys.theta*180/m.pi:.1f}°")
        
        self.mode_display = self.Cv.create_text(self.main.XMax - 150, 20, text= f"Mode : {self.FC.MODES[self.FC.curr_mode]}")
        
        self.score_display = self.Cv.create_text(self.main.XMax - 50, 20, text= f"Score : {self.main.score}")
        
    
    def target_setup(self):
        self.im_target = Image.open("./assets/red-plain-1.png").resize((self.phys.ltarget,self.phys.htarget))
        self.Target_PI = ImageTk.PhotoImage(self.im_target)
        
        self.target = self.Cv.create_image(self.phys.xtarget,self.phys.ytarget,image = self.Target_PI)
    
    def text_update(self):
        self.Cv.itemconfig(self.Tl_display, text= f"Tl : {self.phys.Tl:.1f}")
        self.Cv.itemconfig(self.Tr_display, text= f"Tr : {self.phys.Tr:.1f}")
        self.Cv.itemconfig(self.omega_display, text= f"Omega : {self.phys.omega:.1f}")
        self.Cv.itemconfig(self.omegaT_display, text= f"Omega_t : {self.FC.omega_target:.1f}")
        self.Cv.itemconfig(self.theta_display, text= f"theta : {self.phys.theta*180/m.pi:.1f}°")
        
        
        self.Cv.itemconfig(self.score_display, text= f"Score : {self.main.score}")
    
    def target_move(self):
        self.Cv.coords(self.target, self.phys.xtarget,self.phys.ytarget)
    
    def quad_im_setup(self):
        self.quad_im = []
        for i in range(4):           
            self.quad_im.append(Image.open(f"./assets/drone-{i+1}.png").resize((2*self.phys.l,20)))
            
        self.q_curr = 0
        self.quad_PI_update(self.q_curr, self.phys.theta)
        self.quad = self.Cv.create_image(self.phys.x, self.phys.y, image = self.quad_PI)
        
        if self.hitbox:
            self.hitbox_quad = self.Cv.create_rectangle(self.phys.x1_HB, self.phys.y1_HB, self.phys.x2_HB, self.phys.y2_HB, outline="red")
            self.hitbox_target = self.Cv.create_rectangle(self.phys.xtarget - self.phys.ltarget/2, self.phys.ytarget - self.phys.htarget/2, self.phys.xtarget + self.phys.ltarget/2 , self.phys.ytarget + self.phys.htarget/2, outline="blue")
        
    def quad_PI_update(self, i, theta):
        thetadeg = 180*theta/m.pi 
        self.quad_PI = ImageTk.PhotoImage(self.quad_im[i].rotate(thetadeg, expand =True))
        
    
    def quad_move(self):        
        
        ## Anim + Angle
        if self.phys.Tl > 0 or self.phys.Tr > 0 :
            self.q_curr = (self.q_curr + 1) % 4
        
        self.quad_PI_update(self.q_curr, self.phys.theta)
        self.Cv.itemconfig(self.quad, image = self.quad_PI)
        
        self.Cv.coords(self.quad, (self.phys.x, self.phys.y))
        if self.hitbox:
            self.Cv.coords(self.hitbox_quad, self.phys.x1_HB, self.phys.y1_HB, self.phys.x2_HB, self.phys.y2_HB)
    
    def gui_update(self):
                
        
        self.quad_move()
        
        self.text_update()
        #if self.phys.target_reached :
        self.target_move()
        if self.hitbox:
            self.Cv.coords(self.hitbox_target, self.phys.xtarget - self.phys.ltarget/2, self.phys.ytarget - self.phys.htarget/2, self.phys.xtarget + self.phys.ltarget/2 , self.phys.ytarget + self.phys.htarget/2)
           # self.phys.target_reached = False
        
        self.update()
    
    ### Probing Management -----------------------------    
    def plot_display(self):
        
        plt.figure()
        plt.plot(self.phys.omega_container, ".k")
        plt.grid()
        
    
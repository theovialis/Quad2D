# -*- coding: utf-8 -*-
"""
Created on Mon Apr 21 09:08:21 2025

@author: Théo

Projet de pilotage autonome d'un drone en 2D en prenant en compte la physique (calculs dynamiques)
inspiré de https://youtu.be/J1hv0MJghag?feature=shared / https://github.com/AlexandreSajus/Quadcopter-AI/blob/main/Reinforcement_Learning_for_the_Control_of_Quadcopters.pdf
(M. Sajus utilise Pygame/ Ici c'est Tkinter Canvas)
"""

import tkinter as tk
from PIL import Image, ImageTk
import math as m
import time, random

#--------------------------------------------------------------------------------------------
class Quad2D_Master:
    
    def __init__(self, GUI = True):
        self.gui = GUI
        
        self.Dt = .05
        
        self.phys = Quad2D_physics(self.Dt)
        self.phys.Initialize_Quad()
        
        
        self.MODE = "RATE"
        
        if self.gui:
            self.GuiClass = Quad2D_GUI(self.phys)
            ## Mode Human
            self.GuiClass.Cv.bind_all("<Key>", self.ManControl)     

        
        
        self.timeloop()
    
    def ManControl(self,e):
        
        key = e.keysym
        
        if key == "Right" and self.phys.theta >= -90:
#            self.omega_target -= m.radians(10)
            self.phys.Tl += self.phys.Dangle
            self.phys.Tr -= self.phys.Dangle
        elif key == "Left" and self.phys.theta <= 90:
#            self.omega_target += m.radians(10)
            self.phys.Tl -= self.phys.Dangle
            self.phys.Tr += self.phys.Dangle        
        elif key == "Up" :
            if self.phys.Tl < self.phys.Tmax:
                self.phys.Tl += self.phys.DThrottle
            if self.phys.Tr < self.phys.Tmax:
                self.phys.Tr += self.phys.DThrottle
        elif key == "Down" :
            if self.phys.Tl > 0:
                self.phys.Tl -= self.phys.DThrottle
            if self.phys.Tr > 0:
                self.phys.Tr -= self.phys.DThrottle
        elif key == "r" :
            self.phys.Initialize_Quad()
            
    
    
    def timeloop(self):
        while True :

            self.phys.timeIncrement()
            
            
            if self.gui :
                self.GuiClass.gui_update()
        
        
            time.sleep(self.Dt)
        

#--------------------------------------------------------------------------------------------
class Quad2D_ControlTools:
    def __init__(self):
        pass
        

    def PID_config(self, G=1.0, Ti=float('inf'), Td=0.0, output_limits=(None, None)):
        
        self.G = G
        self.Ti = Ti
        self.Td = Td
        
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.min_output, self.max_output = output_limits
    
    def PID_compute(self, error, dt):

        P = self.G * error

        # Terme intégral (protège contre Ti = inf ou 0)
        if self.Ti != 0 and self.Ti != float('inf'):
            self.integral += error * dt
            I = (self.G / self.Ti) * self.integral
        else:
            I = 0.0

        # Terme dérivé
        D = self.G * self.Td * (error - self.prev_error) / dt
        self.prev_error = error

        output = P + I + D

        # Saturation
        if self.max_output is not None:
            output = min(output, self.max_output)
        if self.min_output is not None:
            output = max(output, self.min_output)

        return output
        

#--------------------------------------------------------------------------------------------
class Quad2D_physics:
    """
    Classe permettant de résoudre la physique du problème en incluant les collisions
    toutes les distances sont mesurée en pxl.
    
    On doit définir une échelle pour s'y retrouver.
    Le drone fait 60 pxl de large donc on peut dire pour simplifier qu'il fait 60 cm donc 0.6 m ==> 60 = 0.6 => alpha = 0.01 (1 pxl = 0.01 m = 1 cm)
    
    Cette classe doit permettre l'exécusion de la simulation sans GUI
    
    Ajout de la trainée: pour l'instant ça empêche le décollage
    """
    def __init__(self, Dt):        
        self.debug = False
        
        ## Simulation params     
        
        self.Dt = Dt
        self.DThrottle = 10
        self.Dangle = 5
        
        self.XMax = 700 # zone de jeu (donc 7 m)
        self.YMax = 900   
                
        # Saturations
        self.Tmax = 1000 # 10 N = 1 kgf
        self.vMax = 500 # 5 m/s
        self.vLandMax = 30 # 0.3 m/s
        self.OMax = m.pi/2 # 45 deg/s
        self.Yfloor = self.YMax-10
        
        ## Quad Physics
        self.l = 30 # 30 cm / bras de levier du quad
        self.hquad = 20
        self.m = 1 
        self.g = .5e3 # 5 m/s2 (au lieu de 9.81)
        self.Cx = .5 #coeff drag
        
        self.ltarget = 25
        self.htarget = 40
        
        
        
        
    def Initialize_Target(self):
        self.xtarget = random.randrange(self.ltarget, self.XMax-self.ltarget)
        self.ytarget = random.randrange(self.htarget, self.XMax-self.htarget)
        
        self.target_reached = self.check_targetreach()
    
    def Initialize_Quad(self):        
        self.Flying = False
        self.Crashed = False
        self.FlyAway = False
        
        self.t = 0
        ## Quad params
            #angle
        self.theta = 0
        self.omega = 0 #vitesse angulaire
            #position
        self.x = 350 #cm
        self.y = self.Yfloor
            #velocity
        self.vx = 0
        self.vy = 0
            #Throttle left and right
        self.Tl = 0
        self.Tr = 0
        
        self.Initialize_Target()
    
    def timeIncrement(self):
        """
        Gestionnaire de l'évolution temporelle des trois variables (x,y,theta)
        theta doit être résolu en 1er car les autres variables dépendent de lui (mais on pourrait utiliser le theta précédent?)
        
        """
        self.t += self.Dt
        
        self.theta_resolution()
        self.X_resolution()
        self.Y_resolution()
        
        self.flyDetection()
        
        self.FlyAway = self.flyAwayDetection()
        
        self.target_reached = self.check_targetreach()
        if self.target_reached:
            self.Initialize_Target()
        
        if self.debug:
            print(f"---------t : {self.t}-----------")
            print(f"Tr : {self.Tr} , Tl : {self.Tl}")
            print(f"theta : {self.theta} , omega : {self.omega}")
            print(f"x : {self.x} , vx : {self.vx}")
            print(f"y : {self.y} , vy : {self.vy}")
        
    def theta_resolution(self):
        """
        l'angle du Quad est géré par l'équation du moment.
        Attention il y a une erreur dans l'article de M.Sajus : "l" est au dénominateur (sinon l'équation est inhomogène)
        
                                        theta\ddot = omega\dot = (Tr-Tl)/(m*l) [et non (Tr-Tl)l/m]
                                        
        Méthode de décomposition d'Euler x\dot = (xt+1 -xt)/Dt ==> xt+1 = xt + v*Dt
        """
        
        self.omega += self.Dt*(self.Tr - self.Tl)/(self.m * self.l)
        if self.omega > self.OMax:
            self.omega = self.OMax
        elif self.omega < -self.OMax:
            self.omega = -self.OMax
        
        self.theta += self.omega*self.Dt
        
        
    def sign(self,a):
        if a != 0:
            return a/(a**2)**.5
        else:
            return 0
    
    def X_resolution(self):
        """
        Position horizontale du Quad
                                    x\ddot = vx\dot = -(Tr+Tl)*sin(theta)/m
        """
        
        self.vx -= self.Dt*(m.sin(self.theta)*(self.Tr + self.Tl)/(self.m))#-self.sign(self.vx)*.5*self.Cx*self.l**2*self.vx**2)
        
        if self.vx > self.vMax:
            self.vx = self.vMax
        elif self.vx < -self.vMax:
            self.vx = - self.vMax
        
        self.x += self.vx*self.Dt
        
        # bord latéraux cycliques
        
        if self.x > self.XMax:
            self.x -= self.XMax
        elif self.x < 0 :
            self.x += self.XMax
            
        
    def Y_resolution(self):
        """
        Position verticale du Quad
                                    y\ddot = vy\dot = g - (Tr+Tl)*cos(theta)/m + trainée
        la gravité est prise en compte ici
        """
        
        self.vy += self.Dt*(self.g - m.cos(self.theta)*(self.Tr + self.Tl)/(self.m))#-self.sign(self.vy)*.5*self.Cx*self.l**2*self.vy**2)
        if self.vy > self.vMax:
            self.vy = self.vMax
        elif self.vy < -self.vMax:
            self.vy = - self.vMax
        
        self.y += self.vy*self.Dt

    def flyDetection(self):        
        if self.y >= self.Yfloor: 
            self.vy = 0
            self.vx = 0
            self.omega = 0
            self.y = self.Yfloor
            
            if self.Flying:                
                self.Flying = False
                self.Crashed = self.crashDetection()                
#                self.Tl = 0
#                self.Tr = 0
                       
        if (not self.Flying) and (self.y < self.Yfloor) and (self.Tl != 0 or self.Tr != 0):
            print("-- TAKEOFF --")
            self.Flying = True 
        
    def check_targetreach(self):
        
        self.x1_HB = self.x - m.cos(self.theta)*self.l - m.sin(self.theta)*self.hquad/2
        self.y1_HB = self.y - m.sin(self.theta)*self.l - m.cos(self.theta)*self.hquad/2
        self.x2_HB = self.x + m.cos(self.theta)*self.l + m.sin(self.theta)*self.hquad/2
        self.y2_HB = self.y + m.sin(self.theta)*self.l + m.cos(self.theta)*self.hquad/2
        
        Xcond1 = self.x2_HB >= self.xtarget - self.ltarget/2
        Xcond2 = self.x1_HB < self.xtarget + self.ltarget/2
        
        Ycond1 = self.y1_HB < self.ytarget + self.htarget/2
        Ycond2 = self.y2_HB >= self.ytarget - self.htarget/2
                    
        
        if Xcond1 and Xcond2 and Ycond1 and Ycond2 :
            print("++++TARGET REACHED++++")
            return True
        else:
            return False
        
    
    def crashDetection(self):

        if self.vy > self.vLandMax or self.theta > m.pi/2 or self.theta < -m.pi/2 :
            print("--- CRASH ---")
            
            if self.theta > m.pi/2 or self.theta < -m.pi/2 :                
                self.theta = m.pi
            else:
                self.theta = 0
            return True
        else:
            print("--- LANDED ---")
            self.theta = 0
            return False
    
    def flyAwayDetection(self):
        if self.y < 0 :
            print("---- FLYAWAY Detected ++++")
            return True
        if self.FlyAway and self.y > 0 :
            print("---- FLYBACK ---")
            return False


#--------------------------------------------------------------------------------------------
class Quad2D_GUI(tk.Tk):
    
    def __init__(self, physics, hitbox = True):
        tk.Tk.__init__(self)
        
        self.hitbox = hitbox
        
        self.title("Quad2D Simulation / Théo Vialis")
        self.phys = physics
        
        self.Cv = tk.Canvas(self, width = self.phys.XMax, height = self.phys.YMax, background = "lightblue")
        self.Cv.pack()        
                
        self.background_setup()
        self.quad_im_setup()
        self.target_setup()
        
        
    
    def background_setup(self):
        
        self.im_sun = Image.open("./assets/sun.png").resize((100,100))
        self.im_clouds = Image.open("./assets/clouds-moving.png").resize((700,300))
        
        self.Sun_PI = ImageTk.PhotoImage(self.im_sun)
        self.Clouds_PI = ImageTk.PhotoImage(self.im_clouds)
        
        self.sun = self.Cv.create_image(50,50,image = self.Sun_PI)
        self.clouds = self.Cv.create_image(400,250,image = self.Clouds_PI)
        
        self.floor =  self.Cv.create_rectangle(-10, self.phys.YMax-5, self.phys.XMax+10, self.phys.YMax+10, fill ="brown")
        
    
    def target_setup(self):
        self.im_target = Image.open("./assets/red-plain-1.png").resize((self.phys.ltarget,self.phys.htarget))
        self.Target_PI = ImageTk.PhotoImage(self.im_target)
        
        self.target = self.Cv.create_image(self.phys.xtarget,self.phys.ytarget,image = self.Target_PI)
        
    
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
        
        #if self.phys.target_reached :
        self.target_move()
        if self.hitbox:
            self.Cv.coords(self.hitbox_target, self.phys.xtarget - self.phys.ltarget/2, self.phys.ytarget - self.phys.htarget/2, self.phys.xtarget + self.phys.ltarget/2 , self.phys.ytarget + self.phys.htarget/2)
           # self.phys.target_reached = False
        
        self.update()
    


if __name__ == "__main__":
    Quad2D_Master()


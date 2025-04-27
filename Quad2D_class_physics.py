# -*- coding: utf-8 -*-
"""
Created on Sun Apr 27 16:09:43 2025

@author: theov
"""

import math as m
import random

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
        self.DThrottle = 5        
        
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
        self.Tl = 250
        self.Tr = 250       
       
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

    ### TARGET MANAGEMENT ++++++++++++++++++++++++++++++
    
    def Initialize_Target(self):
        self.xtarget = random.randrange(self.ltarget, self.XMax-self.ltarget)
        self.ytarget = random.randrange(self.htarget, self.YMax-self.htarget)
        
        self.target_reached = self.check_targetreach()
    
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
        
    ### TARGET MANAGEMENT ------------------------


#--------------------------------------------------------------------------------------------
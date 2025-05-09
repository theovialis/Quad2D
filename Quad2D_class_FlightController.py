# -*- coding: utf-8 -*-
"""
Created on Sun Apr 27 16:10:10 2025

@author: theov
"""

import math as m

class Quad2D_FlightController:
    """
    
    Classe qui représente le rôle du contrôleur de vol / compagnon pour la simulation
    
    bon réglage pour PID_rate :  Quad2D_PID(G = 5, Ti = 700, Td = 1, output_limits=(-self.phys.Tmax,self.phys.Tmax))
    
    Cette classe gère aussi les inputs de l'utilisateur (comme un FC  qui reçoit des commandes de la RC)
    C'est dans cette classe qu'on gère les modes de vol
    Attention cette classe ne contient pas les outils complexes d'asservissement (ex: PID) / Ces outils ont des classes dédiées
    """
    def __init__(self, physics):
        
        self.phys = physics
        
        self.keypressed = set()
        
        self.MODES = ["ACRO", "STABILIZE", "ALTHOLD", "AUTO"]
        self.curr_mode = 0
                
        self.PID_rate = Quad2D_PID(G = 5, Ti = 700, Td = 1, output_limits=(-self.phys.Tmax,self.phys.Tmax))
        self.PID_angle = Quad2D_PID(G = 5, Ti = 700, Td = 1, output_limits=(-self.phys.OMax,self.phys.OMax))
        self.PID_alt = Quad2D_PID(G = 5, Ti = 700, Td = 1, output_limits=(-self.phys.Tmax,self.phys.Tmax))
        
        self.omega_target = 0 
        self.Domega = 10  # deg/s
        self.Dtheta = 5
        self.Dy = 10 #cm/pxl
        
        self.theta_target_max = 20 #deg



    def ManControl_KP(self,e):
        
        key = e.keysym
        
        if key not in self.keypressed:
            self.keypressed.add(key)
            if key == "Right" :
                self.LeftRight_input(-1)
            elif key == "Left" :                
                self.LeftRight_input(1)
       
            elif key == "Up" :
                self.UpDown_input(1)
                
            elif key == "Down" :
                self.UpDown_input(-1)
                
            elif key == "r" :
                self.phys.Initialize_Quad()
                
            # elif key == "d" :
            #     self.plot_display()
                
            elif key == "m":
                self.mode_change()
                
                
    
    def ManControl_KR(self,e):
        key = e.keysym
        self.keypressed.remove(key)
        if key == "Left" or key == "Right":
            self.LeftRight_release()
            
    
    def mode_change(self):
        self.curr_mode = (self.curr_mode + 1) % 3
        self.theta_target = 0
        self.y_target = self.phys.y
        print(f"++ Changing mode to {self.MODES[self.curr_mode]} ++")
    
    def LeftRight_input(self, sign):
        """
        Methode qui converti les inputs de l'utilisateur en cible pour la quantité qui dépend du mode
        """
        
        if self.curr_mode == 0:
            self.omega_target += sign * m.radians(self.Domega)
            
        elif self.curr_mode == 1:
            self.theta_target += sign * m.radians(self.Dtheta)

    def LeftRight_release(self):
        """
        Methode qui converti les inputs release de l'utilisateur en cible pour la quantité qui dépend du mode
        """
        
        if self.curr_mode == 0:
            self.omega_target = 0
            
        elif self.curr_mode == 1:
            self.theta_target = 0
    
     
    def UpDown_input(self, sign):
        if self.curr_mode < 2:
            if self.phys.Tl > 0 and self.phys.Tl < self.phys.Tmax :
                
                self.phys.Tl += sign*self.phys.DThrottle
                
            if self.phys.Tr > 0 and self.phys.Tr < self.phys.Tmax:
                
                self.phys.Tr += sign*self.phys.DThrottle
                
        if self.curr_mode == 2:
            self.y_target -= sign*self.Dy
    
    
    def rate_control(self):
        """
        Entrée : Vitesse angulaire voulue (omega target)
        Sortie : poussée des moteurs   (Tr et Tl)     Signes opposés pour une accélération angulaire
        """

        self.phys.Tl += self.PID_rate.compute(self.phys.omega - self.omega_target, self.phys.Dt)
        self.phys.Tr -= self.PID_rate.compute(self.phys.omega - self.omega_target, self.phys.Dt)
        
    def angle_control(self):
        """
        Entrée : Angle voulu (theta target)
        Sortie : Vitesse angulaire voule (omega target)     
        """
        self.omega_target -= self.PID_angle.compute(self.phys.theta - self.theta_target, self.phys.Dt)
        
    def altitude_control(self):
        """
        Entrée : Altitude voulue (y) (y dirigé vers le bas)
        Sortie : poussée des moteurs (Tr et Tl)     Même signe pour variation d'altitude
        """
        self.phys.Tl += self.PID_alt.compute(self.phys.y - self.y_target, self.phys.Dt)
        self.phys.Tr += self.PID_alt.compute(self.phys.y - self.y_target, self.phys.Dt)
        


#--------------------------------------------------------------------------------------------

class Quad2D_PID:

    def __init__(self, G=1.0, Ti=float('inf'), Td=0.0, output_limits=(None, None)):
        
        self.G = G
        self.Ti = Ti
        self.Td = Td
        
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.min_output, self.max_output = output_limits
    
    def compute(self, error, dt):

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
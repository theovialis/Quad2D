# -*- coding: utf-8 -*-
"""
Created on Mon Apr 21 09:08:21 2025

@author: Théo

Projet de pilotage autonome d'un drone en 2D en prenant en compte la physique (calculs dynamiques)
inspiré de https://youtu.be/J1hv0MJghag?feature=shared / https://github.com/AlexandreSajus/Quadcopter-AI/blob/main/Reinforcement_Learning_for_the_Control_of_Quadcopters.pdf
(M. Sajus utilise Pygame/ Ici c'est Tkinter Canvas)
"""

import math as m
import time
import matplotlib.pyplot as plt
import Quad2D_class_physics, Quad2D_class_FlightController, Quad2D_class_GUI

#--------------------------------------------------------------------------------------------
class Quad2D_Main:
    """
    Classe qui a le rôle de chef d'orchestre de la simulation
    
    Cette classe contient la boucle temporelle principale (dernière méthode).
    Quand elle est instanciée elle lance la simulation.
    
    Elle gère la cible à atteindre et les collisions
    """
    
    def __init__(self, RealTime = True, GUI = True):
        
        self.RealTime = RealTime
        self.gui = GUI
               
        
        self.Dt = .05 ## Pas de temps physique de la simulation
        
        self.phys = Quad2D_class_physics.Quad2D_physics(self.Dt)
        self.phys.Initialize_Quad()

        
        self.FC = Quad2D_class_FlightController.Quad2D_FlightController(self.phys)        
        
        
        self.size_limit = 1000
        
        self.omega_container = []
        self.theta_container = []
        self.y_container = []
        self.x_container = []
        
        
        
        if self.gui:
            self.GuiClass = Quad2D_class_GUI.Quad2D_GUI(self.phys, self.FC)

        
        self.timeloop()
        
        
    ### Probing Management ++++++++++++++++++++++++++++++

    def record_data(self, data, container):
        
        container.append(data)
        if len(container) > self.size_limit:
            container.pop(-1)
        
    
    def plot_display(self):
        
        plt.figure(1)
        plt.plot(self.omega_container, ".k")
        plt.grid()
        
    ### Probing Management -----------------------------    
    
    
    
    ### QUAD SITUATION DETECTION ---------------
    def crashDetection(self):

        if self.phys.vy > self.phys.vLandMax or self.phys.theta > m.pi/2 or self.phys.theta < -m.pi/2 :
            print("--- CRASH ---")
            
            if self.phys.theta > m.pi/2 or self.phys.theta < -m.pi/2 :                
                self.phys.theta = m.pi
            else:
                self.phys.theta = 0
            return True
        else:
            print("--- LANDED ---")
            self.phys.theta = 0
            return False
    
    def flyAwayDetection(self):
        if self.phys.y < 0 :
            print("---- FLYAWAY Detected ++++")
            return True
        if self.phys.FlyAway and self.phys.y > 0 :
            print("---- FLYBACK ---")
            return False
    
    def flyDetection(self):        
        if self.phys.y >= self.phys.Yfloor: 
            self.phys.vy = 0
            self.phys.vx = 0
            self.phys.omega = 0
            self.phys.y = self.phys.Yfloor
            
            if self.phys.Flying:                
                self.phys.Flying = False
                self.phys.Crashed = self.crashDetection()                
 
                       
        if (not self.phys.Flying) and (self.phys.y < self.phys.Yfloor) and (self.phys.Tl != 0 or self.phys.Tr != 0):
            print("-- TAKEOFF --")
            self.phys.Flying = True 
    
    ### QUAD SITUATION DETECTION ---------------
## ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
    def timeloop(self):
        ### ------- MAIN TIMELOOP --------------------
        while True :
            
            ## 1. Control calculation
            self.FC.rate_control()
            
            ### OUTPUT : modif of Tl & Tr in physics
            
            ## 2. Physics calculation
            self.phys.timeIncrement()            
            ## OUTPUT : new values for x,y,vx,vy,theta,omega
            
            ### 3. Quad situation check (optional)
            self.flyDetection()            
            self.phys.FlyAway = self.flyAwayDetection()
            ### OUTPUT : warnings if quad is out of bands or crashed/landed
            
            ### 4. Chack collision with target
            self.target_reached = self.phys.check_targetreach()
            if self.target_reached:
                self.phys.Initialize_Target()
            ### OUTPUT if collision => target moves to somewhere else
            
            ### 5. Probe/Record useful data for analysis
            self.record_data(self.phys.omega, self.omega_container)
            ### OUTPUT : list filled with wanted data
            
            ### 6. Updating the GUI
            if self.gui and self.RealTime :
                self.GuiClass.gui_update()
            ### OUTPUT : 
            
            if self.RealTime:
                time.sleep(self.Dt)
        ### --------- MAIN TIMELOOP ------------------
## +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    

if __name__ == "__main__":
    Quad2D_Main()


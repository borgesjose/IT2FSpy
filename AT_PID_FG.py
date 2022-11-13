# ================================================================================== #
# title           :AT_PID_FG.py                                                      #
# description     :python automatic pid based in phase and gain margins controller   #
# author          :José Borges                                                       #
# date            :20220105                                                          #
# version         :0.1                                                               #
# notes           :                                                                  #
# python_version  :3.6                                                               #
# ================================================================================== #

#######################################################  
# Universidade Federal do Piauí                       #
# Campus ministro petronio Portela                    #                                       
# Copyright 2021 -José Borges do Carmo Neto-          #
# @author José Brges do Carmo Neto                    #
# @email jose.borges90@hotmail.com                    #
#  -- Script Controlador AT PID FG DIGITAL            #
#  -- Version: 0.1  - 17/04/2021                      #
####################################################### 



""" Automatic PID Controller based on phase and gain margins, simple implementation in the Python Programming Language.
"""

class PID_AT_FG:
    """ PID control using phase and gain margins """

    def __init__(self, Kc,Ti,Td, Tamostra):

        self.Kc = Kc
        self.Ti = Ti
        self.Td = Td

        self.Tamostra = Tamostra


        self.output = 0.0

        self.alpha =  Kc*(1+(Td/Tamostra)+(Tamostra/(2*Ti)))
        self.beta  = -Kc*(1+2*(Td/Tamostra) - (Tamostra/(2*Ti)))
        self.gama  =  Kc*Td/Tamostra


    def update(self, u, y, ref, t):
        
        erro = ref - y;
        self.output = u[t-1] + self.alpha*erro[t] + self.beta*erro[t-1] + self.gama*erro[t-2]
        
        return self.output

    def update_pesos(a,b,c,L,Am):
        self.Kc = (pi/2*Am*L)*b
        self.Ti = self.Kc/((pi/2*Am*L)*c)
        self.Td = ((pi/2*Am*L)*a)/self.Kc
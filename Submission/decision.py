import numpy as np
import time
from random import randint




def isfrontclear(image):
    flag = True
    counter = 0
    for x in range(9,311,1):
        for y in range(128,157,1):
            if 0 < image[y,x][0] < 122:
                if 0 < image[y+3,x][0] < 122 and 0 < image[y-3,x][0] < 122:
                    if 0 < image[y,x+3][0] < 122 and 0 < image[y,x-3][0] < 122:
                        counter += 1
                        if counter > 3:
                            flag = False
                            return flag
    return flag

def isrightclear(image):
    flag = True
    counter = 0
    for x in range(312,317,1):
        for y in range(128,157,1):
            if 0 < image[y,x][0] < 122:
                if 0 < image[y+3,x][0] < 122 and 0 < image[y-3,x][0] < 122:
                    if 0 < image[y,x+3][0] < 122 and 0 < image[y,x-3][0] < 122:
                        counter += 1
                        if counter > 3:
                            flag = False
                            return flag
    return flag

def isleftclear(image):
    flag = True
    counter = 0
    for x in range(3,8,1):
        for y in range(128,157,1):
            if 0 < image[y,x][0] < 122:
                if 0 < image[y+3,x][0] < 122 and 0 < image[y-3,x][0] < 122:
                    if 0 < image[y,x+3][0] < 122 and 0 < image[y,x-3][0] < 122:
                        counter += 1
                        if counter > 3:
                            flag = False
                            return flag
    
    return flag

def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status

        
        image = Rover.img
        if Rover.near_sample != 0:
            Rover.mode = 'sample'
            print("Sample Found")
            
        elif isfrontclear(image) == True:
            Rover.mode = 'forward'
            print("Rover Mode = FORWARD")
            Rover.right = False
            Rover.left = False
            
        else:
            Rover.mode = 'stop'
            print("Rover Mode = STOP")
        
        if Rover.mode == 'forward': 
            if Rover.vel < Rover.max_vel:
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set
                
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -2, 2)
            else:
                Rover.brake = 0
                Rover.throttle = 0
                Rover.throttle = Rover.throttle_set
                
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -2, 2)
            
            
        elif Rover.mode == 'stop':
            
            
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0 
            
            elif isrightclear(image) == True and Rover.left == False:
                    
                    Rover.brake = 0
                    Rover.steer = -15
                    print("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR = ", isrightclear(image) )
                    Rover.right = True
                
                
            
            elif isleftclear(image) == True and Rover.right == False:
                Rover.brake = 0
                Rover.steer = 15
                print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL", isleftclear(image))
                Rover.left = True
                
            else:
               
                if Rover.right == False:
                    Rover.brake = 0
                    Rover.steer = 15
                else:
                     Rover.brake = 0
                     Rover.steer = -15
                    
                    
        
        else:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.send_pickup = True
            print("Picking Up sample")
            time.sleep(30)
            Rover.send_pickup = False
            if Rover.near_sample != 0:
                Rover.brake = 0
                Rover.steer = -5
                
            
            
              
              
           
    return Rover
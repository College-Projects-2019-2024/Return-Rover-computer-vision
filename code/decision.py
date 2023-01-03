import numpy as np
from scipy.optimize import curve_fit
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
import math 


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

def cost(peak, freq, xo, yo, yaw, vis):
    
    c=0
    for r in range (12):
        x= r *np.sin(yaw)
        x = round(x)
        y = r * np.cos(yaw)
        y = round (y)

        
        for t in range (-1,2):
            for r in range (-1,2):
                jj = t+yo+y
                ii = r+xo+x
                if (jj) > -1  and (ii) > -1 and (jj) < 20  and (ii) <20:
                    c+= vis[jj][ii]


    ans = 1000*c - int(freq[peak]) 

    ##ans = freq[peak]
    return ans

def mapping1(values_x,a,b,c): 
    return a*values_x**2+b*values_x+c 

def Angle (Rover):
  
    '''  args, covar = curve_fit(mapping1,Rover.xpix,Rover.ypix)
    a,b,c = args[0], args[1], args[2] 
    x0 = 5
    x1 = 10
    y0 = mapping1(x0,a,b,c)
    y1 = mapping1(x1,a,b,c)
    a = np.arctan((y1-y0)/(x1-x0))
    Rover.pid.setpoint = (a*180/(2*np.pi))
    print( Rover.pid.setpoint )
    return int (Rover.pid(Rover.steer))
    '''    
    Rover.simplified_freq = np.zeros(Rover.freqSize)

    for x in Rover.nav_angles_processed:
        f= x*360 / (2*np.pi)
        Rover.simplified_freq[round (f/Rover.freqScale)+Rover.freqshift]+=1
        


    simplified_peaks, _ = find_peaks(Rover.simplified_freq , distance=Rover.freqScale,height=np.max(Rover.simplified_freq )/4, threshold = 1)



    if (simplified_peaks.size == 0):
        Rover.pid.setpoint = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) 
        return int (Rover.pid(Rover.steer))

    else:
        ans = simplified_peaks[0]

        xo, yo = Rover.pos
        for p in simplified_peaks:
            if ( cost(p,Rover.simplified_freq,xo,yo,Rover.yaw, Rover.vis) < cost(ans,Rover.simplified_freq,xo,yo,Rover.yaw, Rover.vis)): #cheapest
                ans = p
        
        Rover.pid.setpoint = np.clip((ans-Rover.freqshift)*Rover.freqScale, -15, 15)
        return int (Rover.pid(Rover.steer))

        



def decision_step(Rover):

    print (Rover.mode)


    xx, yy  = Rover.pos
    xx = round (xx/Rover.mapScale)
    yy = round (yy/Rover.mapScale)
    Rover.vis[20-yy][xx] =100
    ##print (Rover.vis)
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!


    #return

    if (Rover.total_time < 0.1):
        Rover.xbase ,Rover.ybase = Rover.pos

    #rock stuff
    if len(Rover.rock_angles) > 0:

        if(Rover.vel > 1.7):
            Rover.brake = Rover.brake_set


        Rover.throttle = 0.2
        Rover.steer = np.clip(np.mean((Rover.rock_angles) * 180/np.pi), -15, 15)

        

    # Example:
    # Check if we have vision data to make decisions with
    elif Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            Rover.pid.auto_mode = True 

            
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward: 
                if (Rover.samples_collected>=5 and math.dist((Rover.xbase, Rover.ybase), Rover.pos)<10 ):
                    Rover.mode = 'end'


                if Rover.vel == 0 and Rover.total_time - Rover.stuck_time > 4.0 and Rover.flag == False and Rover.total_time - Rover.rock_reverse_time >20:
                    # Set mode to "stuck" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode='stuck'
                    Rover.stuck_time = Rover.total_time

                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if (abs(Rover.steer) > 8):
                    if Rover.vel < Rover.max_vel_turning:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0

                else :
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                
                ########Rover.pid.setpoint = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) 
                Rover.steer = Angle(Rover)
                if (Rover.steer >=0 and Rover.thereIsRock==False):
                    Rover.last_steer =1
                else:
                    Rover.last_steer = -1
            # If there's a lack of navigable terrain pixels then go to 'stop' mode

            elif  len(Rover.nav_angles) < Rover.stop_forward or (Rover.simplified_freq[Rover.freqshift]+Rover.simplified_freq[Rover.freqshift+1] +Rover.simplified_freq[Rover.freqshift-1] < 1200) :
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    xxx = 0
                    if (len(Rover.nav_angles) < int (Rover.stop_forward /2)):
                        xxx = 90
                    else:
                        xxx = 45
                    if (cost(0, [0], xx, yy, Rover.yaw+xxx, Rover.vis) - cost(0, [0], xx, yy, Rover.yaw-xxx, Rover.vis) < 250) :
                        Rover.last_steer = +1
                    else:
                        Rover.last_steer = -1

            Rover.flag = False

        
        elif Rover.mode == 'stuck':
            Rover.pid.auto_mode = False 
            
            if Rover.total_time - Rover.stuck_time > 1.5:
                
                Rover.mode = 'stop'
                
            else:
                Rover.throttle = 0
                
                Rover.brake = 0
                
                if Rover.last_steer <= 0 :
                    Rover.pid.setpoint = (-15)
                    Rover.steer = Rover.pid(Rover.steer)
                else:
                    Rover.pid.setpoint = (15)
                    Rover.steer = Rover.pid(Rover.steer)
        


        elif Rover.mode == 'reverse':
            Rover.pid.auto_mode = False 
            if( Rover.total_time - Rover.rock_reverse_time < 14):
                Rover.brake = 0
                Rover.throttle = -0.1

            else:
                Rover.brake = Rover.brake_set
                Rover.flag = True
                Rover.mode = 'forward'
        
        
        elif Rover.mode == 'stop':
            Rover.pid.auto_mode = True 
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    if Rover.last_steer <= 0 :
                        Rover.pid.setpoint = (-15)
                        Rover.steer = Rover.pid(Rover.steer)
                    else:
                        Rover.pid.setpoint = (15)
                        Rover.steer = Rover.pid(Rover.steer)

                    
                     # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle

                    ##Rover.pid.setpoint = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    
                    ##Rover.steer = Rover.pid(Rover.steer)
                    Rover.steer = Angle(Rover)


                    Rover.mode = 'forward'
        elif (Rover.mode == 'end'):
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
     
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    
    if Rover.near_sample and not Rover.picking_up:
            Rover.brake = Rover.brake_set
            Rover.send_pickup = True
            Rover.rock_collected = True
            Rover.rock_reverse_time = Rover.total_time
            Rover.mode = 'reverse'
    
    return Rover


import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

def cost(peak, prefix, xo, yo, yaw, vis):
    
    c=0
    for r in range (10):
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


    ans = prefix[peak] - 1500*c

    ##ans = prefix[peak]
    return ans

def Angle (Rover):
    Rover.simplified_prefix = np.zeros(Rover.prefixSize)

    for x in Rover.nav_angles_processed:
        f= x*360 / (2*np.pi)
        Rover.simplified_prefix[round (f/Rover.prefixScale)+Rover.prefixshift]+=1
        


    simplified_peaks, _ = find_peaks(Rover.simplified_prefix , distance=Rover.prefixScale*1,height=np.max(Rover.simplified_prefix )/4, threshold = 1)



    if (simplified_peaks.size == 0):
        Rover.pid.setpoint = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) 
        return int (Rover.pid(Rover.steer))

    else:
        ans = simplified_peaks[0]
        print((simplified_peaks-Rover.prefixshift)*Rover.prefixScale )
        xo, yo = Rover.pos
        for p in simplified_peaks:
            if ( cost(p,Rover.simplified_prefix,xo,yo,Rover.yaw, Rover.vis) > cost(ans,Rover.simplified_prefix,xo,yo,Rover.yaw, Rover.vis)):
                ans = p
        Rover.pid.setpoint = (ans-Rover.prefixshift)*Rover.prefixScale
        return int (Rover.pid(Rover.steer))




def decision_step(Rover):
    
    xx, yy  = Rover.pos
    xx = round (xx/Rover.mapScale)
    yy = round (yy/Rover.mapScale)
    Rover.vis[20-yy][xx] =1

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
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
                if (Rover.steer >=0):
                    Rover.last_steer =1
                else:
                    Rover.last_steer = -1
            # If there's a lack of navigable terrain pixels then go to 'stop' mode

            elif  len(Rover.nav_angles) < Rover.stop_forward or (Rover.simplified_prefix[Rover.prefixshift]+Rover.simplified_prefix[Rover.prefixshift+1] +Rover.simplified_prefix[Rover.prefixshift-1] < 1200) :
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
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
                    if (Rover.last_steer <=0 ) :
                        Rover.steer = -15
                    else:
                        Rover.steer = 15

                    
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
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover


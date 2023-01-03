import numpy as np
import time

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if Rover.vel <= 0.1 and Rover.total_time - Rover.stuck_time > 6:
                # Set mode to "stuck" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.stuck=True
                Rover.mode="stuck"
                Rover.stuck_time = Rover.total_time
            else:
                Rover.stuck=False

            if(Rover.stuck_pos is not None):
                distanceMoved=np.sqrt( (Rover.stuck_pos[0]-Rover.pos[0])**2 + (Rover.stuck_pos[1]-Rover.pos[1])**2 )
                if(Rover.stuck==True and (distanceMoved <0.5)): #accepted range
                    Rover.stuck_count+=1
                elif(Rover.stuck==True):#stuck again but not in same position
                    Rover.stuck_count=0
            # Check the extent of navigable terrain
           
            
            if len(Rover.nav_angles)>=14000:               
                Rover.steer = np.clip(np.max(Rover.nav_angles * 180/np.pi), 40, 80)

            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                if(np.min(Rover.nav_distrock) !=None):
                    Rover.mode = 'rockFound'
                    
                else:
                # Set steering to average angle clipped to the range +/- 15
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            if(Rover.vel<= 0.1 and Rover.total_time-Rover.stuck_time >15 and Rover.picking==False):
                throttle=0
                Rover.brake=Rover.brake_set
                Rover.stuck_time=Rover.total_time
                Rover.stuck=True
                Rover.mode="stuck"
                Rover.stuck_time = Rover.total_time
            else:
                Rover.stuck=False
            if(Rover.stuck_pos is not None):
                distanceMoved=np.sqrt( (Rover.stuck_pos[0]-Rover.pos[0])**2 + (Rover.stuck_pos[1]-Rover.pos[1])**2 )

                if(Rover.stuck==True and (distanceMoved <0.5)): #accepted range
                    Rover.stuck_count += 1
                elif(Rover.stuck==True):#stuck again but not in same position
                    Rover.stuck_count=0

            
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward and not Rover.near_sample:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    if(np.mean(Rover.nav_angles)<=0.5 and np.mean(Rover.nav_angles)>=-0.5):
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        Rover.mode = 'forward'
                    else:
                        Rover.steer = -10 


                if ((len(Rover.nav_angles) >= Rover.go_forward) and not Rover.near_sample):
                    if(np.mean(Rover.nav_angles)<=0.5 and np.mean(Rover.nav_angles)>=-0.5):
                        # Set throttle back to stored valu
                         Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        Rover.mode = 'forward'
                    else:
                        Rover.steer = -10

        elif Rover.mode =="stuck":

            #in case of staying stuck several times
            if(Rover.stuck_count == 0):
                Rover.stuck_pos=Rover.pos
            #(np.mean(Rover.nav_angles)>=0.5 or np.mean(Rover.nav_angles)<=-0.5) the navigable train is not in the middle
            if(Rover.stuck_count>=2):
                Rover.mode="backward"
            elif (Rover.stuck_count>1 or (np.mean(Rover.nav_angles)>=0.5 or np.mean(Rover.nav_angles)<=-0.5)):
 
                Rover.throttle = 0
                Rover.brake = 0

                Rover.steer = -15
            
      
            #general case
            if(np.mean(Rover.nav_angles)>=0.5 or np.mean(Rover.nav_angles)<=-0.5):
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -10
            # Now we're stopped and we have vision data to see if there's a path forward
            else:
                Rover.throttle = Rover.throttle_set
                # Release the brake to allow turning
                Rover.brake = 0

                if(Rover.mode is not "backward"):
                    Rover.mode="forward" # returns to previous mode

            
        elif Rover.mode == 'rockFound':
            Rover.steer = np.clip(np.mean(Rover.nav_anglesrock * 180/np.pi), -50, 50)

            if(np.min(Rover.nav_distrock)!=None and np.min(Rover.nav_distrock)<25):
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.picking=True
                Rover.mode = 'collectingRock'

            else:
                Rover.mode='forward'
                
        elif Rover.mode == 'collectingRock':
            if ((len(Rover.nav_angles) < Rover.go_forward) and not Rover.near_sample and Rover.picking== True):
                Rover.mode = 'backward'
            elif((len(Rover.nav_angles) >= Rover.go_forward)):
                Rover.picking = False
                Rover.mode = 'forward'
                
        elif Rover.mode == 'backward':
            Rover.throttle = 0
            Rover.brake=0
            Rover.steer = 0
            Rover.throttle = -0.1
            
            if(Rover.vel < -0.3 and Rover.stuck_count >=2 and Rover.picking == False):#entering from stuck
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.brake = 0
                Rover.steer = -15
                Rover.mode = 'forward'
            elif(Rover.vel < -0.3 and Rover.picking == True): #entering from collectingrRock
                Rover.picking = False
                Rover.mode = 'stop'

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


import numpy as np
import time

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    #get staring postion
    print("Mode: ",Rover.mode)
    
    
    if(Rover.total_time is not None and np.round(Rover.total_time, 1)<=0.1):
        Rover.start_pos=Rover.pos
    
    
    
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        
        # Check for Rover.mode status
#Forward
        if Rover.mode == 'forward': 
            
            
            #reset time if collecting rocks 
            if(Rover.picking==True):
                Rover.stuck_time = Rover.total_time
            if Rover.vel <= 0.1 and Rover.total_time - Rover.stuck_time > 6 :
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
            
            if len(Rover.nav_angles)>=12000:               
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
                    if(Rover.vel>=1):
                        Rover.throttle=0
                        Rover.brake=Rover.brake_set/2
                    Rover.mode = 'rockFound'
                #avg obstacle and avg terrain aproximately equal and very near to obstacle  
                elif(Rover.obs_dist is not None and np.min(Rover.obs_dist) <2):
                    
                    Rover.throttle=0
                    Rover.brake=Rover.brake_set
                    
                    Rover.steer= 15 
                    #Rover.mode ="stop"
                else:
                    # Set steering to average angle clipped to the range +/- 15
                    Rover.steer = np.clip(np.median(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

                    #return to original position

            if(Rover.start_pos is not None and Rover.pos is not None):

                Rover.distanceToStart = np.sqrt((Rover.pos[0]-Rover.start_pos[0])**2 + (Rover.pos[1]-Rover.start_pos[1])**2)
                

                # Calculate angle away from vertical for each pixel
                Rover.anglesToStart = np.arctan2((Rover.pos[1]-Rover.start_pos[1]), (Rover.pos[0]-Rover.start_pos[0]))

                if(Rover.distanceToStart is not None):
                    if(Rover.finished and Rover.picking == False):
                        if(Rover.distanceToStart<15): #threshold to say the rover is near start
                            
                            Rover.mode = "goingHome"
                                    

        # If we're already in "stop" mode then make different decisions
#Stop
        elif Rover.mode == 'stop':
            if(Rover.vel<= 0.1 and Rover.total_time-Rover.stuck_time >15 and Rover.picking==False):
                throttle=0
                Rover.brake=Rover.brake_set
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
                    #Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!

                if ((len(Rover.nav_angles) >= Rover.go_forward) and not Rover.near_sample):
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
                #if ((len(Rover.nav_angles) < Rover.go_forward) and not Rover.near_sample and Rover.picking== True):
                
#Stuck
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
            
            
                
            
#rockFound                    
        elif Rover.mode == 'rockFound':
            #Rover.stuck_time = Rover.total_time
            Rover.steer = np.clip(np.mean(Rover.nav_anglesrock * 180/np.pi), -50, 50)
            throttle=0
            if(np.min(Rover.nav_distrock) is not None and np.min(Rover.nav_distrock)<25):
                Rover.throttle = 0
                # Set brake to stored brake value
                if(Rover.near_sample):
                    Rover.brake = Rover.brake_set
                else:
                    Rover.brake =0
                Rover.steer = 0
                Rover.picking=True
                Rover.mode = 'collectingRock'
            
            #elif(np.min(Rover.nav_distrock) is None and (len(Rover.nav_angles) < Rover.go_forward)):

                #Rover.mode='backward'

            else:
                
                Rover.mode='forward'

#collectingRock                
        elif Rover.mode == 'collectingRock':
            #Rover.stuck_time = Rover.total_time
            if ((len(Rover.nav_angles) < Rover.go_forward) and not Rover.near_sample and Rover.picking== True):
                print("Rover.picking in collecting rocks: ", Rover.picking)
                Rover.mode = 'backward'
            elif((len(Rover.nav_angles) >= Rover.go_forward)):
                Rover.picking = False
                Rover.mode = 'forward'
#Backward
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
            elif(Rover.total_time - Rover.stuck_time > 5):  #if stuck in backwards for more than 5 secs
                Rover.mode="forward"
                Rover.stuck_time = Rover.total_time
                
            
#goingHome        
        elif Rover.mode == "goingHome":
            Rover.steer = (Rover.anglesToStart * 180/np.pi)
            if(Rover.distanceToStart<5): #reached desitanation
                Rover.mode="finished"            
            else:
                Rover.mode = "forward"
            
#finished        
        elif Rover.mode =="finished":
            
            Rover.throttle =0
            Rover.brake=Rover.brake_set
            Rover.steer=0
            
                

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
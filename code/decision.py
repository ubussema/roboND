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
    #print("NavAhead: "+str(Rover.navAhead))
    #print("wall_left: "+str(Rover.wall_left))
    print("mode: "+str(Rover.mode)+" ub_state: "+str(Rover.ub_state))
    #print("Rover.max_vel: "+str(Rover.max_vel))

    if Rover.nav_angles is not None:
        Rover.max_vel=Rover.navAhead*Rover.highestVel/Rover.longestSight
        
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if Rover.navAhead >= Rover.stop_dist:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                #old. Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                
                # steering
				# 2point controller to describe a wall crawler for the left wall.
                if Rover.wall_left > Rover.maxWallDist: #too far from wall
                    Rover.steer = +15
                elif Rover.wall_left < Rover.minWallDist: #too close to wall
                    Rover.steer = -15
                elif Rover.wall_left == 0 and Rover.wall_right==0:
                    Rover.mode = 'stop' #no navigable terrain at sense distance
                else:
                    Rover.steer = 0 # distance is fine.

                  
                    
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            #old. elif len(Rover.nav_angles) < Rover.stop_forward:
            elif Rover.navAhead < Rover.stop_dist:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    
            # a not very beautiful override of the state machine in case the rover is blocked.
            if Rover.sd_state == 'stuck':
                Rover.mode = 'unblocking'
        

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
                #old. if len(Rover.nav_angles) < Rover.go_forward:
                if Rover.navAhead < Rover.go_dist:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if Rover.navAhead >= Rover.go_dist:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'

            # a not very beautiful override of the state machine in case the rover is blocked.
            if Rover.sd_state == 'stuck':
                Rover.mode = 'unblocking'
                    
        elif Rover.mode == 'findwall':
            Rover.throttle = Rover.throttle_set
            Rover.brake = 0
            
            
            
            #steering around obstacles at the right hand side.
            if Rover.wall_right < Rover.minWallDist: #too close to wall
                Rover.mode = 'stop'
                #Rover.steer = 15
            elif Rover.wall_left < Rover.startWallFollow:
                Rover.mode = 'forward'
            elif Rover.navAhead <= Rover.stop_dist:
                Rover.mode = 'stop'
            elif Rover.wall_left == 0 and Rover.wall_right==0:
                Rover.mode = 'stop' #no navigable terrain at sense distance
            else:
                Rover.steer = 0 # distance is fine.
                Rover.mode = 'findwall'

            # a not very beautiful override of the state machine in case the rover is blocked.
            if Rover.sd_state == 'stuck':
                Rover.mode = 'unblocking'
                        
                        
        elif Rover.mode == 'unblocking':
            # when unblocking, the rover is standing still (that's the condition to get into this state)
            Rover.brake = 0
            
            #state decisions for unblocking mode.
            if Rover.sd_state == 'stuck':
                if Rover.ub_state == 'init':
                    Rover.ub_yaw = Rover.yaw
                    Rover.ub_state = 'turn'
                    Rover.ub_timer = time.time()
                elif Rover.ub_state == 'turn':
                    Rover.throttle = 0
                    Rover.brake = 1
                    Rover.brake = 0
                    Rover.steer = -Rover.ub_minYaw
                    #state decisions for 'turn' mode.
                    #yawDelta = Rover.ub_yaw-Rover.yaw+360 if Rover.ub_yaw < Rover.yaw else Rover.ub_yaw-Rover.yaw
                    #yawDelta = Rover.yaw-Rover.ub_yaw if Rover.yaw > Rover.ub_yaw else 
                    #print("yawDelta: "+str(yawDelta)+" Rover.ub_minYaw: "+str(Rover.ub_minYaw)+" Rover.ub_yaw: "+str(Rover.ub_yaw)+" Rover.yaw:"+str(Rover.yaw))
                    #if yawDelta > Rover.ub_minYaw:
                    if (time.time() - Rover.ub_timer) > Rover.ub_turn_time:
                        Rover.ub_timer = time.time() #timer set.
                        Rover.ub_state = 'go'
                    else:
                        Rover.ub_state = 'turn'
                elif Rover.ub_state == 'go':
                    Rover.steer = 0
                    Rover.throttle = Rover.throttle_set
                    if (time.time() - Rover.ub_timer) > Rover.ub_time:
                        Rover.ub_state = 'turn'
                        Rover.ub_yaw = Rover.yaw
                        Rover.ub_timer = time.time()
                    else:
                        Rover.ub_state = 'go'

                Rover.mode = 'unblocking'
            else:
                Rover.mode = 'forward'
                Rover.ub_state = 'init'
            
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


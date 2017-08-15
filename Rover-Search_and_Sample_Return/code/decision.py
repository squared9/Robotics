import numpy as np
import random
import perception

max_cycle_length = 200
cycle_steps = 100
slow_speed = 0.2
max_slow_frames = 100
max_turning_frames = 120


collected_sample_positions = []


def is_collected(x, y):
    for i in range(len(collected_sample_positions)):
        cx = collected_sample_positions[i][0]
        cy = collected_sample_positions[i][1]
        # print("cx=",cx,"cy=",cy,"x=",x,"y=",y)
        dx = cx - x
        dy = cy - y
        if np.sqrt(dx * dx + dy * dy) < 3:
            return True
    return False


def nearest_sample(Rover):
    x = Rover.pos[0]
    y = Rover.pos[1]
    samples_y, samples_x = Rover.worldmap[:, :, 1].nonzero()
    min_dist = 1E7
    ix = None
    for i in range(len(samples_x)):
        if is_collected(samples_x[i], samples_y[i]):
            continue
        dx = x - samples_x[i]
        dy = y - samples_y[i]
        dist = dx * dx + dy * dy
        if dist < min_dist:
            min_dist = dist
            ix = i
    if ix is None:
        return None, None, None, None
    else:
        angle = np.arctan2(samples_y[ix] - y, samples_x[ix] - x) / np.pi * 180
        if angle < 0:
            angle += 360
        return samples_x[ix], samples_y[ix], min_dist, angle


def angular_distance(rho, phi):
    delta = abs(phi - rho) % 360
    result = delta
    if delta > 180:
        result = delta - 360
    return result


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    # If in a state where want to pickup a rock send pickup command

    Rover.sample_angular_distance = None
    sx, sy, sample_dist, sample_angle = nearest_sample(Rover)
    if sx is not None:
        # print("### sample x=", sx, "y=", sy, "dist=", sample_dist, "angle=", sample_angle, "px=", Rover.pos[0], "py=", Rover.pos[1], "yaw=", Rover.yaw, "ad=", angular_distance(Rover.yaw, sample_angle))
        Rover.sample_angle = sample_angle

    if Rover.mode == "stop_slow":
        print("*** Too slow *** Recovery in", Rover.steps_to_follow)
        if Rover.steps_to_follow <= 0:
            Rover.mode = "forward"
            Rover.continuous_slowness = 0
            return Rover
        Rover.brake = 0
        Rover.throttle = 0
        Rover.steer += 5.0 * Rover.recovery_direction
        Rover.steps_to_follow -= 1
        return Rover

    if Rover.continuous_slowness > max_slow_frames:
        print("*** Too slow too long ***")
        Rover.mode = "stop_slow"
        Rover.brake = 0
        Rover.throttle = 0
        if random.random() < 0.5:
            Rover.recovery_direction = -1.0
        else:
            Rover.recovery_direction = 1.0
        Rover.steer += 5.0 * Rover.recovery_direction
        Rover.steps_to_follow = int (random.random() * max_turning_frames)

        return Rover

    if Rover.mode == 'force_forward':
        print("*** Recovering from a cycle *** To go:", Rover.steps_to_follow)
        if len(Rover.nav_angles) < Rover.stop_forward:
            Rover.mode = "stop"
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
        elif Rover.steps_to_follow == 0:
            Rover.mode = "forward"
        else:
            Rover.steps_to_follow -= 1;
        return Rover

    if Rover.near_sample and Rover.vel > 0:
        print("*** Rover approaching sample *** Stopping ***")
        Rover.brake = Rover.brake_set
        Rover.throttle = 0
        return Rover

    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        print("*** Rover near sample *** Standing still ***")
        Rover.send_pickup = True
        if sx is not None:
            collected_sample_positions.append([sx, sy])
        return Rover

    if Rover.steer < 0:
        Rover.continuous_left += 1
        Rover.continuous_right = 0
    elif Rover.steer > 0:
        Rover.continuous_right += 1
        Rover.continuous_left = 0
    else:
        Rover.continuous_left = 0
        Rover.continuous_right = 0

    if Rover.continuous_left > max_cycle_length or Rover.continuous_right > max_cycle_length:
        Rover.mode = 'force_forward'
        Rover.steer = 0
        Rover.throttle = Rover.throttle_set
        Rover.steps_to_follow = cycle_steps
        return Rover

    if Rover.picking_up:
        pass

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check if rover goes too slow
            if Rover.vel <= slow_speed and not Rover.near_sample:
                Rover.continuous_slowness += 1
            else:
                Rover.continuous_slowness = 0
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                if sample_dist is None:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                else:
                    # push angle towards nearest sample exponentially
                    steering_angle = np.mean(Rover.nav_angles * 180/np.pi)
                    adist = angular_distance(Rover.yaw, sample_angle)
                    Rover.sample_angular_distance = adist
                    mixer = np.exp(-sample_dist / 10)
                    Rover.steer = np.clip((1 - mixer) * steering_angle - mixer * adist, -15, 15)
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
            Rover.continuous_slowness = 0
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
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    return Rover


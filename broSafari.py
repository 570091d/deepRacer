import math

def reward_function(params):
    '''
    Example of rewarding the agent to stay inside two borders
    and penalizing getting too close to the objects in front
    '''

    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    objects_distance = params['objects_distance']
    _, next_object_index = params['closest_objects']
    objects_left_of_center = params['objects_left_of_center']
    is_left_of_center = params['is_left_of_center']
    waypoints = params['waypoints']
    closestWaypoints = params['closest_waypoints']
    heading = params['heading']
    speed = params['speed']

    # Initialize reward with a small number but not zero
    # because zero means off-track or crashed
    reward = 1e-3

    # Reward if the agent stays inside the two borders of the track
    if all_wheels_on_track and (0.5 * track_width - distance_from_center) >= 0.05:
        reward_lane = 1.0
    else:
        reward_lane = 1e-3

    # Penalize if the agent is too close to the next object
    reward_avoid = 1.0

    # Distance to the next object
    distance_closest_object = objects_distance[next_object_index]
    # Decide if the agent and the next object is on the same lane
    is_same_lane = objects_left_of_center[next_object_index] == is_left_of_center

    if is_same_lane:
        if 0.5 <= distance_closest_object < 0.8: 
            reward_avoid *= 0.5
        elif 0.3 <= distance_closest_object < 0.5:
            reward_avoid *= 0.2
        elif distance_closest_object < 0.3:
            reward_avoid = 1e-3 # Likely crashed

    # Calculate reward by putting different weights on 
    # the two aspects above
    reward += 1.0 * reward_lane + 4.0 * reward_avoid
    
    # calculate centerline based on the closest waypoints
    nextPoint = waypoints[closestWaypoints[1]]
    prevPoint = waypoints[closestWaypoints[0]]
    
    #calculate the direction in radius, arctan2(dy, dx), the reslut is (-pi, pi) in radians
    trackDirection = math.atan2(nextPoint[1] - prevPoint[1], nextPoint[0] - prevPoint[0])
    #convert to degree
    trackDirection = math.degrees(trackDirection)
    #calculate the difference between thre track direction and the heading direction
    directionDiff = abs(trackDirection - heading)
    if directionDiff > 180:
        directionDiff = 360 - directionDiff
    #the stick
    directionThreshold = 10.0
    if directionDiff > directionThreshold:
        reward *= 0.5

    return reward
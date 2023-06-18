import math
"""
def phase_position(waypoint_position,phases_number):
# Given the waypoint_position (coordinates of waypoint) and the phases number,
# this function finds all positions of the phases around own waypoint.

    # vector of phases coordinates
    phase_position = []
    # distance between phase and waypoint
    radius =  0.5
    delta_angle = 45
    for i in range(phases_number):
        angle = delta_angle * i
        x = waypoint_position[0] + radius * math.cos(angle)
        y = waypoint_position[1] + radius * math.sin(angle)
        phase = [x,y]
        phase_position.append(phase)
    return phase_position

phase_position = phase_position([0,0],3)
print(phase_position[0][0])
"""
def get_phase_position(x0,y0,phases_number):
    # list of 2D vectors: Given the waypoint_position (coordinates x0,y0 of waypoint) and the phases number,
    # this function finds all positions of the phases around own waypoint.
        # vector of phases coordinates
        phase_position = []
        # distance between phase and waypoint
        radius =  0.5
        # angle between two phases
        delta_angle = 45
        for i in range(phases_number):
            angle = delta_angle * i
            x = x0 + radius * math.cos(angle)
            y = y0 + radius * math.sin(angle)
            phase = [x,y]
            phase_position.append(phase)
        return phase_position

phase_position = get_phase_position(0,0,3)
print(phase_position[1][0])
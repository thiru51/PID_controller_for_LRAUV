import numpy as np

"""
Rudder control module for vessel simulation.

This module provides different rudder control strategies for vessel steering:
"""

def ssa(ang, deg=False):
    """
    Smallest signed angle that lies between -pi and pi.
    If deg is True, the angle is assumed to be in degrees and is converted to radians.
    If deg is False, the angle is assumed to be in radians.

    Args:
        ang (float): Angle in degrees or radians
        deg (bool): If True, the angle is assumed to be in degrees

    Returns:
        float: Smallest signed angle between -pi and pi. The angle is in radians if deg is False, and in degrees if deg is True.
    """

    #===========================================================================
    # TODO: Implement the ssa function
    #===========================================================================
    # Write your code here

    if deg is True:
        ang = np.radians(ang)

    SSA = np.arctan2(np.sin(ang), np.cos(ang))
    #===========================================================================

    return np.degrees(SSA) if deg else SSA


def clip(val, min_val, max_val):
    """
    Clip a value to a range.
    """
    return max(min_val, min(val, max_val))


def pid_control(t, state, waypoints, waypoint_idx, ye_int=0.0):
    """
    Implement a PID control strategy to follow the waypoints. 
    x`
    Args:
        t (float): Current simulation time [s]
        state (ndarray): Current vessel state vector
        waypoints (ndarray): Waypoints array
        waypoint_idx (int): Current waypoint index
        ye_int (float): Integral term of cross-track error
        
    Returns:
        float: Commanded rudder angle in radians
        int: Next waypoint index
    """

    # Check if we have reached the last waypoint
    if waypoint_idx == len(waypoints):
        return 0.0, waypoint_idx

    # Current state
    u, v, r = state[3], state[4], state[5]
    x, y, psi = state[6], state[7], state[11]
    
    # Current and previous waypoints
    wp_xn, wp_yn, _ = waypoints[waypoint_idx]
    wp_xn1, wp_yn1, _ = waypoints[waypoint_idx - 1]

    delta_c = 0.0
    ye = 0.0

    
    #===========================================================================
    # TODO: Implement the PID control
    #===========================================================================
    # Write your code here

    #calculate ye
    
    x1, y1, x2, y2, x3, y3 = wp_xn1,wp_yn1, wp_xn,wp_yn, x,y
    vec1 = np.array([x3-x1,y3-y1])
    vec2 = np.array([x2-x1,y2-y1])
    cross_p = np.cross(vec2,vec1)/np.linalg.norm(vec2)
    ye = cross_p
    # outerloop
    kpo , kio = 1.0,0.07

    psi_des = -kpo*ye -kio*ye_int
    r_des = 0.0

    # innerloop
    kpi , kdi = 1.0,1.0

    delta_c = kpi*ssa((psi_des-psi)) + kdi*(r_des-r)
    



    

    #===========================================================================
    
    # Clip the rudder angle
    delta_c = clip(delta_c, -35*np.pi/180, 35*np.pi/180)

    # Distance to waypoint
    wp_dist = np.linalg.norm(np.array([x - wp_xn, y - wp_yn, 0.0]))
    if wp_dist < 0.5:
        waypoint_idx += 1

    # Do not change the following line (the negative sign is due to the sign convention of the rudder angle)
    return -delta_c, ye, waypoint_idx

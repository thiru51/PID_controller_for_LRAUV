import numpy as np
import warnings

def Smat(vec):
    """
    Calculate the skew-symmetric matrix for a given vector
    
    Args:
        vec (array): Vector to be converted to skew-symmetric matrix
        
    Returns:
        array: Skew-symmetric matrix
    """
    S = np.zeros((3,3))
    #===========================================================================
    # TODO: Implement the skew-symmetric matrix
    #===========================================================================
    # Write your code here
    S = np.array([[0, -vec[2], vec[1]], [vec[2], 0, -vec[0]], [-vec[1], vec[0], 0]])
    pass
    #===========================================================================
    return S

def eul_to_rotm(angles):
    """
    Convert Euler angles to rotation matrix
    
    Args:
        angles (array): Euler angles [phi, theta, psi]
        
    Returns:
        array: Rotation matrix
    """
    R = np.zeros((3,3))

    phi = angles[0]
    theta = angles[1]
    psi = angles[2]

    #===========================================================================
    # TODO: Implement the rotation matrix
    #===========================================================================
    # Write your code here
    
    s1 = np.sin(phi)
    s2 = np.sin(theta)
    s3 = np.sin(psi)
    c1 = np.cos(phi)
    c2 = np.cos(theta)
    c3 = np.cos(psi)

    c1,c2,c3 = np.cos(phi), np.cos(theta), np.cos(psi)
    s1,s2,s3 = np.sin(phi), np.sin(theta), np.sin(psi)


    R = np.array([[c2*c3, -c1*s3+s1*s2*c3, s1*s3+c1*s2*c3],
                  [c2*s3, c1*c3+s1*s2*s3, -s1*c3+c1*s2*s3],
                  [-s2, s1*c2, c1*c2]])
    
    #===========================================================================
    return R

# Compute Euler angles from rotation matrix
def rotm_to_eul(rotm, order='ZYX', prev_eul=None, deg=False, silent=True):
    eul = np.zeros(3, dtype=float)
    
    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
    
        theta1 = np.arcsin(-rotm[2, 0])
        if theta1 > 0:
            theta2 = np.pi - theta1
        else:
            theta2 = -np.pi - theta1
        
        if theta1 == np.pi/2:
            phi1 = np.arctan2(rotm[0,1], rotm[1,1])
            psi1 = 0

            phi2 = phi1
            psi2 = psi1        

        elif theta1 == -np.pi/2:
            phi1 = np.arctan2(-rotm[0,1], rotm[1,1])
            psi1 = 0

            phi2 = phi1
            psi2 = psi1

        else:
            phi1 = np.arctan2(rotm[2,1], rotm[2,2])
            phi2 = np.arctan2(-rotm[2,1], -rotm[2,2])

            psi1 = np.arctan2(rotm[1,0], rotm[0,0])
            psi2 = np.arctan2(-rotm[1,0], -rotm[0,0])
        
        eul1 = np.array([phi1, theta1, psi1])
        eul2 = np.array([phi2, theta2, psi2])

        if prev_eul is not None:
            if np.linalg.norm(eul1 - prev_eul) >= np.linalg.norm(eul2 - prev_eul):
                eul = eul1
            else:
                eul = eul2
        else:
            if not silent:
                warnings.warn(f'Both ({eul1[0]*180/np.pi:.2f}, {eul1[1]*180/np.pi:.2f}, {eul1[2]*180/np.pi:.2f}) and ({eul2[0]*180/np.pi:.2f}, {eul2[1]*180/np.pi:.2f}, {eul2[2]*180/np.pi:.2f}) are possible. But the first set is chosen!')
            eul = eul1
        
        if deg:
            eul = eul * 180 / np.pi

    return eul

def eul_rate_matrix(angles):
    """
    Calculate the Euler rate matrix J2
    
    Args:
        angles (array): Euler angles [phi, theta, psi]
        
    Returns:
        array: Euler rate matrix
    """
    J2 = np.zeros((3,3))
    
    phi = angles[0]
    theta = angles[1]
    psi = angles[2]

    #===========================================================================
    # TODO: Implement the Euler rate matrix J2
    #===========================================================================
    # Write your code here
    s1 = np.sin(phi)
    s2 = np.sin(theta)
    s3 = np.sin(psi)
    c1 = np.cos(phi)
    c2 = np.cos(theta)
    c3 = np.cos(psi)

    J2 = np.array([[ 1, s1*s2/c2, c1*s2/c2], [ 0, c1, -s1], [ 0, s1/c2, c1/c2]])

    c1,c2,c3 = np.cos(phi), np.cos(theta), np.cos(psi)
    s1,s2,s3 = np.sin(phi), np.sin(theta), np.sin(psi)

    J2 = np.array([[1, s1*s2/c2, c1*s2/c2],
                  [0, c1, -s1],
                  [0, s1/c2, c1/c2]])
    
    #===========================================================================
    return J2

def eul_to_quat(eul, order='ZYX', deg=False):
    """
    Convert Euler angles to quaternion
    
    Args:
        eul (array): Euler angles [phi, theta, psi]
        
    Returns:
        array: Quaternion
    """
    quat = np.zeros(4, dtype=float)

    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
        
        if deg:
            phi = eul[0] * np.pi / 180
            theta = eul[1] * np.pi / 180
            psi = eul[2] * np.pi / 180
        else:
            phi = eul[0]
            theta = eul[1]
            psi = eul[2]

        quat[0] = np.cos(psi/2) * np.cos(theta/2) * np.cos(phi/2) + np.sin(psi/2) * np.sin(theta/2) * np.sin(phi/2)
        quat[1] = np.cos(psi/2) * np.cos(theta/2) * np.sin(phi/2) - np.sin(psi/2) * np.sin(theta/2) * np.cos(phi/2)
        quat[2] = np.sin(psi/2) * np.cos(theta/2) * np.sin(phi/2) + np.cos(psi/2) * np.sin(theta/2) * np.cos(phi/2)
        quat[3] = np.sin(psi/2) * np.cos(theta/2) * np.cos(phi/2) - np.cos(psi/2) * np.sin(theta/2) * np.sin(phi/2)

        quat = quat / np.linalg.norm(quat)

    return quat

# Compute Euler angles from quaternion
def quat_to_eul(quat, order='ZYX', deg=False, prev_quat=None, silent=True):
    eul = np.zeros(3, dtype=float)
    
    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
        qw = quat[0]
        qx = quat[1]
        qy = quat[2]
        qz = quat[3]

        theta1 = np.arcsin(2 * (qy * qw - qx * qz))
        
        if theta1 > 0:
            theta2 = np.pi - theta1
        else:
            theta2 = -np.pi - theta1
        
        if theta1 == np.pi/2:
            phi1 = np.arctan2(2 * (qx * qy - qz * qw), 1 - 2 * (qx ** 2 + qz ** 2))
            psi1 = 0

            phi2 = phi1
            psi2 = psi1        

        elif theta1 == -np.pi/2:
            phi1 = np.arctan2(-2 * (qx * qy - qz * qw), 1 - 2 * (qx ** 2 + qz ** 2))
            psi1 = 0

            phi2 = phi1
            psi2 = psi1

        else:
            phi1 = np.arctan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2))
            phi2 = np.arctan2(-2 * (qy * qz + qx * qw), -1 + 2 * (qx ** 2 + qy ** 2))

            psi1 = np.arctan2(2 * (qx * qy + qz * qw), 1 - 2 * (qy ** 2 + qz ** 2))
            psi2 = np.arctan2(-2 * (qx * qy + qz * qw), -1 + 2 * (qy ** 2 + qz ** 2))
        
        eul1 = np.array([phi1, theta1, psi1])
        eul2 = np.array([phi2, theta2, psi2])

    if prev_quat is not None:
        if np.linalg.norm(eul1 - quat_to_eul(prev_quat)) >= np.linalg.norm(eul2 - quat_to_eul(prev_quat)):
            eul = eul1
        else:
            eul = eul2
    else:
        if not silent:
            warnings.warn(f'Both ({eul1[0]*180/np.pi:.2f}, {eul1[1]*180/np.pi:.2f}, {eul1[2]*180/np.pi:.2f}) and ({eul2[0]*180/np.pi:.2f}, {eul2[1]*180/np.pi:.2f}, {eul2[2]*180/np.pi:.2f}) are possible. But the first set is chosen!')
        eul = eul1
        
    if deg:
        eul = eul * 180 / np.pi

    return eul

    

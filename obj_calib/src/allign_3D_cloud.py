import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

"""
Functions for:
    - Get centroid point of a pointcloud
    - SVD to get rotation between 2 (registered) pointclouds (barebone ICP)
    - Function to allign a pointcloud given rotation and translation
    - Function to calculate the Euclidean distance between pairs of points (have to be "alligned" (a[0] corresponds to b[0] etc.))
"""

def get_COM(points):
    """
    Input: 
        - pointcloud (array/list of points) 
    Output:
        - single point COM (center of "mass")
    """
    points = np.asfarray(points)
    COM = points.mean(axis = 0)
    return COM

def SVD(ref_points, points_to_allign):
    """
    Considering the corespondancies are known
    only SVD is adequate

    Source for code:
    https://github.com/ClayFlannigan/icp/blob/master/icp.py
    http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/17-icp.pdf
    
    Input:
        - reference point
        - point to allign
    Output:
        - transformation
        - rotation
    """
    # Initialize arrays
    ref_points_new = np.asfarray(ref_points)
    points_to_allign_new = np.asfarray(points_to_allign)
    
    # Substract the COM of each point 
    # in correspoinding point clouds
    # Normalized around (0,0,0)
    ref_norm = ref_points_new - get_COM(ref_points)
    allign_norm = points_to_allign_new - get_COM(points_to_allign)

    # SVD
    H = np.dot(ref_norm.T, allign_norm)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # Translation 
    # first option is described in documents?
    # wrong solution? need to normalize it or something?
    #trans = get_COM(points_to_allign).T - np.dot(R, get_COM(ref_points).T)
    trans  = get_COM(ref_points_new) - get_COM(points_to_allign_new)
    return R, trans

def allign_3D_points(points_to_allign, R, trans):
    """
    Input: 
        - points to allign
        - rotation matrix
        - translation vector (either use form cloud to cloud or from origin to ref)
    Output:
        - points alligned to ref
    """

    # First allign to origin
    points_temp = points_to_allign - get_COM(points_to_allign)
    # Rotate
    points_temp = np.dot(points_temp, R)
    # Move back to original position
    points_temp = points_temp + get_COM(points_to_allign)
    # Use calculated translation
    return points_temp + trans

def get_Err(ref_points, alligned_points):
    """
    Input:
        - 2 (registered) pointclouds
    Output:
        - sum of euclidean distance error
    """
    # Euclidean distance
    return np.sqrt(np.abs(np.sum((ref_points - alligned_points)**2, axis = 1)))

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rot2eul(R) :
    assert(isRotationMatrix(R))     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])    
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])



if __name__ == "__main__":

    points_to_allign =   [[0.380300526741, -0.0509588477927, 0.590257668649],
                        [0.509649026218, -0.180262205769, 0.590266107385],
                        [0.603921164664, -0.0860162912744, 0.59025727482],
                        [0.484784954886, 0.0331148741351, 0.590366715816]]

    points_ref = [[0, 0, 0.0065],
                    [0.1706, 0, 0.0065],
                    [0.1706, 0.2506, 0.0065], 
                    [0, 0.2506, 0.0065]]

    data1 = np.array(points_ref)                                                # ref points
    data2 = np.array(points_to_allign)                                          # to allign points
    R, trans = SVD(data1, data2)
    #Rot_matrix = Rot.from_matrix(R)
    Eul = rot2eul(R)

    print("Rotational matrix:\n{}\n\nEuler angles (zyx):\n{}\n\nTranslation vector:\n{}\n".format(np.around(R, 3), np.around(Eul, 2), np.around(trans, 2)))

    data3 = allign_3D_points(data2, R, trans)                                   # alligned points

    print("Euclidean distance between pairs:\n{}".format(np.around(get_Err(data1, data3), 4)))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.scatter(data1[:,0], data1[:,1], data1[:,2], label = "Reference points")  # ref points
    ax.scatter(data2[:,0], data2[:,1], data2[:,2], label = "Measured points")   # to allign points
    ax.scatter(data3[:,0], data3[:,1], data3[:,2], label = "Alligned points")   # alligned points

    ax.set_title('Pointclouds')
    #ax.set(xlim = (-0.2, 0.5), ylim = (-0.1, 0.5), zlim = (-0.25, 0.25))
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend(loc = 3)
    # ax.set_aspect("equal", "box")
    plt.show()

    print("Done")

"""
[[0.380300526741, -0.0509588477927, 0.590257668649],
[0.509649026218, -0.180262205769, 0.590266107385],
[0.603921164664, -0.0860162912744, 0.59025727482],
[0.484784954886, 0.0331148741351, 0.590366715816]]

[-0.40936392,  0.19633062, -0.58378694]

[[ 6.86660253e-01,  7.26978467e-01, -7.35003635e-05],
[-7.26978409e-01,  6.86660151e-01, -4.80070011e-04],
[-2.98530790e-04,  3.83078173e-04,  9.99999882e-01]]))
"""
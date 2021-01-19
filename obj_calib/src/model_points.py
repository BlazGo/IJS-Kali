import admesh
import numpy as np

def get_points(mesh_path, point_num=4):
    stl = admesh.Stl(mesh_path)
    print("Faces num: {}".format(len(stl)))

    x_max_value = 0
    x_max_if = 0
    x_max_iv = 0

    y_max_value = 0
    y_max_if = 0
    y_max_iv = 0

    z_max_value = 0
    z_max_if = 0
    z_max_iv = 0

    f = 0
    v_list = []
    for facet in stl:
        v = 0
        # get the normal
        facet['normal']
        # walk the vertices
        for vertex in facet['vertex']:
            # read the coordinates
            x = vertex['x']
            y = vertex['y']
            z = vertex['z']
            
            v_list.append([x, y, z])
    
    nv = np.array(v_list)*0.01 # Correct scale

    maxi = np.max(nv, axis=0)
    mini = np.min(nv, axis=0)
    COM = (maxi + mini) /2 # center of model dimension
    COP = np.mean(nv, axis=0) # center of model points
    print("Center of model: {}\nCenter of points: {}".format(COM, COP))

    dist = np.zeros((np.shape(nv)[0],1))
    for i in range(0, np.shape(nv)[0]):
         dist[i] = np.sqrt((COM[0] - nv[i,0])**2 + (COM[1] - nv[i,1])**2)
    
    indexes, temp = np.where(dist == np.max(dist))

    for vertex in indexes:
        #print(nv[vertex,:], vertex)
        pass
    """
    print(np.where(nv[:,0] == maxi[0]))
    print(np.where(nv[:,1] == maxi[1]))
    print(np.where(nv[:,2] == maxi[2]))
    
    print(x_max_value, x_max_if, x_max_iv)
    print(y_max_value, y_max_if, y_max_iv)
    print(z_max_value, z_max_if, z_max_iv)
    """
    return [[0, 0, 0.0065], [0.1706, 0, 0.0065], [0.1706, 0.24, 0.0065], [0, 0.24, 0.0065]]
    
path = "/home/blaz/catkin_ws/src/obj_calib/meshes/ipad7.stl"
get_points(mesh_path = path)
print(0.01253*2)
import admesh

def get_points(mesh_path, point_num=4):
    stl = admesh.Stl(mesh_path)
    print("Vertices num: {}".format(len(stl)))

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
            
            if x > x_max_value:
                x_max_value = x
                x_max_if = f
                x_max_iv = v
                
            elif y > y_max_value:
                y_max_value = x
                y_max_if = f
                y_max_iv = v
                
            elif z > z_max_value:
                z_max_value = x
                z_max_if = f
                z_max_iv = v
                
 
            v += 1   
        f += 1
    print(x_max_value, x_max_if, x_max_iv)
    print(y_max_value, y_max_if, y_max_iv)
    print(z_max_value, z_max_if, z_max_iv)

path = "/home/blaz/catkin_ws/src/obj_calib/meshes/robot_link1.stl"
get_points(mesh_path = path)
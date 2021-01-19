import datetime
import numpy as np
import Tkinter as tk
from Tkinter import Scrollbar
from ttk import Progressbar 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import quaternion_from_euler as eul2qaut

print("[INFO] {} Starting GUI".format(datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S")))

from pose_tracking import poseTrack
from marker_dispay import markerDisplay
from object_positioner import meshDisplay
from allign_3D_cloud import SVD, rot2eul


class UI():
    
    """ Class for UI
    - init draws the UI
    - button functions added as functions
    """

    def __init__(self, points_on_model = [], node_name="obj_calib"):
        
        # Initialize ROS components
        self.ref_model_trans = [-0.5, 0.5, 0]

        self.poseTrack = poseTrack(node_name=node_name)                # point capture
        self.markerDisplay = markerDisplay(node_name=node_name, topic_name="points_array")        # marker display
        self.markerDisplay_model = markerDisplay(node_name=node_name, topic_name="model_points")  # marker model display
        self.meshDisplay = meshDisplay(node_name=node_name)            # Model display
        self.meshDisplay.draw_mesh(pose = self.ref_model_trans)     

        self.root = tk.Tk()
        self.root.geometry("690x420")
        self.root.title("Object calibration")

        point_list = []
        for point in points_on_model:
            temp_point = []
            for i in range(len(point)):
                temp = point[i] + self.ref_model_trans[i]
                temp_point.append(temp)
            point_list.append(temp_point)

        self.points_on_model_og = points_on_model
        self.points_on_model = point_list
        self.progress = 0

        model_point = [self.points_on_model[0]]
        self.markerDisplay_model.draw_markers(model_point)

        lb_cpos_txt = tk.Label(self.root, text = "Current position [m]")
        lb_progress = tk.Label(self.root, text = "Progress (points)")
        self.lb_cpos = tk.Label(self.root, text = "__Position__", font=(None, 15))
        self.lb_time = tk.Label(self.root, text = "__Time__")

        bt_rec_point = tk.Button(self.root, text = "rec point", command = self.frec_point, height = 2, width = 12)
        bt_graph = tk.Button(self.root, text = "graph", command = self.fgraph, height = 2, width = 12)
        bt_calibrate = tk.Button(self.root, text = "calibrate", command = self.fcalibrate, height = 2, width = 12)
        bt_publish = tk.Button(self.root, text = "publish", command = self.fpublish, height = 2, width = 12)
        bt_reset = tk.Button(self.root, text = "reset", command = self.freset, height = 1, width = 10)
        bt_cancel = tk.Button(self.root, text = "cancel", command = self.fcancel, height = 1, width = 10)

        self.txt = tk.Text(self.root, height = 15, width = 35)
        self.progressbar = Progressbar(self.root, orient = tk.HORIZONTAL, length = 250, mode = 'determinate') 
        
        # Positioning
        lb_cpos_txt.grid(row = 0, column = 1, sticky = tk.W)
        lb_progress.grid(row = 7, column = 1, sticky = tk.W)
        self.lb_cpos.grid(row = 1, column = 1, columnspan = 2)
        self.lb_time.grid(row = 9, column = 0, pady = (20,10))
        
        bt_rec_point.grid(row = 2, column = 0, padx = (5, 10))
        bt_graph.grid(row = 3, column = 0, padx = (5, 10))
        bt_calibrate.grid(row = 4, column = 0, padx = (5, 10))
        bt_publish.grid(row = 5, column = 0, padx = (5, 10))
        bt_reset.grid(row = 6, column = 0, padx = (5, 10))
        bt_cancel.grid(row = 8, column = 0, padx = (5, 10))
        
        self.txt.grid(row = 2, column = 1, rowspan = 5, columnspan = 2)
        self.progressbar.grid(row = 8, column = 1, columnspan = 2)

    def mainloop(self):
        self.update_cpos()
        self.update_time()
        self.update_progress()
        #self.update_mesh()
        self.root.mainloop()

    def frec_point(self):
        if len(self.poseTrack.points) >= len(self.points_on_model):
            print("[INFO] No more points needed")
            self.markerDisplay_model.delete_markers()        
            return

        trans = self.poseTrack.record_pose()
        # Recorded points
        self.markerDisplay.delete_markers()
        self.markerDisplay.draw_markers(self.poseTrack.points)

        self.update_progress()

        # Points on marker
        self.markerDisplay_model.delete_markers()        

        # Calculate at which point we are (num model points - (num model points - num rec points))
        # at beginning = 0
        index = len(self.points_on_model) - (len(self.points_on_model) - len(self.poseTrack.points))
        try:
            model_point = [self.points_on_model[index]]
            self.markerDisplay_model.draw_markers(model_point)
        except IndexError:
            pass

        # Write in the console
        self.txt.insert(tk.INSERT, "[INFO] Point recorded.\n")
        self.txt.insert(tk.INSERT, "x: {:.4f} y: {:.4f} z: {:.4f}\n".format(trans[0], trans[1], trans[2]))

        print("[INFO] Point recorded.")
        print("x: {} y: {} z: {}".format(trans[0], trans[1], trans[2]))

    def fgraph(self):
        data = self.poseTrack.points
        print(data)

        # Defining the graph
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = '3d')
        for point in data:
            ax.scatter(point[0], point[1], point[2], label = "Reference points")  # ref points

        ax.set_title('Pointclouds')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()
        
        print("[INFO] Calibrate.")

    def fcalibrate(self):
        points_ref = np.array(self.points_on_model_og)
        #points_allign = np.array(self.poseTrack.points)
        points_allign =   [[0.380300526741, -0.0509588477927, 0.590257668649],
                        [0.509649026218, -0.180262205769, 0.590266107385],
                        [0.603921164664, -0.0860162912744, 0.59025727482],
                        [0.484784954886, 0.0331148741351, 0.590366715816]]
        """
        points_ref = [[0, 0, 0.0065],
                    [0.1706, 0, 0.0065],
                    [0.1706, 0.2506, 0.0065], 
                    [0, 0.2506, 0.0065]]
                                        # to allign points
        
        """
        R, trans = SVD(points_ref, points_allign)
        Eul = rot2eul(R)
        quat = eul2qaut(Eul[0], Eul[1], Eul[2])
        
        deg = []
        for angle in Eul:
            deg.append(round(angle*(180/np.pi), 2))
        trans_t = []
        for pos in trans:
            trans_t.append(round(pos, 2))

        print("Translation: {}\nRotations (zyx):{}".format(trans_t, deg))

        self.meshDisplay = meshDisplay()            # Model display
        self.meshDisplay.draw_mesh(pose = trans, rot = quat)  

        self.txt.insert(tk.INSERT, "[INFO] Calibrate.\n")
        print("[INFO] Calibrate.")

    def fpublish(self):
        self.txt.insert(tk.INSERT, "[INFO] Publish.\n")
        print("[INFO] Publish.")

    def freset(self):
        self.poseTrack.reset_points()
        self.markerDisplay.delete_markers()

        self.txt.delete(1.0,tk.END)
        self.txt.insert(tk.INSERT, "[INFO] Reset points.\n")
        self.update_progress()
        
        model_point = [self.points_on_model[0]]
        self.markerDisplay_model.draw_markers(model_point)
        print("[INFO] Reset points.")

    def fcancel(self):
        self.txt.insert(tk.INSERT, "[INFO] Cancel.\n")
        print("[INFO] Cancel.")
        self.root.destroy()

    def update_cpos(self):
        t, r = self.poseTrack.curr_pose()
        self.lb_cpos.config(text = "x: {:.4f} y: {:.4f} z: {:.4f}".format(t[0], t[1], t[2]))
        self.root.after(100, self.update_cpos)

    def update_time(self):
        now = datetime.datetime.now()
        self.lb_time.config(text = str(now.strftime("%d/%m/%Y %H:%M:%S")))
        self.root.after(200, self.update_time)

    def update_progress(self):
        self.progress = (float(len(self.poseTrack.points)) / float(len(self.points_on_model)))*100
        self.progressbar["value"] = self.progress

    def update_mesh(self):
        self.meshDisplay.draw_mesh(pose=[-0.5, 0.5, 0])
        self.root.after(5000, self.update_mesh)

if __name__ == "__main__":
    from model_points import get_points
    
    points_on_model = [[0, 0, 0.0065], [0.1706, 0, 0.0065], [0.1706, 0.2506, 0.0065], [0, 0.2506, 0.0065]] # test for progressbar need length

    path = "/home/blaz/catkin_ws/src/obj_calib/meshes/ipad6.stl"
    points_on_model = get_points(path)
    print(points_on_model)

    main = UI(points_on_model = points_on_model)
    main.mainloop()
import os
import numpy as np
import linkml_py


def read_vg(path:str):
    points  = np.array([])
    colors  = np.array([])
    normals = np.array([])
    planes = []
    indecies = []



    with open(path, "r") as f:
        num_points = int(f.readline().split(":")[1].strip())
        points = np.fromstring(f.readline(),dtype=np.float32, sep=" ").reshape([-1,3])

        num_colors = int(f.readline().split(":")[1].strip())
        colors = np.fromstring(f.readline(),dtype=np.float32, sep=" ").reshape([-1,3])

        num_normals = int(f.readline().split(":")[1].strip())
        normals = np.fromstring(f.readline(),dtype=np.float32, sep=" ").reshape([-1,3])


        num_groups = int(f.readline().split(" ")[1])

        for idx in range(num_groups):
            group_type = int(f.readline().split(":")[1].strip())
            num_group_parameters = int(f.readline().split(":")[1].strip())
            group_parameters = np.fromstring(f.readline().split(":")[1].strip(),dtype=np.float32, sep=" ")
            group_label = str(f.readline().split(":")[1].strip())
            group_color = np.fromstring(f.readline().split(":")[1].strip(),dtype=np.float32, sep=" ")

            group_num_point = int(f.readline().split(":")[1].strip())
            group_points = np.fromstring(f.readline(),dtype=np.int32, sep=" ")
            num_children = int(f.readline().split(":")[1].strip())

            if (group_type == 0): # Only try to add planes
                planes.append(linkml_py.Plane(group_parameters[0], group_parameters[1], group_parameters[2], group_parameters[3]))
                indecies.append(group_points)

    results = linkml_py.PlaneFittingResults.from_numpy(planes, indecies)

    
    return [points, colors, normals, results]

points, colors, normals, results = read_vg("/home/mephisto/repos/PolyFit/data/PolyFit_data/Fig4f.vg")
# points, colors, normals, results = read_vg("/home/mephisto/repos/PolyFit/data/PolyFit_data/Fig1.vg")

cloud = linkml_py.PointCloud.from_numpy(points, normals, colors)

planes = []
for idx in range(len(results.indecies)):
    planes.append(linkml_py.fit_plane_thorugh_points(cloud, results.indecies[idx]))
plane_results = linkml_py.PlaneFittingResults.from_numpy(planes, results.indecies)
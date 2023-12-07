import os
import numpy as np
import linkml_py

points = np.load("/home/mephisto/server_data/Amtssygehus/points.npy")
normals = np.load("/home/mephisto/server_data/Amtssygehus/normals.npy")
colors = np.load("/home/mephisto/server_data/Amtssygehus/colors.npy")
planes = np.load("/home/mephisto/server_data/Amtssygehus/planes.npy")

points_pp = []
indecies = []
for path, folders, files in os.walk("/home/mephisto/server_data/Amtssygehus/planes/"):

    # Sorting
    keys = [int(name.split(".")[0].split("-")[1]) for name in files]
    files = [x for _, x in sorted(zip(keys, files))]


    for file in files:
        if "points" in file:
            points_pp.append(np.load(os.path.join(path, file)))
        elif "indecies" in file:
            indecies.append(np.load(os.path.join(path, file)))


cloud = linkml_py.PointCloud.from_numpy(points, normals)
plane_results = linkml_py.PlaneFittingResults.from_numpy(planes, indecies)

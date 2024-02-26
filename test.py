import os
import sys
import datetime
# import scipy


print(os.getpid())
print(datetime.datetime.now())


sys.path.insert(
    0, '/home/mephisto/repos/linkml_cpp/build/')

try:
    import linkml_py
    import numpy as np
except Exception as e:
    print(e)
    exit()

# Load data
# import test_data_loaders.VertexGroup as data
# import test_data_loaders.LoadPolyFitRefinedPlanes as data2
# import test_data_loaders.Amtssygehus as data
# import test_data_loaders.Office as data


# print(data.cloud)
# print(data2.plane_results)


# Rifinement Test
# params = linkml_py.Refinement_Parameters()
# params.distance_threshhold = 0.2
# res = linkml_py.refine_planes(data.cloud, data.plane_results, params)


# clusters = linkml_py.clustering(data.cloud, data.plane_results)

# linkml_py.read("/home/mephisto/server_data/stray_scans/0ba33d855b", start=0, n_frames=0, step=5, inference=True)
# linkml_py.read("/home/mephisto/server_data/stray_scans/0ba33d855b", start=1850, n_frames=500, step=5, inference=True)
# linkml_py.read("/home/mephisto/server_data/stray_scans/7092626a22", start=0, n_frames=0, step=5)

# pcd = linkml_py.merge("/home/mephisto/repos/linkml_cpp/clouds", "/home/mephisto/repos/linkml_cpp/merged_cloud.pcd", 1000)
# linkml_py.display("/home/mephisto/repos/linkml_cpp/merged_cloud.pcd")


# pcd = linkml_py.load("/home/mephisto/repos/linkml_cpp/merged_cloud.pcd")
# pcd = linkml_py.filter(pcd)
# linkml_py.save(pcd, "/home/mephisto/repos/linkml_cpp/filtered_cloud.pcd")
# linkml_py.display(pcd)
# linkml_py.display("/home/mephisto/repos/linkml_cpp/filtered_cloud.pcd")


pcd = linkml_py.load("/home/mephisto/repos/linkml_cpp/filtered_cloud.pcd")
# pcd = linkml_py.load("/home/mephisto/repos/linkml_cpp/clouds/cloud_000000.pcd")
pcd = linkml_py.region_growing(pcd,
                         minClusterSize= 100, #int(2*(1/0.02)*(1/0.02)),
                         numberOfNeighbours= 15,
                         smoothnessThreshold = float(3.0 * np.pi / 180.0),
                         curvatureThreshold = 0.001)
linkml_py.display(pcd)

# minClusterSize: 100
# numberOfNeighbours: 15
# smoothnessThreshold: 0.052360
# curvatureThreshold: 0.001000
# Total time: 00:04:40s (Region growing)



print("Done")
# print(clusters)
# linkml_py.create_cell_complex(data.cloud, data.plane_results)
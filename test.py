import os
import sys
import importlib
import scipy


print(os.getpid())


sys.path.insert(
    0, '/home/mephisto/repos/linkml_cpp/build/')

try:
    import linkml_py
    import numpy as np
except Exception as e:
    print(e)
    exit()

# Load data
import test_data_loaders.VertexGroup as data
# import test_data_loaders.LoadPolyFitRefinedPlanes as data2
# import test_data_loaders.Amtssygehus as data
# import test_data_loaders.Office as data


# print(data.cloud)
# print(data2.plane_results)


# Rifinement Test
# params = linkml_py.Refinement_Parameters()
# params.distance_threshhold = 0.2
# res = linkml_py.refine_planes(data.cloud, data.plane_results, params)


clusters = linkml_py.clustering(data.cloud, data.plane_results)
print("Done")
# print(clusters)
# linkml_py.create_cell_complex(data.cloud, data.plane_results)
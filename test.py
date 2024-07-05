import os
import sys
import datetime
# import scipy


print(os.getpid())
print(datetime.datetime.now())


sys.path.insert(
    0, '/home/mephisto/repos/linkml_cpp/build/')

import _core
from _core import *


# Actions
PARSE_DATASET: bool      = False
ANNOTATE: bool          = False
REGISTER: bool          = False
FILTER: bool            = False
MERGE: bool             = False
CLUSTER: bool           = False
DOWNSAMPLE: bool        = False
REGION_GROWING: bool    = False
SOLIDIFY: bool          = True



print("LinkML-Py loaded")



def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

# dataset_path = "/home/mephisto/server_data/stray_scans/0ba33d855b/" # CPC Office
dataset_path = "/home/mephisto/server_data/stray_scans/7092626a22/" # Aarhus Office
# dataset_path = "/home/mephisto/server_data/stray_scans/665518e46a/" # Lobby
# dataset_path = "/home/mephisto/server_data/stray_scans/8c0e3c381c/" # New scan small room
# dataset_path = "/home/mephisto/server_data/stray_scans/3c670b035f" # New scan 

dataset = Dataset(dataset_path)
name = f"./{dataset.name}.pcd"




tmp_folder = f"./{dataset.name}/"

# Parse dataset
if (PARSE_DATASET):
    # Remove temp data
    if not os.path.exists(tmp_folder):
        os.makedirs(tmp_folder)
    else:
        for file in os.listdir(tmp_folder):
            os.remove(os.path.join(tmp_folder, file))
    parse_dataset(dataset, tmp_folder, step=4)

if os.path.exists(tmp_folder):
    clouds = PointCloudsOnDisk(tmp_folder)
    #clouds = PointCloudsInMemory(tmp_folder)

# Annotate
if (ANNOTATE):
    for idx, subset in enumerate(chunks(clouds, 1000)):
        subset.annotate("./yolov8x-seg.onnx", dataset)
        print(f"Annotated {idx+1}th subset of {len(clouds)/1000} subsets")
    
    #clouds.annotate("./yolov8x-seg.onnx", dataset)

# Registration
if (REGISTER):
    clouds.register()

# Filter
if (FILTER):
    clouds.filter()

# Merge
if (MERGE):
    cloud = clouds.merge()
    cloud.save(name)

# cloud.display()

print(f"Loading \"{name}\" ...")
cloud = PointCloud(name)
print("Done loading!")

# Clustering
if (CLUSTER):
    cloud.clustering()
    cloud.save(name)

# Downsample
if (DOWNSAMPLE):
    cloud.downsample(0.05)
    cloud.save(name)

# cloud.display()


# Region growing
if (REGION_GROWING):
    cloud.region_growing(
        #angle_threshold = float(0.96592583),
        #plane_dist_threshold = float(0.1),
        minClusterSize = 500,
        #early_stop = float(0.3),
        radius = float(0.3),
        #interval_0 = float(16),
        #interval_factor = float(1.5),
        )
    cloud.save(name)

# cloud.display()


# Solidify
if (SOLIDIFY):
    breps = cloud.solidify()

    brep_folder = "./Breps/"

    # Remove existing breps
    if not os.path.exists(brep_folder):
        os.makedirs(brep_folder)
    else:
        for file in os.listdir(brep_folder):
            os.remove(os.path.join(brep_folder, file))

    # Save breps
    for idx, brep in enumerate(breps):
        brep.save(brep_folder + f"{dataset.name}_{idx}.off")

cloud.display()


#cProfile.runctx("cloud.solidify()", globals(), locals(), "Profile.prof")
#s = pstats.Stats("Profile.prof")
#s.strip_dirs().sort_stats("time").print_stats()
#cloud.display()

# cloud = PointCloud("/home/mephisto/server_data/test_export.pcd")
# breps = cloud.solidify()
#cloud.display()

# cloud = PointCloud("/home/mephisto/server_data/test_export.pcd")
# [breps[idx].save(f"brep_new_{idx}.off") for idx in range(len(breps))]

# breps = [Brep.load(f"brep_new_{idx}.off") for idx in range(1)]
# for idx, b in enumerate(breps):
#     b.display(f"Brep {idx}")

# files = [f"brep_{idx}.off" for idx in range(4)]
# [Brep.load(f) for f in files]
# Brep.load("test.off")

print("Done")

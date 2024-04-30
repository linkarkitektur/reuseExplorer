import os
import sys
import datetime
# import scipy


print(os.getpid())
print(datetime.datetime.now())


sys.path.insert(
    0, '/home/mephisto/repos/linkml_cpp/build/')

import linkml_py
from linkml_py import *


print("LinkML-Py loaded")



def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

#dataset = Dataset("/home/mephisto/server_data/stray_scans/7092626a22/")



#dataset = Dataset("/home/mephisto/server_data/stray_scans/665518e46a/")
#parse_dataset(dataset, "./clouds/", step=4)

#clouds = PointCloudsOnDisk("./clouds/")
#clouds = PointCloudsInMemory("./one_room/")


#for idx, subset in enumerate(chunks(clouds, 1000)):
#    subset.annotate("./yolov8x-seg.onnx", dataset)
#    print(f"Annotated {idx+1}th subset of {len(clouds)/1000} subsets")

#clouds.annotate("./yolov8x-seg.onnx", dataset)
#clouds.register()


#cloud = clouds.merge().filter().save("./665518e46a.pcd")
#cloud.display()
#cloud.downsample(0.02).save("./665518e46a.pcd")
#cloud.display()

#cloud = cloud.clustering().save("./665518e46a.pcd")
#cloud.region_growing().save("./665518e46a.pcd")
#cloud.display()


#import pstats, cProfile



#path = "./CPH_office_downsampled.pcd"
#path = "./Aarhus_office_downsampled.pcd"
path = "./one_room_downsampled.pcd"
cloud = PointCloud(path)
#cloud.region_growing().save(path)
cloud.solidify()#.display()#.save(path).display()

#cProfile.runctx("cloud.solidify()", globals(), locals(), "Profile.prof")
#s = pstats.Stats("Profile.prof")
#s.strip_dirs().sort_stats("time").print_stats()
#cloud.display()

print("Done")
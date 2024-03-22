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

dataset = Dataset("/home/mephisto/server_data/stray_scans/7092626a22/")
parse_dataset(dataset, "./clouds/", step=4)


clouds = PointCloudsOnDisk("./clouds/")

for idx, subset in enumerate(chunks(clouds, 1000)):
    subset.annotate("./yolov8x-seg.onnx", dataset)
    print(f"Annotated {idx+1}th subset of {len(clouds)/1000} subsets")

clouds.register()

cloud_t = clouds.merge().filter() # <-- There is an issue if a point cloud is freed.
cloud = cloud_t.downsample(0.02).save("./Aarhus_office_downsampled.pcd")

cloud.display()
clouds.display()

print("Done")
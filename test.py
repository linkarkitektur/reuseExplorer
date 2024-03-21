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
#parse_dataset(Dataset("/home/mephisto/server_data/stray_scans/665518e46a", ), "./second/")
#PointCloudsOnDisk("./second/").register() # .merge().filter().save("./second.pcd").display()
# Currently register is returning an invalid pointer.
#PointCloudsOnDisk("./second/").merge().filter().save("./second.pcd").display()

#PointCloud("./merged_cloud.pcd").display()


dataset = Dataset("/home/mephisto/server_data/stray_scans/0ba33d855b/")
parse_dataset(dataset, "./clouds/", step=5)

#PointCloudsInMemory("./one_room/").annotate("./yolov8x-seg.onnx",dataset)

clouds = PointCloudsOnDisk("./clouds/").annotate("./yolov8x-seg.onnx",dataset)
#clouds = PointCloudsOnDisk("./clouds/").register()
#clouds = PointCloudsOnDisk("./clouds/").merge().filter().save("./CPH_office.pcd").display()


#PointCloud("./CPH_office.pcd").display()
#clouds = PointCloudsOnDisk("./clouds/").display()

print("Done")
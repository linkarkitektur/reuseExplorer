import os
import numpy as np
import linkml_py

points = np.load("/home/mephisto/repos/LinkML/Data/points.npy")
normals = np.load("/home/mephisto/repos/LinkML/Data/normals.npy")

# Downsample
# points =  np.concatenate((points[::5000,:],  np.atleast_2d(np.min(points, axis=0)),  np.atleast_2d(np.max(points, axis=0))))
# normals = np.concatenate((normals[::5000,:], np.atleast_2d(np.min(normals, axis=0)), np.atleast_2d(np.max(normals, axis=0))))

cloud = linkml_py.PointCloud.from_numpy(points, normals)

planes = np.load("/home/mephisto/repos/LinkML/Data/planes2.npy")
planes = [ linkml_py.Plane(p[0],p[1],p[2],p[3]) for p in planes.reshape((-1,4))]


cloud.buildIndex()

indecies = []
for p,d,fs in os.walk("/home/mephisto/repos/LinkML/Data/points2"):
    fs.sort()

    for f in fs:
        pts = np.load(os.path.join(p,f))
        l = []
        for pt in pts:
            pos = linkml_py.pos(pt[0],pt[1],pt[2])
            l.append(cloud.radiusSearch(pos, 0.005)[0])

        indecies.append(l)


plane_results = linkml_py.PlaneFittingResults.from_numpy(planes, indecies)
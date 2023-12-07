import os
import numpy as np
import linkml_py

points = np.load("/home/mephisto/repos/LinkML/Data/points.npy")
normals = np.load("/home/mephisto/repos/LinkML/Data/normals.npy")

points =  np.concatenate((points[::5000,:],  np.atleast_2d(np.min(points, axis=0)),  np.atleast_2d(np.max(points, axis=0))))
normals = np.concatenate((normals[::5000,:], np.atleast_2d(np.min(normals, axis=0)), np.atleast_2d(np.max(normals, axis=0))))


planes = np.load("/home/mephisto/repos/LinkML/Data/planes2.npy")
planes = [ linkml_py.Plane(p[0],p[1],p[2],p[3]) for p in planes.reshape((-1,4))]
points = []

for p,d,fs in os.walk("/home/mephisto/repos/LinkML/Notebooks/Plane Fitting/points2"):
    fs.sort()

    for f in fs:
        points.append(np.load(os.path.join(p,f)))
for pts, p in zip(points, planes):
    p._points = pts

# TODO: Need to add indecies, rather then points
# And make a results object


cloud = linkml_py.PointCloud.from_numpy(points, normals)
print(cloud)
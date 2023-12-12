import os
import numpy as np
import linkml_py


def read_planes(path:str):
    planes = []
    indecies = []

    with open(path, "r") as f:
    	# write << "n_groups: " << groups.size() << std::endl;
        n_planes = int(f.readline().split(":")[1].strip())
        
        # for (auto &grp: groups){
        for idx in range(n_planes):
            arr = np.fromstring(f.readline(),dtype=np.float32, sep=" ")
            planes.append(linkml_py.Plane(arr[0],arr[1],arr[2],arr[3]))
            indecies.append(np.array([])) #Just an empy arraz 


    results = linkml_py.PlaneFittingResults.from_numpy(planes, indecies)
    return results

plane_results = read_planes("/home/mephisto/repos/PolyFit/build/bin/planes.txt")
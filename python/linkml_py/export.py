from _core import PointCloud

import numpy as np
import pandas as pd

from numpy.lib.recfunctions import structured_to_unstructured



def toPTS(cloud: PointCloud, path: str):
    data = np.array(cloud)

    xyz = structured_to_unstructured(data[["x","y","z"]])
    rgb = data["rgb"].view("u4").reshape(-1, 1).view(np.uint8)[:,:3]
    # Seems that the result is BGR instead of RGB
    rgb[:,[0,2]] = rgb[:,[2,0]]
    intensity = np.zeros((data.shape[0], 1), dtype=np.uint8)

    # Apparently TwinMotion will have the intensity as the 4th column instad of the 7th
    df = pd.DataFrame(np.concatenate([xyz, intensity, rgb], axis=1), 
                columns=["x", "y", "z", "i", "r", "g", "b"])
    
    df["i"] = df["i"].astype(np.uint8)
    df["r"] = df["r"].astype(np.uint8)
    df["g"] = df["g"].astype(np.uint8)
    df["b"] = df["b"].astype(np.uint8)
    
    with open(path, "w") as f:
        f.write(f"{len(df)}\n")
        df.to_csv(f, index=False, sep=" ", header=False)
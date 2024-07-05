import os
import argparse as arg

from ._core import Dataset, PointCloud, PointCloudsOnDisk, PointCloudsInMemory

def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

def run():

    arg_parser = arg.ArgumentParser(description="Run the CLI")
    arg_parser.add_argument("--verbose", action="store_true", help="Enable verbose output")
    arg_parser.add_argument("--input", type=str, help="Input directory", default="./")
    arg_parser.add_argument("--config", type=str, help="Config file")



    arg_parser.add_argument("--process", default=False, action="store_true", help="Process the input")
    arg_parser.add_argument("--parse-dataset",default=False, action="store_true", help="Parse the dataset")
    arg_parser.add_argument("--annotate",default=False, action="store_true", help="Annotate the dataset")
    arg_parser.add_argument("--register",default=False, action="store_true", help="Register the dataset")
    arg_parser.add_argument("--filter",default=False, action="store_true", help="Filter the dataset")
    arg_parser.add_argument("--merge",default=False, action="store_true", help="Merge the dataset")
    arg_parser.add_argument("--cluster",default=False, action="store_true", help="Cluster the dataset")
    arg_parser.add_argument("--downsample",default=False, action="store_true", help="Downsample the dataset")
    arg_parser.add_argument("--region-growing",default=False, action="store_true", help="Region growing")
    arg_parser.add_argument("--solidify",default=False, action="store_true", help="Solidify the dataset")
    

    args = arg_parser.parse_args()


    print("Running...")

    print(args)


    if not os.path.exists(args.input):
        return


    dataset = Dataset(args.input)
    name = f"./{dataset.name}.pcd"

    print(f"Dataset: {dataset.name}")

    return

    tmp_folder = f"./{dataset.name}/"

    # Parse dataset
    if (args.parse_dataset):
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
    if (args.annotate):
        for idx, subset in enumerate(chunks(clouds, 1000)):
            subset.annotate("./yolov8x-seg.onnx", dataset)
            print(f"Annotated {idx+1}th subset of {len(clouds)/1000} subsets")
        
        #clouds.annotate("./yolov8x-seg.onnx", dataset)

    # Registration
    if (args.register):
        clouds.register()

    # Filter
    if (args.filter):
        clouds.filter()

    cloud = None

    # Merge
    if (args.merge):
        cloud = clouds.merge()
        cloud.save(name)

    # cloud.display()

    if cloud is None:
        print(f"Loading \"{name}\" ...")
        cloud = PointCloud(name)
        print("Done loading!")

    # Clustering
    if (args.cluster):
        cloud.clustering()
        cloud.save(name)

    # Downsample
    if (args.downsample):
        cloud.downsample(0.05)
        cloud.save(name)

    # cloud.display()


    # Region growing
    if (args.region_growing):
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
    if (args.solidify):
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


    print("Done")

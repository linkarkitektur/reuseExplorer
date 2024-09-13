import os
import numpy as np
import argparse as arg
import configparser
from  pathlib import Path
import logging

from ._core import Dataset, PointCloud, PointCloudsOnDisk, Brep, parse_dataset


logger = logging.getLogger(__name__)


def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]






def run():

    arg_parser = arg.ArgumentParser(description="Run the CLI")
    arg_parser.add_argument("--verbose", action="store_true", help="Enable verbose output")
    arg_parser.add_argument("input", type=str, help="Input directory")
    arg_parser.add_argument("--config", type=str, help="Config file")
    arg_parser.add_argument("--show", default=False, action="store_true", help="Show the dataset")


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
    config = configparser.ConfigParser()


    config.read(f"{Path(__file__).parent}/default.ini")

    # Override config with args
    config['STEPS']["PARSE_DATASET"] = "yes" if args.parse_dataset  else "no"
    config['STEPS']["ANNOTATE"] = "yes" if args.annotate else "no" 
    config['STEPS']["REGISTER"] = "yes" if args.register else "no"
    config['STEPS']["FILTER"] = "yes" if args.filter else "no"
    config['STEPS']["MERGE"] = "yes" if args.merge else "no"
    config['STEPS']["CLUSTER"] = "yes" if args.cluster else "no"
    config['STEPS']["DOWNSAMPLE"] = "yes" if args.downsample else "no"
    config['STEPS']["REGION_GROWING"] = "yes" if args.region_growing else "no"
    config['STEPS']["SOLIDIFY"] = "yes" if  args.solidify else "no"

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    logger.debug(f"Args: {args}") # Log args


    # Log config
    logger.debug("Config:")
    for section in config.sections():
        logger.debug(f"{section}: {', '.join([f'{k} = {v}' for k, v in config[section].items()])}")


    if not os.path.exists(args.input):
        logger.error("Input not found")
        return
    
    # Check if input is a directory or a pcd file.
    is_dataset = os.path.isdir(args.input)
    is_cloud = args.input.endswith(".pcd")

    # Add trailing slash to input if it is a directory and missing
    if is_dataset and not args.input.endswith("/"):
        args.input += "/"

    # Check if input is a valid dataset or cloud
    if not is_dataset and not is_cloud:
        logger.error("Invalid input")
        return


    logger.info("Running...")

    dataset = Dataset(args.input) if is_dataset else None
    if dataset != None:
        logger.info(f"Dataset: {dataset.name}")
        logger.debug(f"Dataset: {dataset}")
        name = f"./{dataset.name}.pcd"
    
    cloud = PointCloud(args.input) if is_cloud else None

    has_output_path = True if config['PARSE_DATASET'].get('output_path') and config['PARSE_DATASET'].get('output_path') != "None" else False
    config['PARSE_DATASET']['output_path'] = f"./{dataset.name}/" if has_output_path and is_dataset else config['PARSE_DATASET']['output_path']
    logger.debug(f"Output path: {config['PARSE_DATASET']['output_path']}")


    # Parse dataset
    if (config['STEPS'].getboolean('PARSE_DATASET') and dataset != None):
        logger.info("Parsing dataset...")
        # Remove temp data
        if not os.path.exists(config['PARSE_DATASET']['output_path']):
            os.makedirs(config['PARSE_DATASET']['output_path'])
        else:
            for file in os.listdir(config['PARSE_DATASET']['output_path']):
                os.remove(os.path.join(config['PARSE_DATASET']['output_path'], file))
        
        parse_dataset(
            dataset,
            config['PARSE_DATASET']['output_path'],
            start=int(config['PARSE_DATASET']['start']),
            step=int(config['PARSE_DATASET']['step']),
            stop= None if config['PARSE_DATASET']['stop'] == "None" else int(config['PARSE_DATASET']['stop'])
        )

  
    if os.path.exists(config['PARSE_DATASET']['output_path']) and dataset != None:
        clouds = PointCloudsOnDisk(config['PARSE_DATASET']['output_path'])

    # Annotate
    if (config['STEPS'].getboolean('ANNOTATE') and dataset != None):
        logger.info("Annotating dataset...")
        for idx, subset in enumerate(chunks(clouds, int(config['ANNOTATE']['chunk_size']))):
            subset.annotate(str(Path(config['ANNOTATE']['yolo_path']).resolve()), dataset)
            logger.info(f"Annotated {idx+1}th subset of {len(clouds)/int(config['ANNOTATE']['chunk_size'])} subsets")
    
    # Registration
    if (config['STEPS'].getboolean('REGISTER') and dataset != None):
        logger.info("Registering dataset...")
        clouds.register()

    # Filter
    if (config['STEPS'].getboolean('FILTER') and dataset != None):
        logger.info("Filtering dataset...")
        clouds.filter(int(config['FILTER']['value']))


    # Merge
    config['MERGE']['GROUP_SIZE']
    config['MERGE']['downsample_factor']
    if (config['STEPS'].getboolean('MERGE') and dataset != None):
        logger.info("Merging dataset...")
        cloud = clouds.merge()
        cloud.save(name)


    if cloud is None and is_dataset:

        if not os.path.exists(name):
            logger.error(f"File \"{name}\" not found")
            return
        
        logger.info(f"Loading \"{name}\" ...")
        cloud = PointCloud(name)
        # logger.debug("Done loading!")


    logger.debug(f"Cloud size: {len(cloud)}")

    # Clustering
    if (config['STEPS'].getboolean('CLUSTER') and cloud != None):
        logger.info("Clustering dataset...")
        cloud.clustering(
            cluster_tolerance = float(config['CLUSTER']['cluster_tolerance']),
            min_cluster_size = int(config['CLUSTER']['min_cluster_size']),
            max_cluster_size = int(config['CLUSTER']['max_cluster_size'] if config['CLUSTER']['max_cluster_size'] != "max" else np.iinfo(np.uint32).max )
        )
        cloud.save(name)

    # Downsample
    if (config['STEPS'].getboolean('DOWNSAMPLE') and cloud != None):
        logger.info("Downsampling dataset...")
        cloud.downsample(float(config['DOWNSAMPLE']['leaf_size']))
        cloud.save(name)

    # Region growing
    if (config['STEPS'].getboolean('REGION_GROWING') and cloud != None):
        logger.info("Region growing...")
        cloud.region_growing(
                angle_threshold         = float(config['REGION_GROWING']['angle_threshold']),
                plane_dist_threshold    = float(config['REGION_GROWING']['plane_dist_threshold']),
                minClusterSize          = int(  config['REGION_GROWING']['minClusterSize']),
                early_stop              = float(config['REGION_GROWING']['early_stop']),
                radius                  = float(config['REGION_GROWING']['radius']),
                interval_0              = float(config['REGION_GROWING']['interval_0']),
                interval_factor         = float(config['REGION_GROWING']['interval_factor']))
        cloud.save(name)

    # Solidify
    logger.debug(Path(args.input).stem)
    brep_folder = f"./{dataset.name}{config['SOLIDIFY']['brep_folder_path']}" if is_dataset else f"./{Path(args.input).stem}{config['SOLIDIFY']['brep_folder_path']}"
    if (config['STEPS'].getboolean('SOLIDIFY') and cloud != None):
        logger.info("Solidifying dataset...")
        breps = cloud.solidify(
            downsample_size = int(config['SOLIDIFY']['downsample_size']),
            sx = float(config['SOLIDIFY']['sx']),
            sy = float(config['SOLIDIFY']['sy']),
            expand_factor = float(config['SOLIDIFY']['expand_factor']),
            inflate_factor = float(config['SOLIDIFY']['inflate_factor']),
            max_loop = float(config['SOLIDIFY']['max_loop']),
            mult_factor = float(config['SOLIDIFY']['mult_factor']),
            fitting = float(config['SOLIDIFY']['fitting']),
            coverage = float(config['SOLIDIFY']['coverage']),
            complexity = float(config['SOLIDIFY']['complexity']),
        )

        # Remove existing breps
        if not os.path.exists(brep_folder):
            os.makedirs(brep_folder)
        else:
            for file in os.listdir(brep_folder):
                os.remove(os.path.join(brep_folder, file))

        # Save breps
        for idx, brep in enumerate(breps):
            brep.save(brep_folder + f"{dataset.name}_{idx}.off")

    logger.info("Done")


    if not args.show:
        return
    
    if dataset != None:
        logger.info("Showing dataset...")
        dataset.display(dataset.name, False)

    if os.path.exists(brep_folder):
        for file in os.listdir(brep_folder):
            brep = Brep.load(brep_folder + file)
            brep.display(file, False)

    cloud.display()


    




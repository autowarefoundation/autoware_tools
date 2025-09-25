import json
import os
import sys
import time

from autoware_lanelet2_divider.debug import Debug
from autoware_lanelet2_divider.debug import DebugMessageType
import lanelet2
from lanelet2.projection import UtmProjector
import mgrs
from osgeo import gdal  # cspell: ignore osgeo
from osgeo import ogr  # cspell: ignore osgeo
from tqdm import tqdm
import utm
import yaml


def create_grid_layer(grid_edge_size, layer_grids, mgrs_grid) -> None:
    """
    Create a grid layer of polygons in the specified GDAL layer.

    Parameters:
        grid_edge_size (float): The size of each grid cell.
        layer_grids (ogr.Layer): The layer in which to create the grid polygons.
        mgrs_grid (str): The MGRS grid string used for location projection.

    Returns:
        None
    """
    mgrs_object = mgrs.MGRS()
    zone, hemisphere, origin_y, origin_x = mgrs_object.MGRSToUTM(mgrs_grid)

    grid_count = 100000 / grid_edge_size
    for i in tqdm(
        range(int(grid_count)),
        desc=Debug.get_log("Creating grid layer", DebugMessageType.INFO),
    ):
        for j in range(int(grid_count)):
            feature_grid = ogr.Feature(layer_grids.GetLayerDefn())  # cspell: ignore Defn
            linear_ring = ogr.Geometry(ogr.wkbLinearRing)
            for a in range(5):
                pt_x, pt_y = 0.0, 0.0
                if (a == 0) or (a == 4):
                    pt_x = origin_x + (i * grid_edge_size)
                    pt_y = origin_y + (j * grid_edge_size)
                elif a == 1:
                    pt_x = origin_x + (i * grid_edge_size) + grid_edge_size
                    pt_y = origin_y + (j * grid_edge_size)
                elif a == 2:
                    pt_x = origin_x + (i * grid_edge_size) + grid_edge_size
                    pt_y = origin_y + (j * grid_edge_size) + grid_edge_size
                elif a == 3:
                    pt_x = origin_x + (i * grid_edge_size)
                    pt_y = origin_y + (j * grid_edge_size) + grid_edge_size
                pt_lat, pt_lon = utm.to_latlon(pt_y, pt_x, zone, hemisphere)
                linear_ring.AddPoint_2D(pt_lon, pt_lat)

            polygon = ogr.Geometry(ogr.wkbPolygon)
            polygon.AddGeometry(linear_ring)
            feature_grid.SetGeometry(polygon)
            if layer_grids.CreateFeature(feature_grid) != 0:
                Debug.log("Failed to create feature in shapefile.", DebugMessageType.ERROR)
                sys.exit(1)

            feature_grid.Destroy()


def generate_lanelet2_layer(mgrs_grid, lanelet2_map_path, lanelet2_whole_mls, layer_lanelet2_whole):
    """
    Generate a Lanelet2 layer from the given Lanelet2 map path and adds it to the specified GDAL layer.

    Parameters:
        mgrs_grid (str): The MGRS grid string used for location projection.
        lanelet2_map_path (str): The file path to the Lanelet2 map.
        lanelet2_whole_mls (ogr.Geometry): The MultiLineString geometry to which lanelet geometries will be added.
        layer_lanelet2_whole (ogr.Layer): The layer to which the Lanelet2 features will be added.

    Returns:
        None
    """
    mgrs_object = mgrs.MGRS()
    zone, hemisphere, origin_x, origin_y = mgrs_object.MGRSToUTM(mgrs_grid)
    origin_lat, origin_lon = utm.to_latlon(origin_x, origin_y, zone, hemisphere)

    projector = UtmProjector(lanelet2.io.Origin(origin_lat, origin_lon))
    lanelet2_map, load_errors = lanelet2.io.loadRobust(lanelet2_map_path, projector)

    for lanelet_linestring in lanelet2_map.lineStringLayer:
        linestring = ogr.Geometry(ogr.wkbLineString)
        for node in lanelet_linestring:
            node_lat, node_lon = utm.to_latlon(
                origin_x + node.x, origin_y + node.y, zone, hemisphere
            )
            linestring.AddPoint_2D(node_lon, node_lat)
        lanelet2_whole_mls.AddGeometry(linestring)
    feature_lanelet2 = ogr.Feature(layer_lanelet2_whole.GetLayerDefn())  # cspell: ignore Defn
    feature_lanelet2.SetGeometry(lanelet2_whole_mls)
    if layer_lanelet2_whole.CreateFeature(feature_lanelet2) != 0:
        Debug.log("Failed to create feature in shapefile.", DebugMessageType.ERROR)
        sys.exit(1)
    feature_lanelet2.Destroy()


def generate_yaml_dict(layer_filtered_grids, grid_edge_size, mgrs_grid) -> dict:
    """
    Generate a YAML-compatible dictionary from the filtered grid layer.

    Parameters:
        layer_filtered_grids (ogr.Feature): The layer containing filtered grid features.
        grid_edge_size (float): The size of each grid cell.
        mgrs_grid (str): The MGRS grid string used for location projection.

    Returns:
        dict: A dictionary containing grid metadata for YAML output.
    """
    mgrs_object = mgrs.MGRS()
    zone, hemisphere, origin_x, origin_y = mgrs_object.MGRSToUTM(mgrs_grid)

    metadata_yaml = {}
    for filtered_grid in layer_filtered_grids:
        geometry_filtered_grid = filtered_grid.GetGeometryRef()
        point_lat = 0.0
        point_lon = 0.0
        for linearring in geometry_filtered_grid:
            point_lat = linearring.GetPoint(0)[1]
            point_lon = linearring.GetPoint(0)[0]
        x, y, zone_number, zone_letter = utm.from_latlon(point_lat, point_lon)

        file_id = str(filtered_grid.GetFID()) + ".osm"
        yaml_data = {
            "x_resolution": float(grid_edge_size),
            "y_resolution": float(grid_edge_size),
            file_id: [round(float(x - origin_x), 2), round(float(y - origin_y), 2)],
        }
        metadata_yaml.update(yaml_data)
    return metadata_yaml


def generate_config_json(layer_filtered_grids, extract_dir) -> str:
    """
    Generate a configuration JSON string for Osmium Extract from the filtered grid layer.

    Parameters:
        layer_filtered_grids (ogr.Feature): The layer containing filtered grid features.
        extract_dir (str): The directory where the output files will be saved.

    Returns:
        str: A JSON string containing the configuration for Osmium Extract.
    """
    extracts = []
    for filtered_grid in layer_filtered_grids:
        polygon_inner = []
        geometry_filtered_grid = filtered_grid.GetGeometryRef()
        for linearring in geometry_filtered_grid:
            for i in range(5):
                point_lat = linearring.GetPoint(i)[1]
                point_lon = linearring.GetPoint(i)[0]
                point_list = [point_lon, point_lat]
                polygon_inner.append(point_list)

        polygon_outer = [polygon_inner]
        extract_element = {
            "description": "optional description",
            "output": str(filtered_grid.GetFID()) + ".osm",
            "output_format": "osm",
            "polygon": polygon_outer,
        }
        extracts.append(extract_element)

    config_json_ = {
        "directory": os.path.join(extract_dir, "lanelet2_map.osm"),
        "extracts": extracts,
    }

    return json.dumps(config_json_, indent=2)


def data_preparation(
    mgrs_grid: str, grid_edge_size: int, lanelet2_map_path: str, extract_dir: str
) -> list[str]:
    """
    Prepare the data by creating grid layers, generating Lanelet2 layers, and producing metadata files.

    Parameters:
        mgrs_grid (str): The MGRS grid string used for location projection.
        grid_edge_size (int): The size of each grid cell.
        lanelet2_map_path (str): The file path to the Lanelet2 map.
        extract_dir (str): The directory where the output files will be saved.

    Returns:
        list[str]: A list of paths to the generated configuration JSON files.
    """
    # Create gpkg dataset and layers
    Debug.log("Create output directory if not exist.", DebugMessageType.INFO)
    os.makedirs(extract_dir, exist_ok=True)

    Debug.log("Creating GDAL driver and GPKG layers.", DebugMessageType.INFO)
    driverName = "GPKG"
    drv = gdal.GetDriverByName(driverName)
    if drv is None:
        Debug.log("%s driver not available.\n" % driverName, DebugMessageType.ERROR)
        sys.exit(1)

    ds_grids = drv.Create(
        os.path.join(extract_dir, "output_layers.gpkg"), 0, 0, 0, gdal.GDT_Unknown
    )
    if ds_grids is None:
        Debug.log("Creation of output file failed.", DebugMessageType.ERROR)
        sys.exit(1)

    layer_grids = ds_grids.CreateLayer("grids", None, ogr.wkbPolygon)
    if layer_grids is None:
        Debug.log("Layer creation failed.", DebugMessageType.ERROR)
        sys.exit(1)

    layer_lanelet2_whole = ds_grids.CreateLayer("lanelet2_whole", None, ogr.wkbMultiLineString)
    if layer_lanelet2_whole is None:
        Debug.log("Layer creation failed layer_lanelet2_whole.", DebugMessageType.ERROR)
        sys.exit(1)

    # Create new layer for filtered grids
    layer_filtered_grids = ds_grids.CreateLayer("filtered_grids", None, ogr.wkbPolygon)
    if layer_filtered_grids is None:
        Debug.log("Layer creation failed.", DebugMessageType.ERROR)
        sys.exit(1)

    # Create Grid Layer
    Debug.log(
        "Creating " + str(grid_edge_size) + " meters grid layer along the " + mgrs_grid + ".",
        DebugMessageType.INFO,
    )
    start = time.time()
    create_grid_layer(grid_edge_size, layer_grids, mgrs_grid)
    end = time.time()
    formatted = "{:.1f}".format(end - start)
    Debug.log(
        "Grid layer created. Lasted " + str(formatted) + " seconds.",
        DebugMessageType.SUCCESS,
    )

    # Make a multilinestring in order to use in filtering
    lanelet2_whole_mls = ogr.Geometry(ogr.wkbMultiLineString)

    # Generate the lanelet2_map linestring layer with gpkg for filtering
    generate_lanelet2_layer(mgrs_grid, lanelet2_map_path, lanelet2_whole_mls, layer_lanelet2_whole)

    # Filter and destroy feature
    Debug.log("Filtering the grid layer with input lanelet2 map.", DebugMessageType.INFO)
    layer_grids.SetSpatialFilter(lanelet2_whole_mls)

    # Set filtered grid layer
    for grid in layer_grids:
        geometry_grid = grid.GetGeometryRef()

        filtered_feature_grid = ogr.Feature(
            layer_filtered_grids.GetLayerDefn()
        )  # cspell: ignore Defn
        filtered_feature_grid.SetGeometry(geometry_grid)
        if layer_filtered_grids.CreateFeature(filtered_feature_grid) != 0:
            Debug.log("Failed to create feature in shapefile.", DebugMessageType.ERROR)
            sys.exit(1)

        filtered_feature_grid.Destroy()

    # Create yaml data and write to file
    Debug.log(
        "Generating metadata.yaml for Dynamic Lanelet2 Map Loading.",
        DebugMessageType.INFO,
    )

    metadata_yaml = generate_yaml_dict(layer_filtered_grids, grid_edge_size, mgrs_grid)

    with open(
        os.path.join(extract_dir, "lanelet2_map_metadata.yaml"),
        "w",
    ) as f:
        yaml.dump(metadata_yaml, f, default_flow_style=None, sort_keys=False)

    # Create config.json for Osmium Extract
    Debug.log("Generating config.json for Osmium Extract.", DebugMessageType.INFO)

    config_files = []
    config_name_counter = 1
    total_feature_count = layer_filtered_grids.GetFeatureCount()
    maximum_feature_count = 500

    if total_feature_count > maximum_feature_count:
        fid_list = []
        for i in range(1, total_feature_count + 1):
            fid_list.append(i)  # add fid into fid_list
            if ((i % maximum_feature_count) == 0) or (i == total_feature_count):
                dup_layer_grids = layer_filtered_grids
                dup_layer_grids.SetAttributeFilter("FID IN {}".format(tuple(fid_list)))

                config_json = generate_config_json(dup_layer_grids, extract_dir)
                config_json_name = "config" + str(config_name_counter) + ".json"
                with open(os.path.join(extract_dir, config_json_name), "w") as write_file:
                    write_file.write(config_json)
                    config_files.append(os.path.join(extract_dir, config_json_name))
                config_name_counter += 1
                fid_list.clear()
    else:
        config_json = generate_config_json(layer_filtered_grids, extract_dir)
        config_json_name = "config" + str(config_name_counter) + ".json"
        with open(os.path.join(extract_dir, config_json_name), "w") as write_file:
            write_file.write(config_json)
            config_files.append(os.path.join(extract_dir, config_json_name))

    # return os.path.join(extract_dir, "config.json")
    return config_files

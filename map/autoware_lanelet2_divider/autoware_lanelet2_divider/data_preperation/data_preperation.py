from typing import List
from osgeo import gdal
from osgeo import ogr
from tqdm import tqdm
import lanelet2
from lanelet2.projection import UtmProjector
import sys
import mgrs
import utm
import yaml
import json
import time
import os

from autoware_lanelet2_divider.debug import Debug, DebugMessageType


def create_grid_layer(grid_edge_size, layer_grids, mgrs_grid) -> None:
    mgrs_object = mgrs.MGRS()
    zone, northp, origin_y, origin_x = mgrs_object.MGRSToUTM(mgrs_grid)

    grid_count = 100000 / grid_edge_size
    for i in tqdm(range(int(grid_count)), desc=Debug.get_log("Creating grid layer", DebugMessageType.INFO)):
        for j in range(int(grid_count)):
            feature_grid = ogr.Feature(layer_grids.GetLayerDefn())
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
                pt_lat, pt_lon = utm.to_latlon(pt_y, pt_x, zone, northp)
                linear_ring.AddPoint_2D(pt_lon, pt_lat)

            polygon = ogr.Geometry(ogr.wkbPolygon)
            polygon.AddGeometry(linear_ring)
            feature_grid.SetGeometry(polygon)
            if layer_grids.CreateFeature(feature_grid) != 0:
                print("Failed to create feature in shapefile.\n")
                sys.exit(1)

            feature_grid.Destroy()


def generate_lanelet2_layer(mgrs_grid, lanelet2_map_path, lanelet2_whole_mls, layer_lanelet2_whole):
    mgrs_object = mgrs.MGRS()
    zone, northp, origin_x, origin_y = mgrs_object.MGRSToUTM(mgrs_grid)
    origin_lat, origin_lon = utm.to_latlon(origin_x, origin_y, zone, northp)

    projector = UtmProjector(lanelet2.io.Origin(origin_lat, origin_lon))
    lanelet2_map, load_errors = lanelet2.io.loadRobust(lanelet2_map_path, projector)

    for lanelet_linestring in lanelet2_map.lineStringLayer:
        linestring = ogr.Geometry(ogr.wkbLineString)
        for node in lanelet_linestring:
            node_lat, node_lon = utm.to_latlon(origin_x + node.x, origin_y + node.y, zone, northp)
            linestring.AddPoint_2D( node_lon, node_lat)
        lanelet2_whole_mls.AddGeometry(linestring)
    feature_lanelet2 = ogr.Feature( layer_lanelet2_whole.GetLayerDefn())
    feature_lanelet2.SetGeometry(lanelet2_whole_mls)
    if layer_lanelet2_whole.CreateFeature(feature_lanelet2) != 0:
        print("Failed to create feature in shapefile.\n")
        sys.exit( 1 )
    feature_lanelet2.Destroy()


def generate_yaml_dict(layer_filtered_grids, grid_edge_size, mgrs_grid) -> dict:

    mgrs_object = mgrs.MGRS()
    zone, northp, origin_x, origin_y = mgrs_object.MGRSToUTM(mgrs_grid)

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
            file_id: [round(float(x - origin_x), 2), round(float(y - origin_y), 2)]
        }
        metadata_yaml.update(yaml_data)
    return metadata_yaml


def generate_config_json(layer_filtered_grids, extract_dir) -> str:
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
            "polygon": polygon_outer
        }
        extracts.append(extract_element)

    config_json_ = {
        "directory": os.path.join(extract_dir, "lanelet2_map.osm"),
        "extracts": extracts
    }

    return json.dumps(config_json_, indent=2)


def data_preparation(mgrs_grid, grid_edge_size, lanelet2_map_path, extract_dir) -> list[str]:
    # Create gpkg dataset and layers
    # --------------------------------------------------------------------------------
    Debug.log("Creating GDAL driver and GPKG layers.", DebugMessageType.INFO)
    driverName = "GPKG"
    drv = gdal.GetDriverByName(driverName)
    if drv is None:
        print("%s driver not available.\n" % driverName)
        sys.exit(1)

    ds_grids = drv.Create(os.path.join(extract_dir, "output_layers.gpkg"), 0, 0, 0, gdal.GDT_Unknown)
    if ds_grids is None:
        print("Creation of output file failed.\n")
        sys.exit(1)

    layer_grids = ds_grids.CreateLayer("grids", None, ogr.wkbPolygon)
    if layer_grids is None:
        print("Layer creation failed.\n")
        sys.exit(1)

    layer_lanelet2_whole = ds_grids.CreateLayer("lanelet2_whole", None, ogr.wkbMultiLineString)
    if layer_lanelet2_whole is None:
        print("Layer creation failed layer_lanelet2_whole.\n")
        sys.exit(1)

    # Create new layer for filtered grids
    # --------------------------------------------------------------------------------
    layer_filtered_grids = ds_grids.CreateLayer("filtered_grids", None, ogr.wkbPolygon)
    if layer_filtered_grids is None:
        print("Layer creation failed.\n")
        sys.exit(1)

    # Create Grid Layer
    # --------------------------------------------------------------------------------
    Debug.log("Creating " + str(grid_edge_size) + " meters grid layer along the " + mgrs_grid + ".",
              DebugMessageType.INFO)
    start = time.time()
    create_grid_layer(grid_edge_size, layer_grids, mgrs_grid)
    end = time.time()
    formatted = "{:.1f}".format(end - start)
    Debug.log("Grid layer created. Lasted " + str(formatted) + " seconds.", DebugMessageType.SUCCESS)


    # Make a multilinestring in order to use in filtering
    # --------------------------------------------------------------------------------
    lanelet2_whole_mls = ogr.Geometry(ogr.wkbMultiLineString)

    # Generate the lanelet2_map linestring layer with gpkg for filtering
    # --------------------------------------------------------------------------------
    generate_lanelet2_layer(mgrs_grid, lanelet2_map_path, lanelet2_whole_mls, layer_lanelet2_whole)

    # Filter and destroy feature
    # --------------------------------------------------------------------------------
    Debug.log("Filtering the grid layer with input lanelet2 map.", DebugMessageType.INFO)
    layer_grids.SetSpatialFilter(lanelet2_whole_mls)

    # Set filtered grid layer
    # --------------------------------------------------------------------------------
    for grid in layer_grids:
        geometry_grid = grid.GetGeometryRef()

        filtered_feature_grid = ogr.Feature(layer_filtered_grids.GetLayerDefn())
        filtered_feature_grid.SetGeometry(geometry_grid)
        if layer_filtered_grids.CreateFeature(filtered_feature_grid) != 0:
            print("Failed to create feature in shapefile.\n")
            sys.exit(1)

        filtered_feature_grid.Destroy()

    # Create yaml data and write to file
    # --------------------------------------------------------------------------------
    Debug.log("Generating metadata.yaml for Dynamic Lanelet2 Map Loading.", DebugMessageType.INFO)

    metadata_yaml = generate_yaml_dict(layer_filtered_grids, grid_edge_size, mgrs_grid)

    with open(os.path.join(extract_dir, "lanelet2_map_metadata.yaml"), 'w', ) as f:
        yaml.dump(metadata_yaml, f, default_flow_style=None, sort_keys=False)

    # Create config.json for Osmium Extract
    # --------------------------------------------------------------------------------
    Debug.log("Generating config.json for Osmium Extract.", DebugMessageType.INFO)

    config_files = []
    config_name_counter = 1
    total_feature_count = layer_filtered_grids.GetFeatureCount()

    if total_feature_count > 500:
        fid_list = []
        for i in range(1, total_feature_count+1):
            fid_list.append(i)  # add fid into fid_list
            if ((i % 500) == 0) or (i == total_feature_count):
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

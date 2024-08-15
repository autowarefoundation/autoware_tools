#!/usr/bin/env python3

import os
from pathlib import Path

from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions


def infer_configs(
    bag_uri: str,
    storage_ids={".db3": ("sqlite3", "cdr", "cdr"), ".mcap": ("mcap", "", "")},
) -> tuple:
    bag_uri_path = Path(bag_uri)
    if os.path.isfile(bag_uri):
        data_file = bag_uri_path
    else:
        data_file = next(p for p in bag_uri_path.glob("*") if p.suffix in storage_ids)
        if data_file.suffix not in storage_ids:
            raise ValueError(f"Unsupported storage id: {data_file.suffix}")
    return storage_ids[data_file.suffix]


def create_reader(input_uri: str) -> SequentialReader:
    """Create a reader object from the given input uri. The input uri could be a directory or a file."""
    storage_id, isf, osf = infer_configs(input_uri)
    storage_options = StorageOptions(
        uri=input_uri,
        storage_id=storage_id,
    )
    converter_options = ConverterOptions(
        input_serialization_format=isf,
        output_serialization_format=osf,
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader

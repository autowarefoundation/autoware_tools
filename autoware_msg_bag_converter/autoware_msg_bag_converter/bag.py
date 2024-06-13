# Copyright (c) 2024 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions


def get_default_converter_options() -> ConverterOptions:
    return ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )


def get_default_storage_options(uri: str) -> StorageOptions:
    return StorageOptions(
        uri=uri,
        storage_id="sqlite3",
    )


def create_reader(bag_dir: str) -> SequentialReader:
    storage_options = get_default_storage_options(bag_dir)
    converter_options = get_default_converter_options()

    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def create_writer(bag_dir: str) -> SequentialWriter:
    storage_options = get_default_storage_options(bag_dir)
    converter_options = get_default_converter_options()

    writer = SequentialWriter()
    writer.open(storage_options, converter_options)
    return writer

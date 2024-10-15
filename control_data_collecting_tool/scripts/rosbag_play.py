#!/usr/bin/env python3

# Copyright 2024 Proxima Technology Inc, TIER IV
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

import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class db3SingleTopic:
    def __init__(self, db3_file, topic_id):
        self.conn = sqlite3.connect(db3_file)
        self.c = self.conn.cursor()
        self.c.execute("SELECT * from messages WHERE topic_id = ?", (topic_id,))
        self.is_open = True

    def __del__(self):
        self.conn.close()

    # Fetch the next row from the SQLite query result
    def fetch_row(self):
        return self.c.fetchone()


class db3Reader:
    def __init__(self, db3_file):
        self.db3_file = db3_file

        self.db3_dict = {}

    def __del__(self):
        # Clean up by deleting the topic-specific objects and closing any open database connections
        for topic_db3 in self.db3_dict.values():
            del topic_db3["topic_db3"]

    # Load topic information from the .db3 file
    def load_db3(self, target_topic_name):
        # Try to establish a connection to the .db3 SQLite database and retrieve topics
        try:
            conn = sqlite3.connect(self.db3_file)
            c = conn.cursor()
            c.execute("SELECT * from({})".format("topics"))
            topics = c.fetchall()

        except sqlite3.Error:
            return False

        # Create a dictionary that maps topic names to their respective ID and message type
        topic_types = {
            topics[i][1]: {"topic_id": i + 1, "topic_type": get_message(topics[i][2])}
            for i in range(len(topics))
        }

        # Check if the target topic is included in the database
        target_topic_included = True
        if target_topic_name not in list(topic_types.keys()):
            target_topic_included = False
        conn.close()

        # If the target topic is not found, return False
        if not target_topic_included:
            return target_topic_included

        # if the target topic is included, establish a connection to an SQLite database to read the target topic
        topic_type = topic_types[target_topic_name]["topic_type"]
        topic_db3 = db3SingleTopic(self.db3_file, topic_types[target_topic_name]["topic_id"])
        self.db3_dict[target_topic_name] = {"topic_type": topic_type, "topic_db3": topic_db3}

        # Successfully loaded the target topic
        return True

    # Fetch a deserialized single message for the specified topic
    def read_msg(self, target_topic_name):
        msg = None
        row = self.db3_dict[target_topic_name]["topic_db3"].fetch_row()

        if row is None:
            return msg

        topic_type = self.db3_dict[target_topic_name]["topic_type"]
        msg = deserialize_message(row[3], topic_type)

        return msg

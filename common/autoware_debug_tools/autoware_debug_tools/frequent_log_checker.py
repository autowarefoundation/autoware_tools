#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

import argparse


class Log:
    def __init__(self, node_name, timestamp, content, file_and_line, full_message):
        self.node_name = node_name
        self.timestamp = timestamp
        self.content = content
        self.file_and_line = file_and_line
        self.full_message = full_message

    def is_same(self, log):
        return self.node_name == log.node_name and self.file_and_line == log.file_and_line

    def is_in(self, log_list):
        for target_log in log_list:
            if self.is_same(target_log):
                return True
        return False


def check(log_file, duration_to_count, log_count_threshold, log_format):
    recent_log_list = []
    unique_frequent_log_list = []

    # with open("autoware.log", "r") as f:
    with open(log_file, "r") as f:
        for full_message in f.readlines():
            try:
                if log_format == "1":
                    node_name = full_message.split("[")[4].split("]")[0]
                    timestamp = float(full_message.split("[")[0][:-1])
                    content = full_message.split("]")[3].split(" at ")[0]
                    file_and_line = full_message.split("]")[3].split(" at ")[1]
                    recent_log = Log(node_name, timestamp, content, file_and_line, full_message)
                elif log_format == "2":
                    node_name = full_message.split("]")[0][1:]
                    timestamp = float(full_message.split("]")[1].split(" ")[2])
                    content = full_message.split("]")[3].split(" at ")[0]
                    file_and_line = full_message.split("]")[3].split(" at ")[1]
                    recent_log = Log(node_name, timestamp, content, file_and_line, full_message)
                else:
                    continue
            except IndexError:
                continue
            except ValueError:
                continue

            # skip if the log is already considered as frequent
            if recent_log.is_in(unique_frequent_log_list):
                continue
            recent_log_list.append(recent_log)

            # remove obsolete or already frequent log
            for log in recent_log_list[:]:
                duration = timestamp - log.timestamp
                if duration_to_count < duration:
                    recent_log_list.remove(log)

            # extract duplicated (= frequent) log
            for i in range(len(recent_log_list)):
                log_count = 0
                if recent_log_list[i].is_in(unique_frequent_log_list):
                    continue

                for j in range(len(recent_log_list)):
                    if i <= j:
                        continue
                    if recent_log_list[i].is_same(recent_log_list[j]):
                        log_count += 1

                if log_count_threshold <= log_count:
                    unique_frequent_log_list.append(recent_log_list[i])

    for frequent_log in unique_frequent_log_list:
        print(frequent_log.full_message)


def main():
    parser = argparse.ArgumentParser(description="frequent log checker")
    parser.add_argument("log_file", help="launch log file")
    parser.add_argument("-d", "--log-duration", default=1.0, help="duration to count log")
    parser.add_argument("-c", "--log-count", default=2, help="log count threshold")
    parser.add_argument("-f", "--log-format", default="1", help="log format")
    args = parser.parse_args()

    check(args.log_file, args.log_duration, args.log_count, args.log_format)


if __name__ == "__main__":
    main()

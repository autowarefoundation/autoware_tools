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
    unique_frequent_log_list = {}

    with open(log_file, "r") as f:
        for full_message in f.readlines():
            # log which can be ignored
            if "create_component_factory()" in full_message:
                continue
            if "[robot_state_publisher]" in full_message:
                continue
            if "launchScenePlugin()" in full_message:
                continue

            try:
                # The following implementation depends on the log format.
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

            recent_log_list.append(recent_log)

            # remove obsolete log
            for log in recent_log_list[:]:
                duration = timestamp - log.timestamp
                if duration_to_count < duration:
                    recent_log_list.remove(log)

            # extract duplicated (= frequent) log
            for i in range(len(recent_log_list)):
                log_count = 0
                for j in range(len(recent_log_list)):
                    if i <= j:
                        continue
                    if recent_log_list[i].is_same(recent_log_list[j]):
                        log_count += 1

                if log_count_threshold <= log_count:
                    contained_frequent_log = None
                    for frequent_log in unique_frequent_log_list:
                        if frequent_log.is_same(recent_log_list[i]):
                            contained_frequent_log = frequent_log
                            break

                    if contained_frequent_log:
                        # update the existing value
                        unique_frequent_log_list[contained_frequent_log] = max(
                            unique_frequent_log_list[contained_frequent_log], log_count
                        )
                    else:
                        # add a new key and value
                        unique_frequent_log_list[recent_log_list[i]] = log_count

    if len(unique_frequent_log_list) == 0:
        print(
            "No frequent log. The log format designated by the `-f` option may be different from the actual log format."
        )
    else:
        for frequent_log in unique_frequent_log_list:
            log_count = unique_frequent_log_list[frequent_log]
            print(f"{frequent_log.full_message[:-1]}\t{log_count}")


def main():
    parser = argparse.ArgumentParser(description="frequent log checker")
    parser.add_argument("log_file", help="launch log file")
    parser.add_argument(
        "-d", "--log-duration", default=2.0, type=float, help="duration to count log"
    )
    parser.add_argument("-c", "--log-count", default=5, type=int, help="log count threshold")
    parser.add_argument("-f", "--log-format", default="1", help="log format")
    args = parser.parse_args()

    check(args.log_file, args.log_duration, args.log_count, args.log_format)


if __name__ == "__main__":
    main()

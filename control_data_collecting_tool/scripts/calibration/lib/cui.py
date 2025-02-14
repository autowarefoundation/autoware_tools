#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc. All rights reserved.
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

import sys
import time


def countdown(n):
    while n > 0:
        print(n)
        time.sleep(1)
        n -= 1


def input_yes_or_no(description):
    while True:
        try:
            answer = input(f"{description} (yes/no) > ").lower()

            if answer in {"yes", "no"}:
                return answer
            else:
                print("Please enter 'yes' or 'no'.")
        except KeyboardInterrupt as e:
            print("\nOperation canceled by user : " + str(e))
            sys.exit(1)
        except Exception as e:
            print(e)
            print("Please enter 'yes' or 'no'.")


def ready_check(description):
    is_ready = False
    while not is_ready:
        answer = input_yes_or_no(description)

        if answer == "yes":
            is_ready = True


def do_check(description, func):
    answer = input_yes_or_no(description)

    if answer == "yes":
        try:
            func()
        except Exception as e:
            print(f"Error occurred while executing the function: {e}")


def finish_check(description):
    answer = input_yes_or_no(description)

    if answer == "yes":
        return False
    else:
        return True


def input_target_value(description, min_value, max_value, unit):
    while True:
        try:
            value = float(input(f"Target {description} [{min_value} ~ {max_value} {unit}] > "))
            if value < min_value or max_value < value:
                print(f"Input target {description} is out of range.")
                continue
            break
        except KeyboardInterrupt as e:
            print("\nOperation canceled by user : " + str(e))
            sys.exit(1)
        except Exception as e:
            print(e)
            print("Please input number.")

    print(f"Target {description}: {value}")
    return value

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


from courses.along_road import Along_Road
from courses.along_road import declare_along_road_params
from courses.figure_eight import Figure_Eight
from courses.figure_eight import declare_figure_eight_params
from courses.reversal_loop_circle import Reversal_Loop_Circle
from courses.reversal_loop_circle import declare_reversal_loop_circle_params
from courses.straight_line_negative import Straight_Line_Negative
from courses.straight_line_negative import declare_straight_line_negative_params
from courses.straight_line_positive import Straight_Line_Positive
from courses.straight_line_positive import declare_straight_line_positive_params
from courses.u_shaped import U_Shaped
from courses.u_shaped import declare_u_shaped_return_params


def declare_course_params(course_name, node):
    # Declare the course parameters based on the course name
    if course_name == "eight_course":
        declare_figure_eight_params(node)
    elif course_name == "straight_line_positive":
        declare_straight_line_positive_params(node)
    elif course_name == "straight_line_negative":
        declare_straight_line_negative_params(node)
    elif course_name == "u_shaped_return":
        declare_u_shaped_return_params(node)
    elif course_name == "reversal_loop_circle":
        declare_reversal_loop_circle_params(node)
    elif course_name == "along_road":
        declare_along_road_params(node)


def create_course_subscription(course_name, node):
    if course_name == "eight_course":
        pass
    elif course_name == "straight_line_positive":
        pass
    elif course_name == "straight_line_negative":
        pass
    elif course_name == "u_shaped_return":
        pass
    elif course_name == "reversal_loop_circle":
        pass
    elif course_name == "along_road":
        pass


def load_course(course_name, step_size, params_dict):
    # Load the course based on the course name
    if course_name == "eight_course":
        course = Figure_Eight(step_size, params_dict)
    elif course_name == "straight_line_positive":
        course = Straight_Line_Positive(step_size, params_dict)
    elif course_name == "straight_line_negative":
        course = Straight_Line_Negative(step_size, params_dict)
    elif course_name == "u_shaped_return":
        course = U_Shaped(step_size, params_dict)
    elif course_name == "reversal_loop_circle":
        course = Reversal_Loop_Circle(step_size, params_dict)
    elif course_name == "along_road":
        course = Along_Road(step_size, params_dict)

    # Return the course
    return course

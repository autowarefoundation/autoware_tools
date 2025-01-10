# autoware_lanelet2_map_validator

`autoware_lanelet2_map_validator` is a tool to validate Lanelet2 maps to ensure that Autoware can work properly with it.

This validation tool is an extension of [lanelet2_validation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_validation) so that Autoware specific rules can be applied. As you can see from the codes in the `src/validators` directory, the group of validators belong to this tool inherits the [lanelet::validation::MapValidator class](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_validation/include/lanelet2_validation/BasicValidator.h#L17) from the original `lanelet2_validation`. Therefore, we believe that reading the source code of the `lanelet2_validation` will help you understand this tool better.

**Note that this validator is still on construction that there are only a few rules and a template to define those rules.**

The official Autoware requirements for lanelet2 maps are described in [Vector Map creation requirement specifications (in Autoware Documentation)](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/map-requirements/vector-map-requirements-overview/).

## Design concept

The `autoware_lanelet2_map_validator` is designed to validate `.osm` map files by using and extending the [lanelet2_validation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_validation) package for Autoware.

`autoware_lanelet2_map_validator` takes the lanelet2 map (.osm file) and requirement set (JSON file, optional) as the input, and output validation results to the console.

If a requirement set is given, `autoware_lanelet2_map_validator` also outputs validation results reflecting the input requirement set.
See ["Run with a requirement set"](#run-with-a-requirement-set) for further information, ["Input file"](#input-file) to understand the input format, and ["Output file"](#output-file) to understand the output format.

![autoware_lanelet2_map_validator_architecture](./media/autoware_lanelet2_map_validator_architecture.drawio.svg)

## How to use

Basically, you can use the following command to execute `autoware_lanelet2_map_validator`. However, note that `autoware_lanelet2_map_validator` is a ROS/rclcpp free tool, so you can just run the built executable whatever way.

```bash
ros2 run autoware_lanelet2_map_validator autoware_lanelet2_map_validator
```

### Run ALL validators

You can use `autoware_lanelet2_map_validator` with the following command. This will run all validators including the default built-in validators in the [lanelet2_validation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_validation).

```bash
ros2 run autoware_lanelet2_map_validator autoware_lanelet2_map_validator --map_file <absolute_or_relative_path/to_your/lanelet2_map.osm> --projector mgrs
```

### Run a specific validator

`autoware_lanelet2_map_validator` consists of multiple small validators in order to realize complex requirements with a combination of them.
If you want to call a few validators, you can select them with the `--validator, -v` option.

```bash
ros2 run autoware_lanelet2_map_validator autoware_lanelet2_map_validator --map_file path/to_your/lanelet2_map.osm --projector mgrs --validator mapping.traffic_light.missing_regulatory_elements
```

You can get a list of available validators with the `--print` option.
(`-p` is for `--projector`)

```bash
ros2 run autoware_lanelet2_map_validator autoware_lanelet2_map_validator --print
```

### Run with a requirement set

`autoware_lanelet2_map_validator` can manage to run a group of validators by a list of validator names.
`autoware_lanelet2_map_validator` will scan through the input JSON file given by the `--input_requirements, -i` option, and output the validation results to the directory given by the `--output_directory, -o` option.
The output filename will be `lanelet2_validation_results.json`.

```bash
ros2 run autoware_lanelet2_map_validator autoware_lanelet2_map_validator --map_file <absolute_or_relative_path/to_your/lanelet2_map.osm> --projector mgrs --input_requirements <absolute_or_relative_path_to_autoware_requirement_set.json> --output_directory ./
```

(Short version)

```bash
ros2 run autoware_lanelet2_map_validator autoware_lanelet2_map_validator -m <absolute_or_relative_path/to_your/lanelet2_map.osm> -p mgrs -i <absolute_or_relative_path_to_autoware_requirement_set.json> -o ./
```

#### Input file

The JSON file input should follow the structure like this example.

```json
{
  "requirements": [
    {
      "id": "vm-02-02",
      "validators": [
        {
          "name": "mapping.stop_line.missing_regulatory_elements"
        }
      ]
    },
    {
      "id": "vm-04-01",
      "validators": [
        {
          "name": "mapping.crosswalk.missing_regulatory_elements"
        },
        {
          "name": "mapping.crosswalk.regulatory_element_details",
          "prerequisites": [
            {
              "name": "mapping.crosswalk.missing_regulatory_elements"
            }
          ]
        }
      ]
    },
    {
      "id": "vm-05-01",
      "validators": [
        {
          "name": "mapping.traffic_light.missing_regulatory_elements"
        },
        {
          "name": "mapping.traffic_light.regulatory_element_details",
          "prerequisites": [
            {
              "name": "mapping.traffic_light.missing_regulatory_elements"
            }
          ]
        }
      ]
    }
  ]
}
```

- MUST have a single `requirements` field.
- The `requirements` field MUST be a list of requirements. A requirement MUST have
  - `id` : The id of the requirement. Its name is arbitrary.
  - `validators` : A list of validators that structures the requirement.
    - A validator MUST be given with its name on the `name` field.
    - The name list of available validators can be obtained from the `--print` option.
    - You can add a list of `prerequisites` to each validator. Then, the validator will only be run when the prerequisites pass the validation.
    - In the `prerequisites` field, you can add `forgive_warnings: true` in order to run the validator even if the prerequisites output warning issues. (Error issues from prerequisites will still skip the validation.). Note that NOT writing the `forgive_warnings` field and writing `forgive_warnings: false` means the same.
- The user can write any other field (like `version`) besides `requirements`.

#### Output file

When the `input_requirements` is thrown to `autoware_lanelet2_map_validator`, it will output a `lanelet2_validation_results.json` file which looks like the following example.

```json
{
  "requirements": [
    {
      "id": "vm-02-02",
      "passed": true,
      "validators": [
        {
          "name": "mapping.stop_line.missing_regulatory_elements",
          "passed": true
        }
      ]
    },
    {
      "id": "vm-04-01",
      "passed": false,
      "validators": [
        {
          "issues": [
            {
              "id": 163,
              "issue_code": "Crosswalk.MissingRegulatoryElements-001",
              "message": "No regulatory element refers to this crosswalk.",
              "primitive": "lanelet",
              "severity": "Error"
            },
            {
              "id": 164,
              "issue_code": "Crosswalk.MissingRegulatoryElements-001",
              "message": "No regulatory element refers to this crosswalk.",
              "primitive": "lanelet",
              "severity": "Error"
            },
            {
              "id": 165,
              "issue_code": "Crosswalk.MissingRegulatoryElements-001",
              "message": "No regulatory element refers to this crosswalk.",
              "primitive": "lanelet",
              "severity": "Error"
            },
            {
              "id": 166,
              "issue_code": "Crosswalk.MissingRegulatoryElements-001",
              "message": "No regulatory element refers to this crosswalk.",
              "primitive": "lanelet",
              "severity": "Error"
            }
          ],
          "name": "mapping.crosswalk.missing_regulatory_elements",
          "passed": false
        },
        {
          "issues": [
            {
              "id": 0,
              "issue_code": "General.PrerequisitesFailure-001",
              "message": "Prerequisites didn't pass",
              "primitive": "primitive",
              "severity": "Error"
            }
          ],
          "name": "mapping.crosswalk.regulatory_element_details",
          "passed": false,
          "prerequisites": [
            {
              "name": "mapping.crosswalk.missing_regulatory_elements"
            }
          ]
        }
      ]
    },
    {
      "id": "vm-05-01",
      "passed": false,
      "validators": [
        {
          "name": "mapping.traffic_light.missing_regulatory_elements",
          "passed": true
        },
        {
          "issues": [
            {
              "id": 9896,
              "issue_code": "TrafficLight.MissingRegulatoryElements-001",
              "message": "Regulatory element of traffic light must have a stop line(ref_line).",
              "primitive": "regulatory element",
              "severity": "Error"
            },
            {
              "id": 9918,
              "issue_code": "TrafficLight.MissingRegulatoryElements-001",
              "message": "Regulatory element of traffic light must have a stop line(ref_line).",
              "primitive": "regulatory element",
              "severity": "Error"
            },
            {
              "id": 9838,
              "issue_code": "TrafficLight.MissingRegulatoryElements-001",
              "message": "Regulatory element of traffic light must have a stop line(ref_line).",
              "primitive": "regulatory element",
              "severity": "Error"
            },
            {
              "id": 9874,
              "issue_code": "TrafficLight.MissingRegulatoryElements-001",
              "message": "Regulatory element of traffic light must have a stop line(ref_line).",
              "primitive": "regulatory element",
              "severity": "Error"
            }
          ],
          "name": "mapping.traffic_light.regulatory_element_details",
          "passed": false,
          "prerequisites": [
            {
              "name": "mapping.traffic_light.missing_regulatory_elements"
            }
          ]
        }
      ]
    }
  ]
}
```

- `lanelet2_validation_results.json` inherits the JSON file of `input_requirements` and add results to it.
  - So additional input information not related to this validator also remains in the output.
- `autoware_lanelet2_map_validator` adds a boolean `passed` field to each requirement. If all validators of the requirement have been passed, the `passed` field of the requirement will be `true` (`false` if not).
- The `passed` field is also given to each validator. If the validator found any issues the `passed` field will turn to be `false` (`true` if not), and adds an `issues` field which is a list of issues found. Each issue contains information of `severity`, `primitive`, `id`, and `message`.

### Available command options

| option                     | description                                                                                                                                                     |
| -------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-h, --help`               | Explains about this tool and show a list of options                                                                                                             |
| `--print`                  | Print all available checker without running them                                                                                                                |
| `-m, --map_file`           | Path to the map to be validated                                                                                                                                 |
| `-i, --input_requirements` | Path to the JSON file where the list of requirements and validations is written                                                                                 |
| `-o, --output_directory`   | Directory to save the list of validation results in a JSON format                                                                                               |
| `-v, --validator`          | Comma separated list of regexes to filter the applicable validators. Will run all validators by default. Example: `mapping.*` to run all checks for the mapping |
| `-p, --projector`          | Projector used for loading lanelet map. Available projectors are: `mgrs`, `utm`, and `transverse_mercator`.                                                     |
| `-l, --location`           | Location of the map (for instantiating the traffic rules), e.g. de for Germany                                                                                  |
| `--participants`           | Participants for which the routing graph will be instantiated (default: vehicle)                                                                                |
| `--lat`                    | latitude coordinate of map origin. This is required for the transverse mercator and utm projector.                                                              |
| `--lon`                    | longitude coordinate of map origin. This is required for the transverse mercator and utm projector.                                                             |

### Available validators

Since there will be hundreds of validators in the future, the documents for each validator should categorized in the docs file.
The directory structure should be the same to that of the `src/validators` directory.

#### Stop Line

- [mapping.stop_line.missing_regulatory_elements](./docs/stop_line/missing_regulatory_elements_for_stop_lines.md)

#### Traffic Light

- [mapping.traffic_light.missing_regulatory_elements](./docs/traffic_light/missing_regulatory_elements_for_traffic_lights.md)
- [mapping.traffic_light.regulatory_element_details](./docs/traffic_light/regulatory_element_details_for_traffic_lights.md)

#### Crosswalk

- [mapping.crosswalk.missing_regulatory_elements](./docs/crosswalk/missing_regulatory_elements_for_crosswalk.md)
- [mapping.crosswalk.regulatory_element_details](./docs/crosswalk/regulatory_element_details_for_crosswalks.md)

## How to add a new validator

If you want to contribute to `autoware_lanelet2_map_validator`, please check out the [how_to_contribute](./docs/how_to_contribute.md) first.

## Relationship between requirements and validators

This is a table describing the correspondence between the validators that each requirement consists of.
The "Validators" column will be blank if it hasn't be implemented.

| ID       | Requirements                                            | Validators                                                                                                                                                                                                                                                                                                                                                      |
| -------- | ------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| vm-01-01 | Lanelet basics                                          |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-02 | Allowance for lane changes                              |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-03 | Linestring sharing                                      |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-04 | Sharing of the centerline of lanes for opposing traffic |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-05 | Lane geometry                                           |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-06 | Line position (1)                                       | (Not validatable because real-world correspondence cannot be determined programmatically.)                                                                                                                                                                                                                                                                      |
| vm-01-07 | Line position (2)                                       | (Not validatable because real-world correspondence cannot be determined programmatically.)                                                                                                                                                                                                                                                                      |
| vm-01-08 | Line position (3)                                       | (Not validatable because real-world correspondence cannot be determined programmatically.)                                                                                                                                                                                                                                                                      |
| vm-01-09 | Speed limits                                            |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-10 | Centerline                                              |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-11 | Centerline connection (1)                               |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-12 | Centerline connection (2)                               |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-13 | Roads with no centerline (1)                            |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-14 | Roads with no centerline (2)                            |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-15 | Road shoulder                                           |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-16 | Road shoulder Linestring sharing                        |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-17 | Side strip                                              |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-18 | Side strip Linestring sharing                           |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-01-19 | Walkway                                                 |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-02-01 | Stop line alignment                                     | (Not validatable because real-world correspondence cannot be determined programmatically.)                                                                                                                                                                                                                                                                      |
| vm-02-02 | Stop sign                                               | [mapping.stop_line.missing_regulatory_elements](./docs/stop_line/missing_regulatory_elements_for_stop_lines.md)                                                                                                                                                                                                                                                 |
| vm-03-01 | Intersection criteria                                   | [mapping.intersection.intersection_area_dangling_reference](./docs/intersection/intersection_area_dangling_reference.md)                                                                                                                                                                                                                                        |
| vm-03-02 | Lanelet's turn direction and virtual                    |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-03 | Lanelet width in the intersection                       |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-04 | Lanelet creation in the intersection                    |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-05 | Lanelet division in the intersection                    |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-06 | Guide lines in the intersection                         |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-07 | Multiple lanelets in the intersection                   |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-08 | Intersection Area range                                 | [mapping.intersection.intersection_area_validity](./docs/intersection/intersection_area_validity.md), [mapping.intersection.intersection_area_segment_type](./docs/intersection/intersection_area_segment_type.md)                                                                                                                                              |
| vm-03-09 | Range of Lanelet in the intersection                    |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-10 | Right of way (with signal)                              |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-11 | Right of way (without signal)                           |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-12 | Right of way supplements                                | (Nothing to validate.)                                                                                                                                                                                                                                                                                                                                          |
| vm-03-13 | Merging from private area, sidewalk                     |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-14 | Road marking                                            |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-03-15 | Exclusive bicycle lane                                  |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-04-01 | Traffic light basics                                    | [mapping.traffic_light.missing_regulatory_elements](./docs/traffic_light/missing_regulatory_elements_for_traffic_lights.md), [mapping.traffic_light.regulatory_element_details](./docs/traffic_light/regulatory_element_details_for_traffic_lights.md), [mapping.traffic_light.missing_referrers](./docs/traffic_light/missing_referrers_for_traffic_lights.md) |
| vm-04-02 | Traffic light position and size                         | [mapping.traffic_light.correct_facing](./docs/traffic_light/traffic_light_facing.md) (TODO: Compare the light_bulb direction and traffic_light direction.)                                                                                                                                                                                                      |
| vm-04-03 | Traffic light lamps                                     |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-05-01 | Crosswalks across the road                              | [mapping.crosswalk.missing_regulatory_elements](./docs/crosswalk/missing_regulatory_elements_for_crosswalk.md), [mapping.crosswalk.regulatory_element_details](./docs/crosswalk/regulatory_element_details_for_crosswalks.md) (TODO: handle cases against missing referrers)                                                                                    |
| vm-05-02 | Crosswalks with pedestrian signals                      |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-05-03 | Deceleration for safety at crosswalks                   |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-05-04 | Fences                                                  |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-06-01 | Buffer Zone                                             |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-06-02 | No parking signs                                        |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-06-03 | No stopping signs                                       |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-06-04 | No stopping sections                                    |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-06-05 | Detection area                                          |                                                                                                                                                                                                                                                                                                                                                                 |
| vm-07-01 | Vector Map creation range                               | (Not validatable because real-world correspondence cannot be determined programmatically?)                                                                                                                                                                                                                                                                      |
| vm-07-02 | Range of detecting pedestrians who enter the road       | (Not validatable because real-world correspondence cannot be determined programmatically.)                                                                                                                                                                                                                                                                      |
| vm-07-03 | Guardrails, guard pipes, fences                         | (Not validatable because real-world correspondence cannot be determined programmatically.)                                                                                                                                                                                                                                                                      |
| vm-07-04 | Ellipsoidal height                                      | (Not validatable because real-world correspondence cannot be determined programmatically?)                                                                                                                                                                                                                                                                      |

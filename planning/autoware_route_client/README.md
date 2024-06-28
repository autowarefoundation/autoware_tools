# Route Client

This package contains a tool to send request to set route.

## Usage

### Prepare a route file

Prepare a YAML file containing route information.
The file format is like following:

```yaml
goal:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
segments:
  - preferred:
      id: 0
      type: lane
    alternatives:
      - id: 1
        type: lane
  - preferred:
      id: 2
      type: lane
    alternatives: []
  - preferred:
      id: 3
      type: lane
    alternatives:
      - id: 4
        type: lane
```

### Send request to set route

Execute following command.

```bash
ros2 run autoware_route_client route_client.py <path_to_yaml_file>
```

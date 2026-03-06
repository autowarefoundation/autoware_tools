# Route history and replay for the planning tool

## Overview

This package provides an interactive RViz panel for saving poses, enabling the quick (re)generation of routes for simulation and testing.

## Motivation

The repeatable testing of a specific route is slowed by the need to replot. This may be done manually through "Add goal pose" and similar toolbar tools, but use of the GUI is prone to inaccuracy. Alternatively, if poses are recorded separately, service calls may uses. This RViz panel aims to streamline this process.

## Quick Start

```sh
cd ~/autoware_tools/planning
colcon build --packages-select autoware_route_history
source install/setup.bash
```

After launching the planning tool the RouteHistory panel will be discoverable.
Panels > Add New Panel > RouteHistoryPanel > OK.

## Output Files

- **`~/.ros/route_history.yaml`** - Saved routes are paired with uuids and written to a YAML file. A new file path can be chosen through the "Load save file" button in the running RViz panel.

## Parameters

- **`save_file_path`** - Holds the path string associated with the location of the save file.

## Future Work

- **Groups** - group routes for sequential, automatic simulation.
- **Terminal access to features** - easy-to-use terminal commands for managing tests.

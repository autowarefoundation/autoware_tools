# Topic Connection Checker

The Topic Connection Checker is an advanced diagnostic tool designed to simulate how an Autoware Engineer debugs an Autoware system in the field when certain topics are not functioning as expected. This tool is essential for identifying and resolving issues in the complex topic network of an Autoware system.

## Overview

The tool consists of two main components:

1. Topic Connection Checker
2. Topic Localizer

These components work together to provide a comprehensive debugging experience, following a systematic approach to identify and locate problematic topics in the Autoware system.

## Debugging Process

The Topic Connection Checker follows a step-by-step process that mimics an experienced Autoware Engineer's debugging approach:

1. Identify key topics with unexpected output using diagnostic information and major final output topics.
2. Track the publishing nodes of these blocked output topics.
3. Investigate the subscribed inputs of these publishing nodes to find upstream blocked topics.
4. Trace back to the source to identify key topics that are not being published.
5. Locate the source of lost topics in the code or launching system.

Steps 1-4 are handled by the Topic Connection Checker, while step 5 is addressed by the Topic Localizer.

## Topic Connection Checker

### Usage

To run the Topic Connection Checker, use the following command:

```bash
ros2 run autoware_debug_tools topic_connection_checker
```

### Mechanism

The Topic Connection Checker operates as follows:

1. Subscribes to `/diagnostics` for three seconds, focusing on `hardware_id` with `topic_state_monitor`.
2. Subscribes to and traces stuck topics and all upstream topics.
3. Performs multiple iterations to identify topics without publishers.
4. Reports ERROR in the command line for problematic topics.

The identified problematic topics can then be used as input for the Topic Localizer.

## Topic Localizer

### Usage

When launching from the `autoware/pilot-auto` directory, use the following command:

```bash
ros2 run autoware_debug_tools topic_localizer --topics $TOPIC1,$TOPIC2
```

Replace `$TOPIC1,$TOPIC2` with the actual topic names you want to localize, separated by commas.

### Mechanism

The Topic Localizer employs a two-step approach to find the source of problematic topics:

1. Direct Search:
   - Scans all HPP/CPP and launch.py files for code snippets containing the exact names of the target topics.

2. Launch System Analysis:
   - Starts with `autoware_launch/launch/autoware.launch.xml` using default arguments.
   - Statically traces all XML files in the launch system.
   - Identifies launch parameters or remapped topics matching the names of the target topics.

## Best Practices

1. Always start with the Topic Connection Checker to identify problematic topics.
2. Use the output from the Topic Connection Checker as input for the Topic Localizer.
3. Pay attention to ERROR messages in the command line output.
4. When using the Topic Localizer, ensure you're in the correct directory (autoware/pilot-auto).
5. Keep track of the relationships between topics to understand the flow of data in your Autoware system.

## Troubleshooting

- If the Topic Connection Checker doesn't identify any issues, but you're still experiencing problems, it could be related to the pulishing frequencies. The connection checker marks a topic as "fine" if it has a single output in five seconds.
- If the Topic Localizer doesn't find any matches, it is usually because of the topics are launch using launch.py files with sophisticated construction functions.

By using these tools effectively, Autoware Engineers can quickly identify and resolve topic-related issues, ensuring smooth operation of the Autoware system.

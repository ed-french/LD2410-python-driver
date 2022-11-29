# LD2410 Threaded python driver

## Overview
This driver allows the sensor to be driven in python via a threaded driver that buffers data.
Not directly suitable for micropython, but might provide a useful guide to anyone writing drivers
for this sensor as the documentation is a bit ropey (included here).

## Usage

Simplest usage, just fetching the most recent readings:
```
    with ld2410.LD2410("COM7",maxlen=20) as sensor:
        print(f"\n\nREADING SET\n{sensor.get_latest()}")
```
"maxlen" can be set to determine the required buffer size
each Reading object can be queried for the full engineering mode dataset


Sometimes it's useful to get access to a buffer of recent readings, for instance to smooth data, to allow less frequent checking etc. Recent readings are in a deque, so popleft will pull the oldest.
Readings have a .timestamp datetime to put them in time order

Readings are collected in a Daemon thread that is shut down tidily when the context manager ends.

Thresholds can be set and tested but this isn't fully implemented yet!

# DIAMANTS V3 - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
# 
# Copyright (c) 2025 DIAMANTS Project Contributors
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# -*- coding: utf-8 -*-
"""
Helper to decode binary logged sensor data from crazyflie2 with uSD-Card-Deck.

Source: https://github.com/IMRCLab/crazyflie-firmware/blob/master/tools/usdlog/cfusdlog.py

MIT License

Copyright (c) 2021 Intelligent Multi-Robot Coordination Lab @ TU Berlin, Germany

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import argparse
from zlib import crc32
import struct
import numpy as np

# extract null-terminated string
def _get_name(data, idx):
    endIdx = idx
    while data[endIdx] != 0:
        endIdx = endIdx + 1
    return data[idx:endIdx].decode("utf-8"), endIdx + 1

def decode(filename):
    # read file as binary
    with open(filename, 'rb') as f:
        data = f.read()

    # check magic header
    if data[0] != 0xBC:
        print("Unsupported format!")
        return

    # check CRC
    crc = crc32(data[0:-4])
    expected_crc, = struct.unpack('I', data[-4:])
    if crc != expected_crc:
        print("WARNING: CRC does not match!")

    # check version
    version, num_event_types = struct.unpack('HH', data[1:5])
    if version != 1 and version != 2:
        print("Unsupported version!", version)
        return

    result = dict()
    event_by_id = dict()

    # read header with data types
    idx = 5
    for _ in range(num_event_types):
        event_id, = struct.unpack('H', data[idx:idx+2])
        idx += 2
        event_name, idx = _get_name(data, idx)
        result[event_name] = dict()
        result[event_name]["timestamp"] = []
        num_variables, = struct.unpack('H', data[idx:idx+2])
        idx += 2
        fmtStr = "<"
        variables = []
        for _ in range(num_variables):
            var_name_and_type, idx = _get_name(data, idx)
            var_name = var_name_and_type[0:-3]
            var_type = var_name_and_type[-2]
            result[event_name][var_name] = []
            fmtStr += var_type
            variables.append(var_name)
        event_by_id[event_id] = {
            'name': event_name,
            'fmtStr': fmtStr,
            'numBytes': struct.calcsize(fmtStr),
            'variables': variables,
            }

    while idx < len(data) - 4:
        if version == 1:
            event_id, timestamp, = struct.unpack('<HI', data[idx:idx+6])
            idx += 6
        elif version == 2:
            event_id, timestamp, = struct.unpack('<HQ', data[idx:idx+10])
            timestamp = timestamp / 1000.0
            idx += 10
        event = event_by_id[event_id]
        fmtStr = event['fmtStr']
        eventData = struct.unpack(fmtStr, data[idx:idx+event['numBytes']])
        idx += event['numBytes']
        for v,d in zip(event['variables'], eventData):
            result[event['name']][v].append(d)
        result[event['name']]["timestamp"].append(timestamp)

    # remove keys that had no data
    for event_name in list(result.keys()):
        if len(result[event_name]['timestamp']) == 0:
            del result[event_name]

    # convert to numpy arrays
    for event_name in result.keys():
        for var_name in result[event_name]:
            result[event_name][var_name] = np.array(result[event_name][var_name])

    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename")
    args = parser.parse_args()
    data = decode(args.filename)
    print(data)

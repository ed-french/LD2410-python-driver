#!/usr/bin/env python3

#################################################################
#                                                               #
#                                                               #
#               LD2410 mmWave Sensor Python Driver              #
#                                                               #
#                    (c) Ed French 2022                         #
#                                                               #
#################################################################



"""
    LD2410 mmWave Sensor Driver

    Example code for simplest application

"""

import ld2410
import time


if __name__=="__main__":
    with ld2410.LD2410("COM7",maxlen=20) as sensor:
        for _ in range(100):
            time.sleep(0.5)
            print(f"\n\nREADING SET\n{sensor.get_latest()}")
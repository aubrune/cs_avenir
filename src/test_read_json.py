#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import rospkg
import json
from os.path import join



rospack = rospkg.RosPack()

with open(join(rospack.get_path("cs_avenir"), "config/poses.json")) as f:
    poses = json.load(f)

print "============ Press `Enter` to begin ..."
raw_input()

print "============ read poses.json"
print(poses["industriel"]["joints"])
print ""
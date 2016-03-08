# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/indigo/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/indigo/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/alfredoso/Pleiades/artags_22_nov_2015/devel;/home/alfredoso/Pleiades/spirigo_18_nov_2015_ws/devel;/home/alfredoso/Pleiades/visual_slam_12_oct_2015/ps4eye_ws/devel;/home/alfredoso/Pleiades/rune_finder_11_oct_2015/spiri_catkin/devel;/home/alfredoso/Pleiades/arducopter-pixhawk/simulators/catkin_ws/devel;/opt/ros/indigo".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/devel/env.sh')

output_filename = '/home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)

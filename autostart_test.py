#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyrigyt Toho Technos co.
totech_route_select.py:

ルートの自動選択を実行する。
 1) 機体の現在位置を得る。
 2) ルートファイルをサーチして、現在位置がHomeに入っているルートを探す。
 3) 検索したルートを機体に転送する。

"""
from __future__ import print_function


from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import os
import math
import time
import glob
from os.path import expanduser





#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates mission import/export from a file.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

if not connection_string:
    connection_string = "127.0.0.1:14551"

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


#print('Waiting')
#time.sleep(60)


# Connect to the Vehicle
while True:
    try:
        print('Connecting to vehicle on: %s' % connection_string)
        vehicle = connect(connection_string, wait_ready=True)
    except Exception as e:
        print("chatch exception:%s " % e)
        time.sleep(10)
        print('Retry.')
    else:
        print('Connected.')
        break



# ここからメイン

# 初期化





# for debug 
# とりあえず無限ループ
while True:
    #print("vehicle.mode = 'MANUAL'")
    vehicle.mode = VehicleMode("MANUAL")
    time.sleep(2)
    
    #print("vehicle.mode = 'HOLD'")
    vehicle.mode = VehicleMode("HOLD")
    time.sleep(2)


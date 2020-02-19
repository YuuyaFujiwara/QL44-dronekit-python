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


global global_flg_route_select_req
global global_flg_route_save_req
global global_flg_route_clear_req
global home_path
global ROUTEFILE_EXT

global_flg_route_select_req = False
global_flg_route_save_req = False
global_flg_route_clear_req = False

#home_path = "C:/Users/YuyaFujiwara/Documents/GitHub/dronekit-python/examples/totech_RouteSelect/"
home_path = expanduser("~")
print('home path: %s' % home_path )
route_path = home_path + "/QL44/Routes"
print('route path: %s' % route_path)

ROUTEFILE_EXT = ".waypoints"

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


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)





# ここからメイン

# 保存フォルダ作成
my_makedirs(route_path)


# 初期化
vehicle.parameters['MOMIMAKI_RT_CTRL'] = -1








# for debug 
# とりあえず無限ループ
count = 0
while True:
    print("vehicle.parameters['MOMIMAKI_RT_CTRL'] = ".format( count ) )
    vehicle.parameters['MOMIMAKI_RT_CTRL'] = count
    count = count + 1
    time.sleep(1)
    


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
import math
import time
import glob
#from os.path import expanduser


#print('current path: %s' % expanduser("~"))

home_path = "C:/Users/YuyaFujiwara/Documents/GitHub/dronekit-python/examples/totech_RouteSelect/"
global_flg_route_select_req = False

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates mission import/export from a file.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

if not connection_string:
    connection_string = "127.0.0.1:14552"

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Check that vehicle is armable. 
# This ensures home_location is set (needed when saving WP file)

while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)



# メートル単位で移動した緯度経度を得る
#
# original get_location_metres in mission_basic.py
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

# 緯度経度からコマンド（Missionの部品）を作成する。
def make_command_from_latlng( aLat, aLng, aIndex ):
    cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, aLat, aLon, aIndex )
    return cmd


# ファイル内の1行からコマンド（Missionの部品）を作成する
def make_command_from_line( aLine ):
    linearray = aLine.split('\t')
    ln_index  = int(linearray[0])
    ln_currentwp=int(linearray[1])
    ln_frame  = int(linearray[2])
    ln_command= int(linearray[3])
    ln_param1 = float(linearray[4])
    ln_param2 = float(linearray[5])
    ln_param3 = float(linearray[6])
    ln_param4 = float(linearray[7])
    ln_param5 = float(linearray[8])
    ln_param6 = float(linearray[9])
    ln_param7 = float(linearray[10])
    ln_autocontinue=int(linearray[11].strip())
    return Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)



# 2つのGlobalPositionから、距離をメートル単位で得る
#
# ※ guided_set_speed_yaw.py 内の get_distance_metersでは、緯度を考慮していないため作り直し
def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlng = aLocation2.lon - aLocation1.lon
    avgLat = ( aLocation2.lat + aLocation1.lat ) / 2.0

    dNorth = dlat * 110946.0                                    # 緯度差 * 地球の極円周 / 360
    dEast  = dlat * 111319.0 * math.cos(avgLat*math.pi/180.0)   # 経度差 * 地球の赤道円周 / 360 * cos(緯度)

    return math.sqrt((dNorth*dNorth) + (dEast*dEast))


'''
# original in mission_import_export
def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist
'''




# ルート(mission)をファイルから読み出す
# フォーマット
# セッション[Totech＿route_xxx]で始まり、次のセッションの前まで
#    1行目：HOME Lat, Long
#    2行目～： （HOMEからの) X, Y, Mission
#
#
# original is readmission in mission_import_export
def read_route( aFileName ):

    print("\nReading route from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    homepos = LocationGlobal( -9999, -9999, -9999 ) 
    with open(aFileName) as f:
        i = 0
        for line in f:
            if i == 0:  #セクションの始まりを探す
                #if not line.startswith("[Totech_route_099]"):
                if not line.startswith("QGC WPL 110"):
                    raise Exception("File is not supported WP version")
                else:
                    i = 1
            elif i==1:  #１行目：緯度経度を得る
                linearray=line.split('\t')
                if linearray.count > 2:
                    #lat = float(linearray[6])
                    #lng = float(linearray[7])
                    #homepos = LocationGlobal(　lat, lng ) 
                    # commandを作成しmissionに登録する
                    #cmd = make_command_from_latlng( lat, lng, missionlist.count )
                    cmd = make_command_from_line( line )
                    missionlist.append(cmd)
                    i = 2
            elif line.startswith('['):
                break   # exit from for

            else:
                linearray=line.split('\t')
                if linearray.count > 2:
                    # Homeからの差分で位置を得る
                    #mtr_east  = ln_command=int(linearray[0])
                    #mtr_north = ln_command=int(linearray[1])
                    #pos = get_location_metres( homepos,mtr_north< mtr_east )
                    # commandを作成しmissionに登録する
                    #cmd = make_command_from_latlng( lat, lng, missionlist.count )
                    cmd = make_command_from_line( line )
                    missionlist.append(cmd)
                    i += 1

    return missionlist

# missionの始点と現在位置の差がradius以下であるか確認する。
# 
def mission_distance_check( aMission ):
    if aMission.count == 0:
        return False

    vehicle_loc = vehicle.location.global_frame
    mission_home = LocationGlobal( aMission[0].x, aMission[0].y )   #ミッションの最初のXとY
    dist = get_distance_metres( vehicle_loc, mission_home )
    if dist < 3.0:
        return True
    else:
        return False


'''
# オリジナル
# 指定されたファイルのミッションを機体に送る
#
def upload_mission(aFileName):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(aFileName)
    
    print("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()
'''
#ミッションを機体に送る
def upload_mission(aMission):
    
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in aMission:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()



def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)    
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)
        

def print_mission( aMission ):
    for cmd in aMission:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        print( commandline )


        
def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip())        

'''
#新たなミッションを書き込む。
# 
# original function is adds_square_mission() in mission_basic.py
def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

    print(" Upload new commands to vehicle")
    cmds.upload()
'''


#パラメータ変更時コールバック関数登録
#MOMIMAKI_RT_CTRLパラメータ変更時に、ルート管理を行う。
# ※このパラメータはdronekit-pythonからのルート管理用。船体では使用しない。        
# Add mode attribute callback using decorator (callbacks added this way cannot be removed).
print("\nAdd `MOMIMAKI_RT_CTRL` attribute callback/observer using decorator") 
@vehicle.parameters.on_attribute('MOMIMAKI_RT_CTRL')   
def decorated_routectrl_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.

    global global_flg_route_select_req

    print(" CALLBACK: 'MOMIMAKI_RT_CTRL' changed to", value)

    # パラメータ値により動作を分ける
    if value == 0 :
        print(" value == zero" )
    elif value == 9999 :
        # 全ルートクリア
        print(" clear all route files(not yet)" )
    elif value == 1:
        # 現在のルートをファイルに保存
        print(" save to file current route(not yet)" )
    elif value == 2:
        print(" value == 2nd" )
    elif value == 3:
        print(" value == 3rd" )
    else :
        print(" value == other" )






#モード変更時コールバック関数登録
#モード変更時に、ルート選択を実行する。        
# Add mode attribute callback using decorator (callbacks added this way cannot be removed).
print("\nAdd `mode` attribute callback/observer using decorator") 
@vehicle.on_attribute('mode')   
def decorated_mode_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.

    global global_flg_route_select_req

    print(" CALLBACK: Mode changed to", value)
    # ModeがAutoになった場合に、ルートを決める。
    # ルートが決まらなかったら、STABILIZEにモードを変える。
    if value==VehicleMode("GUIDED"):
        global_flg_route_select_req = True


#
# 自動ルート選択処理
#
def route_select_proc():
    global global_flg_route_select_req

    # ルートファイルを検索
    search_rslt = False
    mission = []
    #for fname in glob.glob("./Routes/*.route"):
    #for fname in glob.glob("./Routes/*.waypoints"):     #とりあえずmission plannerが出力するwaypointsファイルを読み込む
    #tmppath = home_path +  "Routes/*.*"
    for fname in glob.glob( home_path +  "Routes/*.waypoints"):     #とりあえずmission plannerが出力するwaypointsファイルを読み込む
        mission = read_route(fname)
        if mission_distance_check( mission ):
            search_rslt = True
            break       # Exit for

    # 機体に転送
    if search_rslt:
        #for debug
        print_mission( mission )
        upload_mission( mission )
    else:
        print( "No route found\n")
        vehicle.commands.clear()
        vehicle.mode = VehicleMode("MANUAL")

    # reset the flag
    global_flg_route_select_req = False 
    



'''
#Arming状態 変更時コールバック関数登録
print("\nAdd `armed` attribute callback/observer using decorator") 
@vehicle.on_attribute('armed')   
def decorated_armed_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.

    print(" CALLBACK: Armed status was changed to", value)
'''

'''
#(?) modeをSTABLIZEに設定
print(" Set mode=STABILIZE (currently: %s) and wait for callback" % vehicle.mode.name) 
vehicle.mode = VehicleMode("STABILIZE")

print(" Wait 2s so callback invoked before moving to next example")
time.sleep(2)

print("\n Attempt to remove observer added with `on_attribute` decorator (should fail)") 
try:
    vehicle.remove_attribute_listener('mode', decorated_mode_callback)
except:
    print(" Exception: Cannot remove observer added using decorator")
'''


# ここからメイン

# for debug 
# とりあえず無限ループ
while True:
    # フラグが立ったらルート選択処理を行う
    if global_flg_route_select_req==True:
        route_select_proc()
    time.sleep(1)
    


'''
# 1) 現在位置を得る
#Vehicle_loc = vehicle.location.global_frame

# 2) ルートファイルを検索
search_rslt = False
mission = []
for fname in glob.glob("./Routes/*.route"):
    #mission.clear()
    mission = read_route(fname)
    if mission_distance_check( mission ):
        search_rslt = True
        break       # Exit for

#3 機体に転送
if search_rslt:
    #for debug
    print_mission( mission )
    upload_mission( mission )
else:
    vehicle.commands.clear()
    print( "No route found\n")
'''



'''
# 以下、mission_import_exportの残り
import_mission_filename = 'mpmission.txt'
export_mission_filename = 'exportedmission.txt'


#Upload mission from file
upload_mission(import_mission_filename)

#Download mission we just uploaded and save to a file
save_mission(export_mission_filename)

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()


print("\nShow original and uploaded/downloaded files:")
#Print original file (for demo purposes only)
printfile(import_mission_filename)
#Print exported file (for demo purposes only)
printfile(export_mission_filename)
'''
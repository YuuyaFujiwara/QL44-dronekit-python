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


# Check that vehicle is armable. 
# This ensures home_location is set (needed when saving WP file)




'''#Armableを待たない
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
'''


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
    print( "mission_distance = {0}".format(dist) )
    if dist < 10.0:
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
    '''
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    '''
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def download_and_save_mission(aFileName):
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
        

def save_mission(mission, aFileName):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)    
    """
    #Download mission from vehicle
    missionlist = download_mission()
    """
    #Add file-format information
    output='QGC WPL 110\n'

    # とりあえずhome_locatrionは保存しない。
    '''
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    '''

    #Add commands
    for cmd in mission:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)





def print_mission( aMission ):
    for cmd in aMission:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        print( commandline )


        
def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip())        

# フォルダを作成
# すでに存在する場合は何もしない。
# 深い階層のフォルダも作成するはず。(os.makedirs()を使用しているので)
# 注意：親フォルダの所有者がrootの場合、失敗する。
def my_makedirs(path):
    if not os.path.isdir(path):
        os.makedirs(path)

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

'''
#メッセージ受信テスト
#デコレーターを使ってリスナーと作成する
#@vehicle.on_message(['MISSION_ACK', 'MISSION_CLEAR_ALL', 'MISSION_COUNT', 'MISSION_CURRENT', 'MISSION_ITEM', 'MISSION_ITEM_INT', 'MISSION_ITEM_REACHED', 'MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_REQUEST_LIST', 'MISSION_REQUEST_PARTIAL_LIST', 'MISSION_SET_CURRENT', 'MISSION_WRITE_PARTIAL_LIST'])
#print("\nAdd @vehicle.on_message( 'HEARTBEAT' )  using orator") 
#@vehicle.on_message( 'HEARTBEAT' )
print("\nAdd @vehicle.on_message( 'MISSION_*' )  using orator") 
@vehicle.on_message(['MISSION_ACK', 'MISSION_CLEAR_ALL', 'MISSION_COUNT', 'MISSION_CURRENT', 'MISSION_ITEM', 'MISSION_ITEM_INT', 'MISSION_ITEM_REACHED', 'MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_REQUEST_LIST', 'MISSION_REQUEST_PARTIAL_LIST', 'MISSION_SET_CURRENT', 'MISSION_WRITE_PARTIAL_LIST'])
def listener(self, name, message):
	print( " message = ", name,  ", value =", message )
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
    global global_flg_route_save_req
    global global_flg_route_clear_req

    #global global_flg_route_delete_req

    print(" CALLBACK: 'MOMIMAKI_RT_CTRL' changed to", value)

    # パラメータ値により動作を分ける
    if value == 0 :
        print(" value == zero" )
    elif value == 9999 :
        # 全ルートクリア
        print(" clear all route files(not yet)" )
	global_flg_route_clear_req = True
    elif value == 1:
        print(" value == 1st" )
    elif value == 2:
        print(" value == 2nd" )
        # 現在のルートをファイルに保存
        print(" save to file current route(not yet)" )
        global_flg_route_save_req = True
    elif value == 3:
        print(" value == 3rd" )
    else :
        print(" value == ", value )
'''
    # 値を戻す
    if value != -1:
        vehicle.parameters['MOMIMAKI_RT_CTRL'] = -1
'''





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
    # ModeがHOLDになった場合に、ルートを決める。
    # ルートが決まらなかったら、STABILIZEにモードを変える。
    if value==VehicleMode("HOLD"):
        global_flg_route_select_req = True


#
# 自動ルート選択処理
#
def route_select_proc():
    #global global_flg_route_select_req

    # ルートファイルを検索
    search_rslt = False
    mission = []
    for fname in glob.glob( route_path +  "/*" + ROUTEFILE_EXT ):  
        mission = read_route(fname)
        if mission_distance_check( mission ):
            search_rslt = True
            break       # Exit for

    # 機体に転送
    if search_rslt:
        #for debug
        print_mission( mission )
        upload_mission( mission )

        # home location設定してみる
        set_home_loc( mission[0].y, mission[0].x  )

    else:
        print( "No route found\n")
        vehicle.commands.clear()
        vehicle.mode = VehicleMode("MANUAL")

    


#
# ルート保存処理
# 船体内の現在のルートをファイル名をつけて保存する。
#
def route_save_proc():

    #機体からダウンロード
    mission = download_mission()
    print_mission( mission )    #for debug

    #ファイル名を決める]
    cmd = mission[0]
    fname = route_path +  "/rt{0}_{1}{2}".format( cmd.y, cmd.x, ROUTEFILE_EXT )
    print( "save filename = {0}\n".format(fname) )

    #ルートファイル保存
    save_mission( mission, fname )

#
# ルートファイル削除処理
#
def route_clear_proc():
    # ファイル削除ループ
    for fname in glob.glob( route_path +  "/*" + ROUTEFILE_EXT ):  
        print( "{0} remove.".format(fname))
        if os.path.isfile(fname):
            os.remove(fname)
            print( "{0} removed.".format(fname))




# home location設定する。
def set_home_loc( alat, alng ):
    tmploc = vehicle.location.global_frame
    tmploc.lat = alat
    tmploc.lon = alng
    #tmploc.alt = 100
    vehicle.home_location = tmploc
    print( "vehicle.home_location = ({0},{1},{2})".format(vehicle.home_location.lat, vehicle.home_location.lon, vehicle.home_location.alt ) )




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

# 保存フォルダ作成
my_makedirs(route_path)


# 初期化
vehicle.parameters['MOMIMAKI_RT_CTRL'] = -1








# for debug 
# とりあえず無限ループ
while True:
#global変数
    global global_flg_route_select_req
    global global_flg_route_save_req
    global global_flg_route_clear_req

    # フラグが立ったらルート選択処理を行う
    if global_flg_route_select_req==True:
        route_select_proc()
        global_flg_route_select_req = False     # flag reset
        vehicle.parameters['MOMIMAKI_RT_CTRL'] = -1  # 


    # フラグが立ったらルート保存処理を行う
    if global_flg_route_save_req==True:
        route_save_proc()
        global_flg_route_save_req = False     # flag reset
        vehicle.parameters['MOMIMAKI_RT_CTRL'] = -1  # 


    # フラグが立ったらルートファイル削除処理を行う
    if global_flg_route_clear_req==True:
        route_clear_proc()
        global_flg_route_clear_req = False     # flag reset
        vehicle.parameters['MOMIMAKI_RT_CTRL'] = -1  # 



    #print("loop.")
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
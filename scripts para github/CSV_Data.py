##: ESTE SCRIPT REALIZA EL ATERRIZAJE DEL DRON EN EL MARCADOR ARUCO ADEMÁS DE GUARDAR LOS DATOS DE LA POSICIÓN DEL DRON EN UN ARCHIVO.CSV PARA FACILITAR SU ANÁLISIS
##: A LO LARGO DEL SCRIPT SE IRÁ COMENTANDO COMO FUNCIONA CADA PARTE
##: ---------------------------------------------------------------
##: THIS SCRIPT PERFORMS LANDING IN ARUCO MARKER. ALSO IT SAVES LOCATION DATA IN A .CSV FILE TO EASE DATA TREATMENT
##: THROUGH ALL THE SCRIPT COMMENTS WILL EXPLAIN HOW IT WORKS


from os import path, sys

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import argparse
import math
import time
import csv

import airsim
import cv2
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from cv2 import aruco
from dronekit import (Command, LocationGlobal, LocationGlobalRelative,
                      VehicleMode, connect)
from pymavlink import mavutil

from lib_aruco_pose import *

##: CONEXION DEL DRON 
csvFile =  'misiontiempo.csv'
excelFile = 'excel65.csv'

def _csv_write(linea,nombreArchivo):
    with open(nombreArchivo, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([linea])

_csv_write("LAT        LON        ALT    YAW",excelFile)

def connectionDroneKit(port):
    connection_string       = port
    MAV_MODE_AUTO   = 4
    # https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py


    # Parse connection argument
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connect", help="connection string")
    args = parser.parse_args()

    if args.connect:
        connection_string = args.connect


    ################################################################################################
    # Init
    ################################################################################################




    # Connect to the Vehicle
    print ("Connecting")
    _csv_write("Connecting",csvFile)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle
 
vehicle = connectionDroneKit('127.0.0.1:14540')

def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

##: FUNCION PARA OBTENER UNA LOCALIZACIÓN CON UN OFFSET DETERMINADO DESDE UNA LOCALIZACION YA EXISTENTE

def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
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
    return LocationGlobal(newlat, newlon,original_location.alt+alt)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
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
    return(newlat, newlon)

##: FUNCION PARA OBTENER LOS ANGULOS DEL AMRCADOR

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)

##: FUNCION QUE GIRA X E Y DE LA CÁMARA PARA TENER LOS MISMOS EJES QUE EN EL UAV

def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
##: FUNCION QUE PASA LAS COORDENADAS OBTENIDAS EN OPENCV A COORDENADAS NE 

def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)


def check_angle_descend(angle_x, angle_y):
    return(math.sqrt(angle_x**2 + angle_y**2))
 

home_position_set = False

# Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True




# wait for a home position lock
while not home_position_set:
    print ("Waiting for home position...")
    _csv_write("Waiting for home position...",csvFile)
    time.sleep(1)

# Display basic vehicle state

print ("Type: %s" % vehicle._vehicle_type)
_csv_write("Type: %s" % vehicle._vehicle_type,csvFile)
print (" Armed: %s" % vehicle.armed)
_csv_write(" Armed: %s" % vehicle.armed,csvFile)
print ("System status: %s" % vehicle.system_status.state,csv)
_csv_write(" System status: %s" % vehicle.system_status.state,csvFile)
print ("GPS: %s" % vehicle.gps_0)
_csv_write("GPS: %s" % vehicle.gps_0,csvFile)

print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
_csv_write("Latitud Inicial: %s" % vehicle.location.global_relative_frame.lat,csvFile)
print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
_csv_write("Longitud Inicial: %s" % vehicle.location.global_relative_frame.lon,csvFile)
print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
_csv_write("Altura Inicial: %s" % vehicle.location.global_relative_frame.alt,csvFile)
print( "Ángulo de Guiñada Inicial: %.2f" %vehicle.attitude.yaw)

# Change to AUTO mode
MAV_MODE_AUTO   = 4
PX4setMode(MAV_MODE_AUTO)

##: COMO EN PX4 NO FUNCIONAN ORDENES COMO simple.goto O vehicle.mode, HAY QUE UTILIZAR LOS COMMANDS, QUE PERMITEN HACER LAS MISMAS FUNCIONES PERO SU FUNCIONAMIENTO TIENE BUGS


# Load commands
cmds = vehicle.commands
cmds.clear()
cmds.upload()
home = vehicle.location.global_relative_frame


#--- Define Tag
id_to_find  = 999
marker_size  = 100 #- [cm]

ultima_altura= vehicle.location.global_relative_frame.alt



tiempo_mision = 0


freq_send = 1 # Hz
time_0=time.time()
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

land_alt_m         = 5.05 #m

#--- Get the camera calibration path
calib_path  = "/home/alberto/OPENCV__AS/"
camera_matrix   = np.loadtxt(calib_path+'camera_matrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'camera_dist.txt', delimiter=',')
camera_name = 'front_center_custom'


##: LLAMAMOS A LA LIBRERIA CREADA ANTERIORMENTE PARA OBTENER LA IMAGEN
aruco_tracker = ArucoSingleTracker(id_to_find, marker_size, camera_matrix=camera_matrix, camera_distortion=camera_distortion,camera_name=camera_name,show_video=True)

while True:    
    ##: MOSTRAMOS EN UN BUCLE WHILE CADA FRAME DE LA IMAGEN QUE VAMOS OBTENIENDO            
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False,verbose=False)
    if (marker_found==True): 

        inicio_mision = time.perf_counter()
        print("Marcador encontrado")
        _csv_write("Marcador encontrado",csvFile)
        ##_ INTERRUMPE LA MISION DE QGC

    
        PX4setMode(MAV_MODE_AUTO)
        MAV_MODE_AUTO   = 4

        x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
                ##: GUARDAMOS LA POSICION INCIAL DEL DRON
        uav_location_initial        = vehicle.location.global_relative_frame
        z_cm = uav_location_initial.alt
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

        north, east             = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)

        marker_lat, marker_lon  = get_location_metres(uav_location_initial, north*0.01, east*0.01)
        
        if time.time() >= time_0 + 1.0/freq_send and uav_location_initial.alt >=land_alt_m + 0.5:
                # coordenadas del centro del marcador

            ##: OBTENEMOS LA LOCALIZACION DEL MARCADOR COMO OBJETO LOCATION  
            time_0=time.time()

            location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location_initial.alt)

            wp = LocationGlobal (location_marker.lat, location_marker.lon, uav_location_initial.alt)

            cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 1, 0, 0, 0, wp.lat,wp.lon,wp.alt)
            cmds.add(cmd)
            print ("Colocando en   Lat = %.7f  Lon = %.7f Alt=%.2f Yaw = 0"%(wp.lat, wp.lon,wp.alt))
            _csv_write("Colocando en   Lat =%.7f  Lon =%.7f Alt=%.2f Yaw=0"%(wp.lat, wp.lon,wp.alt),csvFile)
            uav_location_initial=vehicle.location.global_relative_frame

            bajada = uav_location_initial.alt/2 #metres
            wp = get_location_offset_meters (location_marker,0,0,-bajada)
            cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 1, 0, 0, 0, wp.lat,wp.lon,wp.alt)
            cmds.add(cmd)
            print ("Bajando a   Lat = %.7f  Lon = %.7f Alt=%.2f Yaw = 0"%(wp.lat, wp.lon,wp.alt))
            _csv_write("Bajando a   Lat =%.7f  Lon =%.7f Alt=%.2f Yaw=0"%(wp.lat, wp.lon,wp.alt),csvFile) 
            suelo = 0-uav_location_initial.alt
            wp = get_location_offset_meters(location_marker,0,0,suelo)
            cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 1, 0, 0, 0, wp.lat,wp.lon,0)
            cmds.add(cmd)
            print ("Aterrizando en   Lat = %.7f  Lon = %.7f Alt=%.2f Yaw= 0"%(wp.lat, wp.lon,wp.alt))
            _csv_write("Aterrizando en   Lat =%.7f  Lon =%.7f Alt=%.2f Yaw=0"%(wp.lat, wp.lon,wp.alt),csvFile)

            cmds.upload()
            print("numero de comandos : %.0f" %vehicle.commands.count)
            _csv_write("numero de comandos : %.0f" %vehicle.commands.count,csvFile)
            # monitor mission execution
            nextwaypoint = vehicle.commands.next
            while nextwaypoint < len(vehicle.commands):
                marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False,verbose=False)
                x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
                        ##: GUARDAMOS LA POSICION INCIAL DEL DRON
                uav_location_initial        = vehicle.location.global_relative_frame
                z_cm = uav_location_initial.alt
                angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

                north, east             = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)

                marker_lat, marker_lon  = get_location_metres(uav_location_initial, north*0.1, east*0.1)

                location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location_initial.alt)




                print ("Commanding to   Lat = %.7f  Lon = %.7f Alt=%.2f"%(location_marker.lat, location_marker.lon,location_marker.alt))
                _csv_write("Commanding to   Lat = %.7f  Lon = %.7f Alt=%.2f Yaw=0"%(location_marker.lat, location_marker.lon,location_marker.alt),csvFile)
                print ("UAV Location    Lat = %.7f  Lon = %.7f Alt=%.2f"%(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt))
                _csv_write("UAV Location    Lat = %.7f  Lon = %.7f Alt=%.2f Yaw=%.2f"%(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt,vehicle.attitude.yaw),csvFile)
                _csv_write("%.7f   %.7f   %.2f   %.2f" %(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt,vehicle.attitude.yaw*rad_2_deg), excelFile)
                if vehicle.commands.next > nextwaypoint:
                    display_seq = vehicle.commands.next+1
                    print ("Moving to waypoint %s" % display_seq)
                    _csv_write ("Moving to waypoint %s" % display_seq,csvFile)
                    nextwaypoint = vehicle.commands.next

                if vehicle.location.global_relative_frame.alt < 0.1:
                    tiempo_mision = time.perf_counter()
                if tiempo_mision -inicio_mision >=50:
                    print ("FIN")
                    break;

            # wait for the vehicle to positioning
            while vehicle.commands.next > 0:
                marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False,verbose=False)
                break





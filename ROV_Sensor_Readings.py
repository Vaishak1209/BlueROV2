import time
import sys
import csv
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('udpin:localhost:14540')

csv_filename = 'sensor_data.csv'
csv_file = open(csv_filename, 'w', newline='')
csv_writer = csv.writer(csv_file)

csv_writer.writerow(['Timestamp', 'GyroX', 'GyroY', 'GyroZ', 'AccX', 'AccY', 'AccZ', 'MagX', 'MagY', 'MagZ', 'Pressure'])


the_connection.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


message = the_connection.mav.command_long_encode(
        the_connection.target_system,  # Target system ID
        the_connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO, # param1: Stream 3d gyro sensor data
        1000000, # 1 second delay
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL,  # param2: Stream 3d accelerometer data
        1000000,
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG, # param3: Stream 3d magnetometer data
        1000000,
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, # param4: Stream abs pressure data
        1000000,
        )

the_connection.mav.send(message)
while True:
    # Wait for a message from the vehicle
    msg = the_connection.recv_match()

    if msg is None:
        print("ROV not connected")
        continue

    if msg and msg.command == mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        accel_x = msg.xacc
        accel_y = msg.yacc
        accel_z = msg.zacc

        print("Accelerometer (m/s^2): X={}, Y={}, Z={}".format(accel_x, accel_y, accel_z))
        csv_writer.writerow(['', '', '', '', accel_x, accel_y, accel_z, '', '', '', ''])
    
    elif msg and msg.command == mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        gyro_x = msg.xgyro
        gyro_y = msg.ygyro
        gyro_z = msg.zgyro

        print("Gyroscope (rad/s): X={}, Y={}, Z={}".format(gyro_x, gyro_y, gyro_z))
        csv_writer.writerow(['', gyro_x, gyro_y, gyro_z, '', '', '', '', '', '', ''])

    elif msg and msg.command == mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        pressure_abs = msg.press_abs
        print("Absolute pressure (Pa) = {}".format(pressure_abs))
        csv_writer.writerow(['', '', '', '', '', '', '', '', '', '', pressure_abs])

    elif msg and msg.command == mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        mag_value_x = msg.xmag
        mag_value_y = msg.ymag
        mag_value_z = msg.zmag
        print("Magnetic field value (T) = X={}, Y={}, Z={}".format(mag_value_x, mag_value_y, mag_value_z))  
        csv_writer.writerow(['', '', '', '', '', '', '', mag_value_x, mag_value_y, mag_value_z, ''])

    csv_file.flush()

csv_file.close()
        

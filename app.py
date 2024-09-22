import math
import numpy as np
import pandas as pd
import re
import folium
import base64
from threading import Thread
from flask import Flask, request, jsonify, render_template_string, send_file
import logging
import os
import io
import paho.mqtt.client as mqtt
import json
import IMU  # Importing the IMU library for Raspberry Pi
import gpsd
import datetime  # Importing the GPSD library
import time

# Global variables
target_coordinates = []
waypoints = []
current_waypoint_index = 0
e_stop_active = False
gps_data = []

# MQTT Configuration
MQTT_SERVER = "192.168.1.120"  # Replace with your MQTT server IP
MQTT_PORT = 1883
MQTT_TOPIC_IMU = "imu/data"
MQTT_TOPIC_COMMAND = "robot/control"

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT server {MQTT_SERVER} successfully.")
    else:
        print(f"Failed to connect, return code {rc}")

client = mqtt.Client()
client.on_connect = on_connect
client.connect(MQTT_SERVER, MQTT_PORT, 60)
client.loop_start()

# Constants
RAD_TO_DEG = 57.29578
G_GAIN = 0.070  # [deg/s/LSB]
AA = 0.40  # Complementary filter constant
M_PI = 3.14159265358979323846

# Compass Calibration values (adjust these with your own calibration data)
magXmin, magYmin, magZmin = 0, 0, 0
magXmax, magYmax, magZmax = 0, 0, 0

# Kalman filter variables
Q_angle, Q_gyro, R_angle = 0.02, 0.0015, 0.005
y_bias, x_bias = 0.0, 0.0
XP_00, XP_01, XP_10, XP_11 = 0.0, 0.0, 0.0, 0.0
YP_00, YP_01, YP_10, YP_11 = 0.0, 0.0, 0.0, 0.0
KFangleX, KFangleY = 0.0, 0.0

# Initialize IMU
IMU.detectIMU()
if IMU.BerryIMUversion == 99:
    print("No BerryIMU found... exiting")
    sys.exit()
IMU.initIMU()

gyroXangle, gyroYangle, gyroZangle = 0.0, 0.0, 0.0
CFangleX, CFangleY = 0.0, 0.0
kalmanX, kalmanY = 0.0, 0.0

# Initialize GPSD
gpsd.connect()

app = Flask(__name__)

def recieve_gps_data():
    global current_lat, current_lon
    try:
        packet = gpsd.get_current()
        if packet.mode >= 2:
            current_lat = packet.lat
            current_lon = packet.lon
            print(f"{current_lat},{current_lon}")
            return current_lat, current_lon
        else:
            return None, None
    except Exception as e:
        logging.error(f"Error retrieving GPS data: {e}")
        return None, None

def kalmanFilterY(accAngle, gyroRate, DT):
    global KFangleY, y_bias, YP_00, YP_01, YP_10, YP_11
    KFangleY += DT * (gyroRate - y_bias)
    YP_00 += -DT * (YP_10 + YP_01) + Q_angle * DT
    YP_01 += -DT * YP_11
    YP_10 += -DT * YP_11
    YP_11 += Q_gyro * DT

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0, K_1 = YP_00 / S, YP_10 / S

    KFangleY += K_0 * y
    y_bias += K_1 * y

    YP_00 -= K_0 * YP_00
    YP_01 -= K_0 * YP_01
    YP_10 -= K_1 * YP_00
    YP_11 -= K_1 * YP_01

    return KFangleY

def kalmanFilterX(accAngle, gyroRate, DT):
    global KFangleX, x_bias, XP_00, XP_01, XP_10, XP_11
    KFangleX += DT * (gyroRate - x_bias)
    XP_00 += -DT * (XP_10 + XP_01) + Q_angle * DT
    XP_01 += -DT * XP_11
    XP_10 += -DT * XP_11
    XP_11 += Q_gyro * DT

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0, K_1 = XP_00 / S, XP_10 / S

    KFangleX += K_0 * x
    x_bias += K_1 * x

    XP_00 -= K_0 * XP_00
    XP_01 -= K_0 * XP_01
    XP_10 -= K_1 * XP_00
    XP_11 -= K_1 * XP_01

    return KFangleX

def calculate_heading():
    global gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, kalmanX, kalmanY
    a = datetime.datetime.now()

    # Read sensor data
    ACCx, ACCy, ACCz = IMU.readACCx(), IMU.readACCy(), IMU.readACCz()
    GYRx, GYRy, GYRz = IMU.readGYRx(), IMU.readGYRy(), IMU.readGYRz()
    MAGx, MAGy, MAGz = IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()

    # Apply compass calibration
    MAGx -= (magXmin + magXmax) / 2
    MAGy -= (magYmin + magYmax) / 2
    MAGz -= (magZmin + magZmax) / 2

    # Calculate loop period
    b = datetime.datetime.now() - a
    a = datetime.datetime.now()
    LP = b.microseconds / (1000000 * 1.0)

    # Gyro to degrees per second
    rate_gyr_x, rate_gyr_y, rate_gyr_z = GYRx * G_GAIN, GYRy * G_GAIN, GYRz * G_GAIN

    # Gyro angles
    gyroXangle += rate_gyr_x * LP
    gyroYangle += rate_gyr_y * LP
    gyroZangle += rate_gyr_z * LP

    # Accelerometer angles
    AccXangle = math.atan2(ACCy, ACCz) * RAD_TO_DEG
    AccYangle = (math.atan2(ACCz, ACCx) + M_PI) * RAD_TO_DEG
    AccYangle = AccYangle - 270.0 if AccYangle > 90 else AccYangle + 90.0

    # Complementary filter
    CFangleX = AA * (CFangleX + rate_gyr_x * LP) + (1 - AA) * AccXangle
    CFangleY = AA * (CFangleY + rate_gyr_y * LP) + (1 - AA) * AccYangle

    # Kalman filter
    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y, LP)
    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x, LP)

    # Heading
    heading = 180 * math.atan2(MAGy, MAGx) / M_PI
    heading += 360 if heading < 0 else 0

    # Tilt compensated heading
    accXnorm = ACCx / math.sqrt(ACCx**2 + ACCy**2 + ACCz**2)
    accYnorm = ACCy / math.sqrt(ACCx**2 + ACCy**2 + ACCz**2)

    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm / math.cos(pitch))

    if IMU.BerryIMUversion in [1, 3]:
        # LSM9DS0 and LSM6DSL & LIS2MDL
        magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
        magYcomp = MAGx * math.sin(roll) * math.sin(pitch) + MAGy * math.cos(roll) - MAGz * math.sin(roll) * math.cos(pitch)
    else:
        # LSM9DS1
        magXcomp = MAGx * math.cos(pitch) - MAGz * math.sin(pitch)
        magYcomp = MAGx * math.sin(roll) * math.sin(pitch) + MAGy * math.cos(roll) + MAGz * math.sin(roll) * math.cos(pitch)

    tiltCompensatedHeading = 180 * math.atan2(magYcomp, magXcomp) / M_PI
    tiltCompensatedHeading += 360 if tiltCompensatedHeading < 0 else 0

    return heading, tiltCompensatedHeading, gyroXangle, gyroYangle, gyroZangle, kalmanX, kalmanY, CFangleX, CFangleY

def calculate_target_heading(current_lat, current_lon, target_lat, target_lon):
    delta_lon = math.radians(target_lon - current_lon)
    lat1 = math.radians(current_lat)
    lat2 = math.radians(target_lat)
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

def normalize_angle(angle):
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle

def find_lookahead_point(current_lat, current_lon, waypoints, lookahead_distance):
    # Convert degrees to radians for calculation
    current_pos = np.array([math.radians(current_lat), math.radians(current_lon)])

    for i in range(len(waypoints) - 1):
        start_wp = np.array([math.radians(waypoints[i][0]), math.radians(waypoints[i][1])])
        end_wp = np.array([math.radians(waypoints[i+1][0]), math.radians(waypoints[i+1][1])])

        # Calculate the vector from the current position to the waypoints
        d = end_wp - start_wp
        f = start_wp - current_pos
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - lookahead_distance**2
        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            continue  # No intersection

        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)

        if 0 <= t1 <= 1:
            intersection = start_wp + t1 * d
            return [math.degrees(intersection[0]), math.degrees(intersection[1])]
        if 0 <= t2 <= 1:
            intersection = start_wp + t2 * d
            return [math.degrees(intersection[0]), math.degrees(intersection[1])]

    return None  # No valid lookahead point found

def calculate_steering_angle(current_lat, current_lon, current_heading, lookahead_point):
    # Calculate the angle between the robot's heading and the lookahead point
    target_heading = calculate_target_heading(current_lat, current_lon, lookahead_point[0], lookahead_point[1])
    steering_angle = normalize_angle(target_heading - current_heading)
    return steering_angle

def track_robot():
    global gps_data, gps_connected, target_coordinates, waypoints, current_waypoint_index, e_stop_active
    gps_data = []
    path_coordinates = []

    # Ensure waypoints are available
    if not waypoints or len(waypoints) == 0:
        logging.error("No waypoints provided.")
        return

    try:
        while True:
            if e_stop_active:
                client.publish(MQTT_TOPIC_COMMAND, "STOP")
                time.sleep(1)
                continue  # Skip the rest if E-Stop is active

            current_lat, current_lon = recieve_gps_data()
            if current_lat is not None and current_lon is not None:
                heading, tiltCompensatedHeading, _, _, _, _, _, _, _ = calculate_heading()
                gps_data.append({
                    'GPS_Lat': current_lat,
                    'GPS_Lon': current_lon,
                    'Heading': heading
                })
                path_coordinates.append([current_lat, current_lon])

                # Pure Pursuit logic
                lookahead_distance = 0.0001  # Adjust as necessary (in degrees)
                lookahead_point = find_lookahead_point(current_lat, current_lon, waypoints, lookahead_distance)

                if lookahead_point is None:
                    logging.info("Reached the end of the path.")
                    client.publish(MQTT_TOPIC_COMMAND, "STOP")
                    break

                # Calculate steering angle to the lookahead point
                steering_angle = calculate_steering_angle(current_lat, current_lon, heading, lookahead_point)

                # Determine command based on the steering angle
                if steering_angle > 5:
                    command = "3"
                elif steering_angle < -5:
                    command = "4"
                else:
                    command = "1"

                client.publish(MQTT_TOPIC_COMMAND, command)
                logging.info(f"Command: {command} | Steering Angle: {steering_angle}")
            else:
                logging.warning("No valid GPS data received.")

            gps_connected = True
            time.sleep(0.1)
    except Exception as e:
        logging.error(f"Error tracking robot: {e}")
        gps_connected = False

def imu_thread():
    while True:
        heading, tiltCompensatedHeading, gyroXangle, gyroYangle, gyroZangle, kalmanX, kalmanY, CFangleX, CFangleY = calculate_heading()
        data = {
            "heading": heading,
            "tiltCompensatedHeading": tiltCompensatedHeading,
            "gyroXangle": gyroXangle,
            "gyroYangle": gyroYangle,
            "gyroZangle": gyroZangle,
            "kalmanX": kalmanX,
            "kalmanY": kalmanY,
            "CFangleX": CFangleX,
            "CFangleY": CFangleY
        }
        client.publish(MQTT_TOPIC_IMU, json.dumps(data))
        time.sleep(0.03)

@app.route('/')
def index():
    return render_template_string(html_content)

@app.route('/get_gps_data', methods=['GET'])
def get_gps_data():
    sanitized_gps_data = [{k: float(v) if isinstance(v, (np.float32, np.float64, np.int32, np.int64, float, int)) else v for k, v in item.items()} for item in gps_data]
    return jsonify(sanitized_gps_data)

@app.route('/send_coordinates', methods=['POST'])
def receive_coordinates():
    global e_stop_active, waypoints, current_waypoint_index, target_coordinates
    e_stop_active = False
    data = request.get_json()
    coordinates = data.get('coordinates', [])
    logging.info(f"Received coordinates: {coordinates}")
    waypoints = [(point['lat'], point['lng']) for point in coordinates]
    current_waypoint_index = 0
    if waypoints:
        target_coordinates = waypoints[0]
    Thread(target=track_robot).start()
    return jsonify({"status": "OK"})

@app.route('/estop', methods=['POST'])
def emergency_stop():
    global e_stop_active
    e_stop_active = True
    client.publish(MQTT_TOPIC_COMMAND, "STOP")
    print("E-Stop Activated!")
    return jsonify({"status": "E-Stop activated"})

@app.route('/undo_estop', methods=['POST'])
def undo_emergency_stop():
    global e_stop_active
    e_stop_active = False
    print("E-Stop Deactivated!")
    return jsonify({"status": "E-Stop deactivated"})

@app.route('/initial_gps', methods=['GET'])
def initial_gps():
    lat, lon = recieve_gps_data()  # Get the initial GPS coordinates from the Raspberry Pi's GPS module
    return jsonify({"lat": lat, "lon": lon})

@app.route('/get_robot_icon', methods=['GET'])
def get_robot_icon():
    try:
        with open('robot_icon.png', 'rb') as image_file:
            encoded_image = base64.b64encode(image_file.read()).decode('utf-8')
        return jsonify({"encoded_image": encoded_image})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# HTML content without PID chart and controls
html_content = '''
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
    <style>
        #map { height: 600px; width: 100%; }
        #controls { margin-top: 20px; }
        button { margin: 5px; }
    </style>
</head>
<body>
    <h1>Robot Control</h1>
    <div id="map"></div>
    <div id="controls">
        <button id="sendButton">Send Command</button>
        <button id="estopButton">E-Stop</button>
        <button id="undoEstopButton">Undo E-Stop</button>
    </div>

    <!-- Leaflet JS and Leaflet.Draw CSS/JS -->
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
    <!-- Include Leaflet Draw -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.js"></script>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.css" />

    <script>
    // Initialize map
    var map = L.map('map').setView([0, 0], 2);

    // Add tile layer to map
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    // Get initial GPS location
    fetch('/initial_gps')
        .then(response => response.json())
        .then(data => {
            var initialLat = data.lat || 0;
            var initialLon = data.lon || 0;
            map.setView([initialLat, initialLon], 18);
        });

    // Add draw control to the map
    var drawnItems = new L.FeatureGroup();
    map.addLayer(drawnItems);

    var drawControl = new L.Control.Draw({
        edit: {
            featureGroup: drawnItems
        }
    });
    map.addControl(drawControl);

    // Save drawing coordinates
    map.on(L.Draw.Event.CREATED, function (event) {
        var layer = event.layer;
        drawnItems.addLayer(layer);
    });

    // Send drawn coordinates to server
    document.getElementById('sendButton').onclick = function() {
        var coordinates = [];
        drawnItems.eachLayer(function(layer) {
            if (layer instanceof L.Polyline || layer instanceof L.Polygon) {
                var latLngs = layer.getLatLngs();
                if (latLngs.length > 0 && Array.isArray(latLngs[0])) {
                    latLngs.forEach(function(latlngArray) {
                        latlngArray.forEach(function(latlng) {
                            coordinates.push({ lat: latlng.lat, lng: latlng.lng });
                        });
                    });
                } else {
                    latLngs.forEach(function(latlng) {
                        coordinates.push({ lat: latlng.lat, lng: latlng.lng });
                    });
                }
            } else if (layer instanceof L.Marker) {
                var latlng = layer.getLatLng();
                coordinates.push({ lat: latlng.lat, lng: latlng.lng });
            }
        });

        fetch('/send_coordinates', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ coordinates: coordinates })
        })
        .then(response => response.json())
        .then(data => {
            console.log(data);
            alert("Coordinates sent to the server");
        })
        .catch(error => console.error('Error:', error));
    };

    // E-Stop functionality
    document.getElementById('estopButton').onclick = function() {
        fetch('/estop', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        .then(response => response.json())
        .then(data => {
            console.log(data);
            alert("E-Stop activated");
        })
        .catch(error => console.error('Error:', error));
    };

    // Undo E-Stop functionality
    document.getElementById('undoEstopButton').onclick = function() {
        fetch('/undo_estop', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        .then(response => response.json())
        .then(data => {
            console.log(data);
            alert("E-Stop deactivated");
        })
        .catch(error => console.error('Error:', error));
    };

    // Create robot marker
    var robotMarker = L.marker([0, 0]).addTo(map);

    // Fetch GPS data and update the map every second
    setInterval(function() {
        fetch('/get_gps_data')
            .then(response => response.json())
            .then(data => {
                if (data && data.length > 0) {
                    var gpsCoordinates = data.map(function(point) {
                        return [point.GPS_Lat, point.GPS_Lon];
                    });

                    // Update robot marker position
                    var latestPosition = gpsCoordinates[gpsCoordinates.length - 1];
                    robotMarker.setLatLng(latestPosition);

                    // Optionally, update polyline showing path
                    // You can add code here to draw the path
                }
            })
            .catch(error => console.error('Error fetching GPS data:', error));
    }, 1000);
    </script>

</body>
</html>
'''

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    Thread(target=imu_thread).start()
    app.run(host='0.0.0.0', port=8000)

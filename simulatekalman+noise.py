import math
import numpy as np
from threading import Thread, Lock
from flask import Flask, request, jsonify, render_template_string
import logging
import time
import random

# Global variables
target_coordinates = []
waypoints = []
current_waypoint_index = 0
e_stop_active = False
gps_data = []
gps_data_lock = Lock()
current_lat, current_lon = 35.8486, -86.3669  # Initial simulated position (e.g., London)
robot_speed = 0.00001  # Simulated speed in degrees
dt = 0.1  # Time step in seconds

# Kalman filter variables
state = np.zeros((5, 1))  # [x, y, vx, vy, heading]
P = np.eye(5) * 0.1       # Covariance matrix
Q = np.eye(5) * 0.01      # Process noise covariance
R_gps = np.eye(2) * 0.5   # GPS measurement noise covariance
R_imu = np.eye(2) * 0.1   # IMU measurement noise covariance

app = Flask(__name__)

# Function to simulate IMU data
def simulate_imu_data(acceleration, angular_velocity):
    # Add some noise to the simulated IMU data
    ax = acceleration[0] + random.gauss(0, 0.01)
    ay = acceleration[1] + random.gauss(0, 0.01)
    omega = angular_velocity + random.gauss(0, 0.1)
    return ax, ay, omega

# Function to simulate GPS data
def simulate_gps_data():
    global current_lat, current_lon, robot_speed, current_waypoint_index, gps_data, state, P
    if current_waypoint_index < len(waypoints):
        target_lat, target_lon = waypoints[current_waypoint_index]

        # Convert lat/lon to meters using a simple equirectangular projection
        x_current, y_current = latlon_to_xy(current_lat, current_lon)
        x_target, y_target = latlon_to_xy(target_lat, target_lon)

        # Calculate desired velocity towards the target
        dx = x_target - x_current
        dy = y_target - y_current
        distance = math.hypot(dx, dy)

        if distance > 0:
            vx = (dx / distance) * robot_speed * (1 / dt)
            vy = (dy / distance) * robot_speed * (1 / dt)
        else:
            vx, vy = 0, 0

        # Simulate IMU data (acceleration and angular velocity)
        # For simplicity, assume constant velocity; acceleration is zero
        ax_true = 0
        ay_true = 0
        omega_true = 0  # No change in heading

        ax, ay, omega = simulate_imu_data([ax_true, ay_true], omega_true)

        # Predict step of Kalman Filter
        kalman_predict(ax, ay, omega)

        # Simulate GPS measurement with noise
        gps_lat = current_lat + random.gauss(0, 0.00001)
        gps_lon = current_lon + random.gauss(0, 0.00001)

        # Update step of Kalman Filter with GPS data
        kalman_update_gps(gps_lat, gps_lon)

        # Update the current position with Kalman filter estimation
        x_est, y_est = state[0, 0], state[1, 0]
        current_lat, current_lon = xy_to_latlon(x_est, y_est)

        # Add simulated GPS data to list
        with gps_data_lock:
            gps_data.append({
                'GPS_Lat': current_lat,
                'GPS_Lon': current_lon,
                'Heading': state[4, 0]
            })

        # Check if the robot has reached the target waypoint
        if calculate_distance(x_current, y_current, x_target, y_target) < 1:
            print(f"Reached waypoint {current_waypoint_index}")
            current_waypoint_index += 1

    return current_lat, current_lon

# Kalman filter predict step
def kalman_predict(ax, ay, omega):
    global state, P, Q, dt
    F = np.eye(5)
    F[0, 2] = dt
    F[1, 3] = dt
    F[4, 4] = 1  # Heading remains the same in prediction

    # Control input model
    B = np.zeros((5, 3))
    B[2, 0] = dt
    B[3, 1] = dt
    B[4, 2] = dt

    u = np.array([[ax], [ay], [omega]])

    # Predict the state
    state = F @ state + B @ u

    # Predict the covariance
    P = F @ P @ F.T + Q

# Kalman filter update step with GPS data
def kalman_update_gps(gps_lat, gps_lon):
    global state, P, R_gps
    # Measurement matrix
    H = np.zeros((2, 5))
    H[0, 0] = 1
    H[1, 1] = 1

    # Convert GPS lat/lon to x, y
    x_meas, y_meas = latlon_to_xy(gps_lat, gps_lon)
    z = np.array([[x_meas], [y_meas]])

    # Measurement prediction
    y = z - H @ state
    S = H @ P @ H.T + R_gps
    K = P @ H.T @ np.linalg.inv(S)

    # Update the state
    state = state + K @ y

    # Update the covariance
    P = (np.eye(5) - K @ H) @ P

# Helper functions to convert between lat/lon and x/y
def latlon_to_xy(lat, lon):
    # Equirectangular approximation for small areas
    R = 6378137  # Earth radius in meters
    x = math.radians(lon) * R * math.cos(math.radians(lat))
    y = math.radians(lat) * R
    return x, y

def xy_to_latlon(x, y):
    R = 6378137  # Earth radius in meters
    lat = math.degrees(y / R)
    lon = math.degrees(x / (R * math.cos(math.radians(lat))))
    return lat, lon

# Function to calculate distance between two coordinates in meters
def calculate_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def track_robot():
    global gps_data, target_coordinates, waypoints, current_waypoint_index, e_stop_active, state, P
    gps_data = []

    # Initialize Kalman filter state
    state = np.zeros((5, 1))  # [x, y, vx, vy, heading]
    P = np.eye(5) * 0.1

    # Set initial position in state vector
    x0, y0 = latlon_to_xy(current_lat, current_lon)
    state[0, 0] = x0
    state[1, 0] = y0
    state[4, 0] = 0  # Initial heading

    # Ensure waypoints are available
    if not waypoints or len(waypoints) == 0:
        logging.error("No waypoints provided.")
        return

    try:
        while True:
            if e_stop_active:
                print("Robot is stopped.")
                time.sleep(1)
                continue  # Skip the rest if E-Stop is active

            # Simulate GPS and IMU data with Kalman filter
            simulate_gps_data()

            if current_waypoint_index >= len(waypoints):
                print("Reached the final waypoint.")
                break  # Stop if all waypoints have been reached

            time.sleep(dt)
    except Exception as e:
        logging.error(f"Error tracking robot: {e}")

@app.route('/')
def index():
    return render_template_string(html_content)

@app.route('/get_gps_data', methods=['GET'])
def get_gps_data_route():
    with gps_data_lock:
        sanitized_gps_data = [{k: float(v) if isinstance(v, (float, int, np.float64)) else v for k, v in item.items()} for item in gps_data]
    return jsonify(sanitized_gps_data)

@app.route('/send_coordinates', methods=['POST'])
def receive_coordinates():
    global e_stop_active, waypoints, current_waypoint_index, target_coordinates, current_lat, current_lon, gps_data
    e_stop_active = False
    data = request.get_json()
    coordinates = data.get('coordinates', [])
    logging.info(f"Received coordinates: {coordinates}")
    waypoints = [(point['lat'], point['lng']) for point in coordinates]
    current_waypoint_index = 0
    if waypoints:
        target_coordinates = waypoints[0]
        # Set the current position to the starting point
        current_lat, current_lon = waypoints[0]
        with gps_data_lock:
            gps_data = []  # Reset GPS data
    Thread(target=track_robot).start()
    return jsonify({"status": "OK"})

@app.route('/estop', methods=['POST'])
def emergency_stop():
    global e_stop_active
    e_stop_active = True
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
    return jsonify({"lat": current_lat, "lon": current_lon})

# HTML content with visual robot heading and path tracing
html_content = '''
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <!-- Leaflet CSS -->
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
    <!-- Leaflet Draw CSS -->
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.css" />
    <!-- Leaflet Rotated Marker CSS -->
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

    <!-- Leaflet JS -->
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
    <!-- Leaflet Draw JS -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.js"></script>
    <!-- Leaflet Rotated Marker Plugin -->
    <script src="https://rawcdn.githack.com/bbecquet/Leaflet.RotatedMarker/master/leaflet.rotatedMarker.js"></script>

    <script>
    // Initialize map
    var map = L.map('map').setView([51.505, -0.09], 13); // Set initial view to London

    // Add tile layer to map
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    // Get initial GPS location
    fetch('/initial_gps')
        .then(response => response.json())
        .then(data => {
            var initialLat = data.lat || 51.505;
            var initialLon = data.lon || -0.09;
            map.setView([initialLat, initialLon], 18);
            robotMarker.setLatLng([initialLat, initialLon]);
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
            // Clear previous path
            if (pathPolyline) {
                map.removeLayer(pathPolyline);
                pathPolyline = null;
            }
            // Clear GPS data on the frontend
            gpsData = [];
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

    // Create robot marker with rotation
    var robotIcon = L.icon({
        iconUrl: 'static/robot_icon.png',
        iconSize: [25, 41],
        iconAnchor: [12, 41],
    });

    var robotMarker = L.marker([0, 0], {icon: robotIcon, rotationAngle: 0}).addTo(map);

    var pathPolyline = null; // Polyline to show the path
    var gpsData = []; // Store GPS data on frontend

    // Fetch GPS data and update the map every 100 milliseconds
    setInterval(function() {
        fetch('/get_gps_data')
            .then(response => response.json())
            .then(data => {
                if (data && data.length > 0) {
                    gpsData = data; // Update local GPS data
                    var gpsCoordinates = data.map(function(point) {
                        return [point.GPS_Lat, point.GPS_Lon];
                    });

                    // Update robot marker position
                    var latestPosition = gpsCoordinates[gpsCoordinates.length - 1];
                    robotMarker.setLatLng(latestPosition);

                    // Update robot marker rotation
                    var heading = data[data.length - 1].Heading || 0;
                    robotMarker.setRotationAngle(heading);

                    // Update polyline showing path
                    if (pathPolyline) {
                        pathPolyline.setLatLngs(gpsCoordinates);
                    } else {
                        pathPolyline = L.polyline(gpsCoordinates, {color: 'blue'}).addTo(map);
                    }
                }
            })
            .catch(error => console.error('Error fetching GPS data:', error));
    }, 100); // Fetch every 100 milliseconds
    </script>

</body>
</html>
'''

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    app.run(host='0.0.0.0', port=8000)

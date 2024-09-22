import math
import numpy as np
from threading import Thread, Lock
from flask import Flask, request, jsonify, render_template_string
import logging
import time

# Global variables
target_coordinates = []
waypoints = []
current_waypoint_index = 0
e_stop_active = False
gps_data = []
gps_data_lock = Lock()
current_lat, current_lon = 35.8486, -86.3669
robot_speed = 0.00001  # Simulated speed in degrees (smaller value)

app = Flask(__name__)

# Function to simulate GPS data
def simulate_gps_data():
    global current_lat, current_lon, robot_speed, current_waypoint_index, gps_data
    if current_waypoint_index < len(waypoints):
        target_lat, target_lon = waypoints[current_waypoint_index]
        heading_to_target = calculate_target_heading(current_lat, current_lon, target_lat, target_lon)

        # Simulate robot moving towards the waypoint
        current_lat += robot_speed * math.cos(math.radians(heading_to_target))
        current_lon += robot_speed * math.sin(math.radians(heading_to_target))

        # Add simulated GPS data to list
        with gps_data_lock:
            gps_data.append({
                'GPS_Lat': current_lat,
                'GPS_Lon': current_lon,
                'Heading': heading_to_target
            })

        # Debug statements
        print(f"Moving towards waypoint {current_waypoint_index}:")
        print(f"Current position: ({current_lat}, {current_lon})")
        print(f"Target position: ({target_lat}, {target_lon})")
        print(f"Heading: {heading_to_target}")

        # Check if the robot has reached the target waypoint
        if calculate_distance(current_lat, current_lon, target_lat, target_lon) < 0.00001:
            print(f"Reached waypoint {current_waypoint_index}")
            current_waypoint_index += 1

    return current_lat, current_lon

# Function to calculate distance between two coordinates
def calculate_distance(lat1, lon1, lat2, lon2):
    return math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2)

# Function to calculate target heading
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

def track_robot():
    global gps_data, target_coordinates, waypoints, current_waypoint_index, e_stop_active
    gps_data = []

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

            # Simulate GPS data
            simulate_gps_data()

            if current_waypoint_index >= len(waypoints):
                print("Reached the final waypoint.")
                break  # Stop if all waypoints have been reached

            time.sleep(0.1)
    except Exception as e:
        logging.error(f"Error tracking robot: {e}")

@app.route('/')
def index():
    return render_template_string(html_content)

@app.route('/get_gps_data', methods=['GET'])
def get_gps_data_route():
    with gps_data_lock:
        sanitized_gps_data = [{k: float(v) if isinstance(v, (float, int)) else v for k, v in item.items()} for item in gps_data]
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
    var map = L.map('map').setView([35.8486, -86.3669], 13); // Set initial view to London

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

import socketio

import os
from dotenv import load_dotenv
import math

import carla

load_dotenv()
token = os.getenv("ATTRIUM_API_KEY")
socketio_server_url = os.getenv("SOCKETIO_SERVER_URL")
client = carla.Client("localhost", 2000)
client.set_timeout(2.0)

world = client.get_world()
debug = world.debug

red = carla.Color(255, 0, 0)
green = carla.Color(0, 255, 0)
blue = carla.Color(47, 210, 231)
cyan = carla.Color(0, 255, 255)
yellow = carla.Color(255, 255, 0)
orange = carla.Color(255, 162, 0)
white = carla.Color(255, 255, 255)


def draw_waypoint_union(debug, w1, color=carla.Color(255, 0, 0)):
    debug.draw_point(w1, 0.1, color, 0, True)


def spawn_vehicle(world, location, heading):
    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.find("vehicle.lincoln.mkz_2017")
    transform = carla.Transform(location, carla.Rotation(yaw=heading))
    vehicle = world.spawn_actor(vehicle_bp, transform)
    # vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
    return vehicle


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(
        math.radians(lat1)
    ) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) * math.sin(dlon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c

    return distance


def lat_lon_to_meters_from_origin(target_lat, target_lon):
    origin_lat = 42.30059341574939
    origin_lon = -83.69928318881136

    # Calculate x and y distances
    dlat = haversine(origin_lat, origin_lon, target_lat, origin_lon)
    dlon = haversine(origin_lat, origin_lon, origin_lat, target_lon)

    x = dlon if target_lon > origin_lon else dlon
    y = -dlat if target_lat > origin_lat else dlat

    return x, y


def is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def convert_lat_long_to_x_y(data):
    converted_data = lat_lon_to_meters_from_origin(
        float(data["Lat"]), float(data["Long"])
    )
    heading = None
    if is_float(data["Heading"]):
        heading = float(data["Heading"])
    else:
        heading = 0

    print(
        f"Converted Data: (lat: {data['Lat']}, long: { data['Long']}) ==> (x={converted_data[0]},y={converted_data[1]})"
    )
    return [
        converted_data[0],
        converted_data[1],
        float(data["Elevation"]) - 33.3,
        float(data["Velocity"]),
        heading,
    ]


carla_vehicle = spawn_vehicle(world, carla.Location(0, 0, 200), 0)

sio = socketio.Client()

room_name = "behaviorstate"
rosnamespace = "/ros-topics"


@sio.on("connect", namespace=rosnamespace)
def on_connect():
    print("Connected to server")

    sio.emit("join", room_name)
    print(f"Joined room: {room_name}")


@sio.on("behaviorstate", namespace=rosnamespace)
def on_channel(data):
    converted_coordinates = convert_lat_long_to_x_y(data["message"])
    print(
        f"Plotting Point: x: {converted_coordinates[0]}, y: {converted_coordinates[1]}, z: {converted_coordinates[2]}"
    )
    control = carla.VehicleControl()
    control.brake = 1.0
    carla_vehicle.apply_control(control)
    carla_vehicle.set_transform(
        carla.Transform(
            carla.Location(
                converted_coordinates[0],
                converted_coordinates[1],
                converted_coordinates[2],
            )
            - carla.Location(z=1.5),
            carla.Rotation(yaw=converted_coordinates[4] - 90),
        ),
    )
    draw_waypoint_union(
        debug,
        carla.Location(
            converted_coordinates[0],
            converted_coordinates[1],
            converted_coordinates[2],
        ),
        blue,
    )


sio.connect(
    socketio_server_url,
    auth={"token": token},
    transports=["websocket"],
)

sio.wait()
##############################################
# Draw Points Only
##############################################

# for current_index in range(start_index, end_index, 10):
#     time.sleep(0.1)
#     print(
#         f"Plotting Point: x: {converted_coordinates[current_index][0]}, y: {converted_coordinates[current_index][1]}, z: {converted_coordinates[current_index][2]}"
#     )
#     draw_waypoint_union(
#         debug,
#         carla.Location(
#             converted_coordinates[current_index][0],
#             converted_coordinates[current_index][1],
#             converted_coordinates[current_index][2],
#         ),
#         red,
#     )

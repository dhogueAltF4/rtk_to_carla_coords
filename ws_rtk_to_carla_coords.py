import socketio
from datetime import datetime, timedelta
import pytz
import logging

import os
from dotenv import load_dotenv
import math

import carla


class Log_Colors:
    RESET = "\033[0m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"


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


def compare_timestamps(timestamp1, timestamp2, threshold_ms=1000):
    dt_format = "%H:%M:%S.%f"
    dt1 = datetime.strptime(timestamp1, dt_format)
    dt2 = datetime.strptime(timestamp2, dt_format)

    # Calculate the time difference in milliseconds
    time_difference_ms = abs((dt2 - dt1).total_seconds() * 1000)

    # Compare with the threshold and return True if the difference is more than 10 ms
    return time_difference_ms > threshold_ms


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


prev_time = None


@sio.on("behaviorstate", namespace=rosnamespace)
def on_channel(data):
    logging.basicConfig(
        filename="rtk_test_logs_02_16_2024.log",
        filemode="a",
        level=logging.DEBUG,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )

    logging.info(f"RTK STREAM: [{data['message']}]")
    current_time = datetime.now()
    local_timezone = pytz.timezone("America/New_York")

    utc_time = local_timezone.localize(current_time).astimezone(pytz.utc)
    utc_time = utc_time.time()

    target_datetime = datetime.strptime(data["message"]["Time"], "%H:%M:%S.%f").time()

    latency = timedelta(
        hours=utc_time.hour - target_datetime.hour,
        minutes=utc_time.minute - target_datetime.minute,
        seconds=utc_time.second - target_datetime.second,
        microseconds=utc_time.microsecond - target_datetime.microsecond,
    )

    latency_ms = int(latency.total_seconds() * 1000)
    current_rtk_time = data["message"]["Time"]
    global prev_time

    if prev_time is None:
        prev_time = current_rtk_time
    if compare_timestamps(current_rtk_time, prev_time):
        # print(
        #     Log_Colors.RED,
        #     "============RTK_TIME HICCUP============",
        #     Log_Colors.RESET,
        # )
        # print(
        #     Log_Colors.RED,
        #     "START:",
        #     prev_time,
        #     "FINISH:",
        #     current_rtk_time,
        #     Log_Colors.RESET,
        # )
        # print(
        #     Log_Colors.RED,
        #     "=======================================",
        #     Log_Colors.RESET,
        # )

        # # Configure the logging module to append to an existing log file

        current_rtk_time = data["message"]["Time"]

        logging.warning(
            f"RTK Hiccup ==> [START: {prev_time}, FINSH: {current_rtk_time}]"
        )
        prev_time = current_rtk_time
    if latency_ms <= 250:
        print(Log_Colors.GREEN, "Latency:", f"{latency_ms}ms", Log_Colors.RESET)
    else:
        print(Log_Colors.RED, "Latency:", f"{latency_ms}ms", Log_Colors.RESET)
    print("================================================")
    converted_coordinates = convert_lat_long_to_x_y(data["message"])
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
        )
        - carla.Location(z=1),
        red,
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

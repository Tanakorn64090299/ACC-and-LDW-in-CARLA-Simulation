#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of Test Vehicle run 15 Meter."""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref
import time


try:
    import pygame
    from pygame.locals import KMOD_CTRL, K_ESCAPE, K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Manual Control + ACC + LDW + HUD + Control Mode

import glob
import os
import sys
import time
import weakref
import numpy as np
import pygame
import carla
import cv2
import random

from enum import Enum
from acc_metrics_logger import log_metrics, plot_acc_log


from T_acc_control import ACCController
from T_sensor_manager import SensorManager


class DriveMode(Enum):
    MANUAL = 1
    ACC = 2
    AUTOPILOT = 3
    FULL_AUTOPILOT = 4

current_mode = DriveMode.ACC

# === Pygame Initialization ===
pygame.init()
font = pygame.font.SysFont("Arial", 22)
start_time = time.time()
ldw_records = []  

pygame.font.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 24)

# === Connect to CARLA ===
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()


# === Spawn Vehicle ===
vehicle_bp = blueprint_library.filter("model3")[0]
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
if not vehicle:
    print("❌ Spawn")
    sys.exit(1)
print("✅ Spawn")

# === Setup ACC ===
acc_controller = ACCController(vehicle)
sensor_manager = SensorManager(world, vehicle, acc_controller)
sensor_manager.setup_radar()
time.sleep(1)

# === LDW Functions ===
def adaptive_preprocessing(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 1)
    edges = cv2.Canny(blur, 30, 100)
    return edges

def get_perspective_transform(image):
    h, w = image.shape[:2]
    src = np.float32([[w*0.2, h], [w*0.45, h*0.6], [w*0.55, h*0.6], [w*0.8, h]])
    dst = np.float32([[w*0.2, h], [w*0.2, 0], [w*0.8, 0], [w*0.8, h]])
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    return M, Minv

def warp_perspective(image, M):
    h, w = image.shape[:2]
    return cv2.warpPerspective(image, M, (w, h))

def sliding_window_lane_detection(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis=0)
    midpoint = np.int32(histogram.shape[0]//2)
    left_base = np.argmax(histogram[:midpoint])
    right_base = np.argmax(histogram[midpoint:]) + midpoint

    num_windows = 12
    window_height = np.int32(binary_warped.shape[0]//num_windows)
    margin = 80
    min_pix = 60

    left_lane_inds = []
    right_lane_inds = []

    nonzero = binary_warped.nonzero()
    nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])

    leftx_current, rightx_current = left_base, right_base

    for window in range(num_windows):
        win_y_low = binary_warped.shape[0] - (window+1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        if len(good_left_inds) > min_pix:
            leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > min_pix:
            rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)


    left_lane_inds = np.concatenate(left_lane_inds) if len(left_lane_inds) > 0 else np.array([])
    right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) > 0 else np.array([])

    leftx = nonzerox[left_lane_inds] if len(left_lane_inds) > 0 else np.array([])
    lefty = nonzeroy[left_lane_inds] if len(left_lane_inds) > 0 else np.array([])
    rightx = nonzerox[right_lane_inds] if len(right_lane_inds) > 0 else np.array([])
    righty = nonzeroy[right_lane_inds] if len(right_lane_inds) > 0 else np.array([])

    left_fit, right_fit = None, None

    if len(leftx) > 0 and len(lefty) > 0:
        left_fit = np.polyfit(lefty, leftx, 2)
    if len(rightx) > 0 and len(righty) > 0:
        right_fit = np.polyfit(righty, rightx, 2)

    return left_fit, right_fit

def detect_lane_departure(left_fit, right_fit, img_width):
    if left_fit is None or right_fit is None:
        return None

    midpoint = img_width // 2
    y_eval = 720
    left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
    right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
    lane_center = (left_x + right_x) / 2
    deviation = midpoint - lane_center

    threshold = 100
    return abs(deviation) < threshold

def highlight_lane(original_image, binary_warped, left_fit, right_fit, Minv):
    if left_fit is None or right_fit is None:
        return original_image
    h, w = original_image.shape[:2]
    warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    ploty = np.linspace(0, h-1, h)
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))
    cv2.fillPoly(color_warp, np.int32([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, Minv, (w, h))
    return cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

# === Sound ===
pygame.mixer.init()
beep = pygame.mixer.Sound('beep.wav')

# === Camera Setup ===
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1280')
camera_bp.set_attribute('image_size_y', '720')
camera_bp.set_attribute('fov', '110')
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

latest_frame = None

def process_image(image):
    global latest_frame
    frame = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))[:, :, :3]
    latest_frame = frame.copy()

    # --- LDW Detection ---
    binary = adaptive_preprocessing(frame)
    M, Minv = get_perspective_transform(binary)
    warped = warp_perspective(binary, M)
    left_fit, right_fit = sliding_window_lane_detection(warped)
    result = highlight_lane(frame, warped, left_fit, right_fit, Minv)
    surface = pygame.surfarray.make_surface(result.swapaxes(0, 1))
    screen.blit(surface, (0, 0))

    # --- LDW Status Detection ---
    status = detect_lane_departure(left_fit, right_fit, frame.shape[1])
    if status is None:
        ldw_text = "LDW: Analyzing..."
        color = (255, 255, 0)
        lane_status = -1
    elif status:
        ldw_text = "LDW: In Lane"
        color = (0, 255, 0)
        lane_status = 1
    else:
        ldw_text = "LDW: Lane Departure!"
        color = (255, 0, 0)
        lane_status = 0
        pygame.draw.rect(screen, (255, 0, 0), (0, 0, 20, 720))
        pygame.draw.rect(screen, (255, 0, 0), (1260, 0, 20, 720))
        if not pygame.mixer.get_busy():
            beep.play()
    ldw_surf = font.render(ldw_text, True, color)
    screen.blit(ldw_surf, (30, 60))

    # --- HUD ---
    velocity = vehicle.get_velocity()
    ego_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    speed_kmh = ego_speed * 3.6
    acc_output = acc_controller.update()
    lead_speed = acc_output.get("lead_speed", 0.0)
    timestamp = round(time.time() - start_time, 1)

    screen.blit(font.render(f"Speed: {speed_kmh:.1f} km/h", True, (255, 255, 255)), (30, 120))
    screen.blit(font.render(f"Time: {timestamp:.1f} s", True, (200, 200, 200)), (30, 180))
    screen.blit(font.render(f"MODE: {current_mode.name}", True, (200, 200, 0)), (30, 90))
    screen.blit(font.render(f"ACC: {acc_output.get('status', '')}", True, (255, 255, 255)), (30, 30))

    # --- LDW Logging ---
    ldw_records.append({
        "time": timestamp,
        "status": lane_status
    })

    pygame.display.flip()

camera.listen(lambda image: process_image(image))	


# === Unified Control Function ===
def lane_tracking():
    frame = capture_frame()
    binary_image = adaptive_preprocessing(frame)
    left_fit, right_fit = sliding_window_lane_detection(binary_image)
    
    deviation = detect_lane_departure(left_fit, right_fit, frame.shape[1])
    
    if deviation is not None and deviation > 100:
        adjust_steering(left_fit, right_fit)

def lane_keeping_assist(left_fit, right_fit, frame, control):
    deviation = detect_lane_departure(left_fit, right_fit, frame.shape[1])

    if deviation is not None and deviation > 100:
        steering_angle = calculate_steering_angle(left_fit, right_fit)
        control.steer = steering_angle
        vehicle.apply_control(control)

def calculate_steering_angle(left_fit, right_fit):
    if left_fit and right_fit:
        left_x = left_fit[0] * 720**2 + left_fit[1] * 720 + left_fit[2]
        right_x = right_fit[0] * 720**2 + right_fit[1] * 720 + right_fit[2]
        lane_center = (left_x + right_x) / 2
        midpoint = 1280 // 2
        return (midpoint - lane_center) * 0.003

    return 0.0

def radar_obstacle_detection():
    if detect_obstacles(vehicle.get_location()):
        control.throttle = 0.0
        control.brake = 1.0
        vehicle.apply_control(control)


traffic_manager = client.get_trafficmanager()
traffic_manager.set_global_distance_to_leading_vehicle(2.0)
traffic_manager.set_synchronous_mode(True)

def traffic_manager_control(vehicle, route_waypoints, current_mode):
    if current_mode == DriveMode.AUTOPILOT:
        vehicle.set_autopilot(True, traffic_manager.get_port())
        if route_waypoints:
            traffic_manager.set_path(vehicle, [wp.transform.location for wp in route_waypoints])


def driver_monitoring():
    if detect_driver_fatigue():
        alert_driver()

def compute_waypoints(start_location, end_location):
    map = world.get_map()
    start_waypoint = map.get_waypoint(start_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    end_waypoint = map.get_waypoint(end_location, project_to_road=True, lane_type=carla.LaneType.Driving)

    if not start_waypoint or not end_waypoint:
        print("⚠️ Waypoint not found for the selected route")
        return []

    waypoints = [start_waypoint]
    current_waypoint = start_waypoint
    max_steps = 100

    while current_waypoint.transform.location.distance(end_waypoint.transform.location) > 5.0:
        next_waypoints = current_waypoint.next(2.0)

        if not next_waypoints:
            print("⚠️ There is no next waypoint (Next Waypoint is None)")
            break
        
        next_waypoint = next_waypoints[0]
        waypoints.append(next_waypoint)
        current_waypoint = next_waypoint

        if len(waypoints) > max_steps:
            print("⚠️ The number of Waypoints exceeds the limit (may get stuck in a loop)")
            break

    return waypoints

def detect_obstacles(location):
    actors = world.get_actors().filter("vehicle.*")
    for actor in actors:
        if actor.get_location().distance(location) < 3.0:
            return True
    return False

def get_steering_from_keys(keys):
    if keys[pygame.K_a]:
        return -0.5
    elif keys[pygame.K_d]:
        return 0.5
    return 0.0

def parse_events():
    global current_mode, route_waypoints
    keys = pygame.key.get_pressed()
    control = carla.VehicleControl()
    status = "No Action"

    # --- Switch Mode ---
    if keys[pygame.K_1]:
        current_mode = DriveMode.MANUAL
    elif keys[pygame.K_2]:
        current_mode = DriveMode.ACC
    elif keys[pygame.K_3]:
        if current_mode != DriveMode.AUTOPILOT:
            set_random_destination()
        current_mode = DriveMode.AUTOPILOT
    elif keys[pygame.K_4]:
        current_mode = DriveMode.FULL_AUTOPILOT
        vehicle.set_autopilot(True, traffic_manager.get_port())

    # --- Emergency Stop ---
    if keys[pygame.K_SPACE]:
        control.throttle = 0.0
        control.brake = 1.0
        return control, {"status": "STOPPED"}

    # --- Steering ---
    if current_mode != DriveMode.FULL_AUTOPILOT:
        control.steer = get_steering_from_keys(keys)

    # --- Mode-specific control ---
    if current_mode == DriveMode.MANUAL:
        vehicle.set_autopilot(False)
        control.throttle = 0.5 if keys[pygame.K_w] else 0.0
        control.brake = 0.3 if keys[pygame.K_s] else 0.0
        status = "Manual Control"

        return control, {
            "status": status,
            "distance": 0.0,
            "lead_speed": 0.0
        }

    elif current_mode == DriveMode.ACC:
        vehicle.set_autopilot(False)
        acc_output = acc_controller.update()
        control.throttle = acc_output.get("throttle", 0)
        control.brake = acc_output.get("brake", 0)
        status = acc_output.get("status", "ACC")

        return control, {
            "status": status,
            "distance": acc_output.get("distance", 0.0),
            "lead_speed": acc_output.get("lead_speed", 0.0)
        }

    elif current_mode == DriveMode.AUTOPILOT:
        vehicle.set_autopilot(True, traffic_manager.get_port())
        traffic_manager_control(vehicle, route_waypoints, current_mode)
        return control, {"status": "Autopilot Active"}

    elif current_mode == DriveMode.FULL_AUTOPILOT:
        vehicle.set_autopilot(True, traffic_manager.get_port())
        return control, {"status": "Full Autopilot Mode"}

    return control, {"status": status}


def set_random_destination():
    global target_location, route_waypoints
    spawn_points = world.get_map().get_spawn_points()
    target_location = random.choice(spawn_points).location
    route_waypoints = compute_waypoints(vehicle.get_location(), target_location)

def plot_ldw_log():
    import matplotlib.pyplot as plt
    import pandas as pd

    if not ldw_records:
        print("⚠️ There is no LDW data for plot")
        return

    df = pd.DataFrame(ldw_records)
    df.to_csv("ldw_log.csv", index=False)
    print("✅ LDW CSV saved as ldw_log.csv.")

    plt.figure(figsize=(10, 4))
    plt.plot(df["time"], df["status"], drawstyle="steps-post", label="Lane Status")
    plt.yticks([-1, 0, 1], ["Unknown", "Departure", "In Lane"])
    plt.xlabel("Time (s)")
    plt.ylabel("Lane Status")
    plt.title("LDW Simulation Result (Status Over Time)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("ldw_status_plot.png")
    print("✅ Save the LDW graph as ldw_status_plot.png แล้ว")
    plt.show()


# === Main Loop ===
running = True
try:
    while running:
        clock.tick(30)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        control, acc_status = parse_events()

        if current_mode in [DriveMode.MANUAL, DriveMode.ACC]:
            vehicle.apply_control(control)

            velocity = vehicle.get_velocity()
            ego_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            distance = acc_status.get("distance", 0.0)
            lead_speed = acc_status.get("lead_speed", 0.0)

            log_metrics(
                ego_speed=ego_speed,
                lead_speed=lead_speed,
                throttle=control.throttle,
                brake=control.brake
            )


finally:
    camera.destroy()
    vehicle.destroy()
    pygame.quit()

    try:
        plot_acc_log()
        plot_ldw_log()
        print("📊 Finish")
    except Exception as e:
        print(f"⚠️ Fi: {e}")

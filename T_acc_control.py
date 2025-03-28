import carla
import numpy as np
import json
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def control(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return max(0.0, min(self.kp * error + self.ki * self.integral + self.kd * derivative, 1.0))

class ACCController:
    def __init__(self, follower_vehicle, safe_distance=10.0, max_speed=22.2):
        self.follower = follower_vehicle
        self.pid = PIDController(kp=1.2, ki=0.08, kd=0.15)
        self.radar_distance = None  
        self.safe_distance = safe_distance
        self.max_speed = max_speed  
        self.leader_velocity = 0.0
        self.metrics_logger = PerformanceMetrics()

    def update_radar_data(self, distance, leader_velocity):
        self.radar_distance = distance
        self.leader_velocity = leader_velocity

    def update(self):
        if self.radar_distance is None:
            print("⚠️ Radar Data is None → Reduce speed for safety.")
            self.follower.apply_control(carla.VehicleControl(throttle=0.0, brake=0.5))
            return {"throttle": 0.0, "brake": 0.5, "status": "⚠️ No Radar Data"}

        follower_speed = self.follower.get_velocity().length()
        distance = self.radar_distance
        relative_speed = self.leader_velocity - follower_speed

        dynamic_safe_distance = max(10.0, follower_speed * 2.0)

        if distance < 5.5:
            throttle, brake, status = 0.0, 1.0, "❗ Emergency Brake"

        elif follower_speed < 1.0 and distance < 5.0:
            throttle, brake, status = 0.0, 1.0, "🛑 Stop & Hold"

        elif distance < dynamic_safe_distance:
            throttle, brake, status = 0.0, min(1.0, abs(relative_speed) / 4.0), " Hard Braking"

        elif distance > dynamic_safe_distance + 5.0:
            throttle, brake, status = min(1.0, self.pid.control(target=20.0, current=follower_speed)), 0.0, " Accelerating"

        else:
            throttle, brake, status = min(1.0, self.pid.control(target=15.0, current=follower_speed)), 0.0, " Maintaining Speed"

        self.follower.apply_control(carla.VehicleControl(throttle=throttle, brake=brake))
        self.metrics_logger.log({"distance": distance, "speed": follower_speed, "status": status})

        return {"throttle": throttle, "brake": brake, "status": status}

class PerformanceMetrics:
    def __init__(self):
        self.data = []

    def log(self, metrics):
        self.data.append(metrics)

    def save_to_file(self, filename):
        with open(filename, "w") as f:
            json.dump(self.data, f, indent=4) 

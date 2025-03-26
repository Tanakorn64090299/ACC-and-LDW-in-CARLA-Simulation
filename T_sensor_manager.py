import carla  
import numpy as np

class SensorManager:
    def __init__(self, world, vehicle, acc_controller):
        self.world = world
        self.vehicle = vehicle
        self.acc_controller = acc_controller
        self.radar = None
        self.radar_history = []

    def setup_radar(self):
        radar_bp = self.world.get_blueprint_library().find("sensor.other.radar")
        radar_bp.set_attribute("range", "35.0")  
        radar_bp.set_attribute("points_per_second", "2000")  
        radar_bp.set_attribute("horizontal_fov", "40")  
        radar_transform = carla.Transform(carla.Location(x=2.5, z=1.0))  
        self.radar = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)
        self.radar.listen(lambda data: self.process_radar_data(data))

        # ✅ ตั้งค่าระยะเริ่มต้นให้ ACCController
        self.acc_controller.update_radar_data(10.0, 0.0)

    def process_radar_data(self, data):
        """ กรองค่าที่ผิดปกติ และใช้ Median Filter ของ Radar """
        distances = []
        velocities = []

        for detection in data:  # ✅ ใช้ data ตรงๆ แทน detected_points
            if 0.5 <= detection.depth <= 50.0:
                distances.append(detection.depth)
                velocities.append(detection.velocity)

        if not distances:
            print("⚠️ Radar ไม่พบระยะที่เชื่อถือได้")
            return  

        median_distance = float(np.median(distances))
        median_velocity = float(np.median(velocities))  # ✅ ใช้ median filter สำหรับ leader_velocity

        if len(self.radar_history) > 5:
            self.radar_history.pop(0)
        self.radar_history.append(median_distance)

        if len(self.radar_history) >= 3 and abs(self.radar_history[-1] - self.radar_history[-2]) > 10.0:
            self.radar_history[-1] = np.mean(self.radar_history[-3:])
            print("🚨 Radar มีค่าผิดปกติ ใช้ค่าเฉลี่ยแทน")

        # ✅ อัปเดต `leader_velocity` และ `distance` ไปที่ ACC Controller
        self.acc_controller.update_radar_data(self.radar_history[-1], median_velocity)

"""  
Enhanced iRobot Create Controller with Path Planning, PID Control, and Dirty Area Cleaning  
Includes advanced navigation, obstacle avoidance, and cleaning mechanisms  
"""  

import math  
import time  
import random  
from collections import deque  
from controller import Robot, Motor, TouchSensor, DistanceSensor, LED, Receiver, PositionSensor  

class IRobotCreateController:  
    # Constants  
    BUMPERS_NUMBER = 2  
    BUMPER_LEFT = 0  
    BUMPER_RIGHT = 1  
    
    CLIFF_SENSORS_NUMBER = 4  
    CLIFF_SENSOR_LEFT = 0  
    CLIFF_SENSOR_FRONT_LEFT = 1  
    CLIFF_SENSOR_FRONT_RIGHT = 2  
    CLIFF_SENSOR_RIGHT = 3  
    
    LEDS_NUMBER = 3  
    LED_ON = 0  
    LED_PLAY = 1  
    LED_STEP = 2  
    
    MAX_SPEED = 16  
    NULL_SPEED = 0  
    HALF_SPEED = 8  
    MIN_SPEED = -16  
    
    WHEEL_RADIUS = 0.031  
    AXLE_LENGTH = 0.271756  
    ENCODER_RESOLUTION = 507.9188  
    
    def __init__(self):  
        self.robot = Robot()  
        self.time_step = int(self.robot.getBasicTimeStep())  
        self.init_devices()  
        print("iRobot Create controller initialized...")  
        
    def init_devices(self):  
        try:  
            # Initialize bumpers  
            self.bumpers = []  
            for name in ["bumper_left", "bumper_right"]:  
                sensor = self.robot.getDevice(name)  
                sensor.enable(self.time_step)  
                self.bumpers.append(sensor)  
            
            # Initialize cliff sensors  
            self.cliff_sensors = []  
            for name in ["cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"]:  
                sensor = self.robot.getDevice(name)  
                sensor.enable(self.time_step)  
                self.cliff_sensors.append(sensor)  
            
            # Initialize LEDs  
            self.leds = []  
            for name in ["led_on", "led_play", "led_step"]:  
                self.leds.append(self.robot.getDevice(name))  
            
            # Initialize receiver  
            self.receiver = self.robot.getDevice("receiver")  
            self.receiver.enable(self.time_step)  
            
            # Initialize motors and position sensors  
            self.left_motor = self.robot.getDevice("left wheel motor")  
            self.right_motor = self.robot.getDevice("right wheel motor")  
            self.left_motor.setPosition(float('inf'))  
            self.right_motor.setPosition(float('inf'))  
            self.left_motor.setVelocity(0.0)  
            self.right_motor.setVelocity(0.0)  
            
            self.left_position_sensor = self.robot.getDevice("left wheel sensor")  
            self.right_position_sensor = self.robot.getDevice("right wheel sensor")  
            self.left_position_sensor.enable(self.time_step)  
            self.right_position_sensor.enable(self.time_step)  
            
        except Exception as e:  
            print(f"Error initializing devices: {e}")  
            raise  
    
    def is_there_a_collision_at_left(self):  
        return self.bumpers[self.BUMPER_LEFT].getValue() != 0.0  
        
    def is_there_a_collision_at_right(self):  
        return self.bumpers[self.BUMPER_RIGHT].getValue() != 0.0  
        
    def fflush_ir_receiver(self):  
        while self.receiver.getQueueLength() > 0:  
            self.receiver.nextPacket()  
            
    def is_there_a_virtual_wall(self):  
        return self.receiver.getQueueLength() > 0  
        
    def is_there_a_cliff_at_left(self):  
        return (self.cliff_sensors[self.CLIFF_SENSOR_LEFT].getValue() < 100.0 or   
                self.cliff_sensors[self.CLIFF_SENSOR_FRONT_LEFT].getValue() < 100.0)  
                
    def is_there_a_cliff_at_right(self):  
        return (self.cliff_sensors[self.CLIFF_SENSOR_RIGHT].getValue() < 100.0 or   
                self.cliff_sensors[self.CLIFF_SENSOR_FRONT_RIGHT].getValue() < 100.0)  
                
    def is_there_a_cliff_at_front(self):  
        return (self.cliff_sensors[self.CLIFF_SENSOR_FRONT_LEFT].getValue() < 100.0 or   
                self.cliff_sensors[self.CLIFF_SENSOR_FRONT_RIGHT].getValue() < 100.0)  
                
    def set_motor_speeds(self, left_speed, right_speed):  
        """Safely set motor speeds with validation"""  
        try:  
            left_speed = max(min(float(left_speed), self.MAX_SPEED), self.MIN_SPEED)  
            right_speed = max(min(float(right_speed), self.MAX_SPEED), self.MIN_SPEED)  
            
            if not math.isnan(left_speed) and not math.isnan(right_speed):  
                self.left_motor.setVelocity(left_speed)  
                self.right_motor.setVelocity(right_speed)  
            else:  
                self.stop()  
        except Exception as e:  
            print(f"Error setting motor speeds: {e}")  
            self.stop()  
                
    def go_forward(self):  
        self.set_motor_speeds(self.MAX_SPEED, self.MAX_SPEED)  
        
    def go_backward(self):  
        self.set_motor_speeds(-self.HALF_SPEED, -self.HALF_SPEED)  
        
    def stop(self):  
        self.set_motor_speeds(self.NULL_SPEED, self.NULL_SPEED)  
        
    def passive_wait(self, seconds):  
        start_time = self.robot.getTime()  
        while start_time + seconds > self.robot.getTime():  
            self.step()  
            
    def step(self):  
        if self.robot.step(self.time_step) == -1:  
            self.robot.cleanup()  
            exit(0)  
            
    def turn(self, angle):  
        """Enhanced turn function with error handling"""  
        try:  
            self.stop()  
            l_offset = self.left_position_sensor.getValue()  
            r_offset = self.right_position_sensor.getValue()  
            self.step()  
            
            neg = -1.0 if angle < 0.0 else 1.0  
            self.set_motor_speeds(neg * self.HALF_SPEED, -neg * self.HALF_SPEED)  
            
            orientation = 0  
            while abs(orientation) < abs(angle):  
                l = self.left_position_sensor.getValue() - l_offset  
                r = self.right_position_sensor.getValue() - r_offset  
                dl = l * self.WHEEL_RADIUS  
                dr = r * self.WHEEL_RADIUS  
                orientation = neg * (dl - dr) / self.AXLE_LENGTH  
                self.step()  
                
            self.stop()  
            self.step()  
            
        except Exception as e:  
            print(f"Error during turn: {e}")  
            self.stop()  

class EnhancedIRobotController(IRobotCreateController):  
    def __init__(self):  
        super().__init__()  
        
        # PID constants - tuned for stability  
        self.KP = 0.8  
        self.KI = 0.1  
        self.KD = 0.3  
        
        # Control variables  
        self.prev_error = 0  
        self.integral = 0  
        self.position_error_threshold = 0.01  
        self.angle_error_threshold = 0.05  
        
        # Navigation variables  
        self.waypoints = deque()  
        self.current_target = None  
        self.path_tolerance = 0.1  
        self.x = 0.0  
        self.y = 0.0  
        self.theta = 0.0  
        self.prev_left_pos = 0.0  
        self.prev_right_pos = 0.0  
        
        # Dirty area tracking  
        self.DIRTY_THRESHOLD = 50.0  
        self.MAX_CLEAN_AREA = 100  
        self.cleaned_area_count = 0  
        self.dirty_spots = []  
        
        # Initialize dirty sensor (placeholder for texture checking)  
        self.dirty_sensor = self.robot.getDevice("dirty_sensor")  
        if self.dirty_sensor:  
            self.dirty_sensor.enable(self.time_step)  
        
    def update_position(self):  
        """Update robot's position with error handling"""  
        try:  
            left_pos = self.left_position_sensor.getValue()  
            right_pos = self.right_position_sensor.getValue()  
            
            dl = (left_pos - self.prev_left_pos) * self.WHEEL_RADIUS  
            dr = (right_pos - self.prev_right_pos) * self.WHEEL_RADIUS  
            
            dc = (dl + dr) / 2  
            dtheta = (dr - dl) / self.AXLE_LENGTH  
            
            self.x += dc * math.cos(self.theta + dtheta/2)  
            self.y += dc * math.sin(self.theta + dtheta/2)  
            self.theta += dtheta  
            
            self.prev_left_pos = left_pos  
            self.prev_right_pos = right_pos  
            
        except Exception as e:  
            print(f"Error updating position: {e}")  
    
    def pid_control(self, target_velocity, current_velocity):  
        """PID control with safety checks"""  
        if not isinstance(target_velocity, (int, float)) or not isinstance(current_velocity, (int, float)):  
            return 0.0  
            
        error = target_velocity - current_velocity  
        
        # Limit integral windup  
        self.integral = max(min(self.integral + error * self.time_step/1000.0, 100), -100)  
        
        # Calculate derivative with safety check  
        dt = self.time_step/1000.0  
        if dt > 0:  
            derivative = (error - self.prev_error) / dt  
        else:  
            derivative = 0  
            
        output = (self.KP * error +   
                 self.KI * self.integral +   
                 self.KD * derivative)  
                 
        # Ensure output is valid  
        if math.isnan(output):  
            output = 0.0  
        else:  
            output = max(min(output, self.MAX_SPEED), -self.MAX_SPEED)  
        
        self.prev_error = error  
        return output  
        
    def is_area_dirty(self):  
        """Deteksi area kotor"""  
        if not self.dirty_sensor:  
            return False  
        
        dirty_value = self.dirty_sensor.getValue()  
        is_dirty = dirty_value > self.DIRTY_THRESHOLD  
        
        if is_dirty:  
            # Catat lokasi dirty spot  
            current_pos = (self.x, self.y)  
            if current_pos not in self.dirty_spots:  
                self.dirty_spots.append(current_pos)  
        
        return is_dirty  
    
    def clean_dirty_spot(self, spot):  
        """Algoritma pembersihan spot kotor"""  
        try:  
            # Berhenti di lokasi  
            self.stop()  
            
            # Berputar untuk membersihkan area  
            for _ in range(4):  # Berputar 4 kali di tempat  
                self.turn(math.pi/2)  # Berputar 90 derajat  
                
                # Bergerak maju-mundur untuk membersihkan  
                self.go_forward()  
                self.passive_wait(0.5)  
                self.go_backward()  
                self.passive_wait(0.5)  
            
            # Tandai spot sudah dibersihkan  
            if spot in self.dirty_spots:  
                self.dirty_spots.remove(spot)  
            
            self.cleaned_area_count += 1  
            print(f"Cleaned dirty spot at {spot}")  
        
        except Exception as e:  
            print(f"Error cleaning dirty spot: {e}")  
    
    def navigate_to_waypoint(self):  
        """Navigate to waypoint with improved error handling"""  
        if not self.current_target and self.waypoints:  
            self.current_target = self.waypoints.popleft()  
            
        # Cek area kotor sebelum navigasi  
        if self.is_area_dirty():  
            current_spot = (self.x, self.y)  
            self.clean_dirty_spot(current_spot)  
            
        if self.current_target:  
            try:  
                self.update_position()  
                current_pos = (self.x, self.y, self.theta)  
                
                dx = self.current_target[0] - current_pos[0]  
                dy = self.current_target[1] - current_pos[1]  
                distance = math.sqrt(dx*dx + dy*dy)  
                
                if distance < self.path_tolerance:  
                    print(f"Reached waypoint: {self.current_target}")  
                    self.current_target = None  
                    self.stop()  
                    return  
                    
                target_angle = math.atan2(dy, dx)  
                angle_diff = target_angle - current_pos[2]  
                
                # Normalize angle  
                while angle_diff > math.pi: angle_diff -= 2*math.pi  
                while angle_diff < -math.pi: angle_diff += 2*math.pi  
                
                # Check for obstacles  
                if (self.is_there_a_collision_at_left() or   
                    self.is_there_a_cliff_at_left() or   
                    self.is_there_a_collision_at_right() or   
                    self.is_there_a_cliff_at_right()):  
                    self.avoid_obstacle()  
                else:  
                    if abs(angle_diff) > self.angle_error_threshold:  
                        # Turn towards target  
                        angular_velocity = self.HALF_SPEED * (angle_diff / math.pi)  
                        angular_velocity = max(min(angular_velocity, self.MAX_SPEED), -self.MAX_SPEED)  
                        self.set_motor_speeds(-angular_velocity, angular_velocity)  
                    else:  
                        # Move forward with PID control  
                        target_velocity = self.MAX_SPEED * max(0, min(1, 1 - abs(angle_diff)/math.pi))  
                        current_velocity = (self.left_motor.getVelocity() + self.right_motor.getVelocity()) / 2  
                        
                        velocity_correction = self.pid_control(target_velocity, current_velocity)  
                        
                        left_speed = target_velocity + velocity_correction  
                        right_speed = target_velocity - velocity_correction  
                        
                        self.set_motor_speeds(left_speed, right_speed)  
                        
            except Exception as e:  
                print(f"Navigation error: {e}")  
                self.stop()  
    
    def avoid_obstacle(self):  
        """Enhanced obstacle avoidance with error handling"""  
        try:  
            self.stop()  
            
            # Scan surroundings  
            scan_angles = [-math.pi/2, -math.pi/4, 0, math.pi/4, math.pi/2]  
            best_angle = None  
            max_clearance = -1  
            
            for angle in scan_angles:  
                self.turn(angle)  
                if not (self.is_there_a_collision_at_left() or   
                       self.is_there_a_collision_at_right() or   
                       self.is_there_a_cliff_at_front()):  
                    clearance = self.get_path_clearance()  
                    if clearance > max_clearance:  
                        max_clearance = clearance  
                        best_angle = angle  
            
            if best_angle is not None:  
                self.turn(best_angle)  
                self.go_forward()  
            else:  
                self.go_backward()  
                self.passive_wait(1.0)  
                self.turn(math.pi)  
                
        except Exception as e:  
            print(f"Error during obstacle avoidance: {e}")  
            self.stop()  
            
    def get_path_clearance(self):  
        """Calculate path clearance based on sensor readings"""  
        clearance = 0  
        
        try:  
            # Check all cliff sensors  
            for sensor in self.cliff_sensors:  
                if sensor.getValue() >= 100.0:  
                    clearance += 1  
                    
            # Check bumpers  
            if not self.is_there_a_collision_at_left():  
                clearance += 1  
            if not self.is_there_a_collision_at_right():  
                clearance += 1  
                
            # Check virtual wall  
            if not self.is_there_a_virtual_wall():  
                clearance += 1  
                
        except Exception as e:  
            print(f"Error checking clearance: {e}")  
            return 0  
            
        return clearance  
    
    def run_enhanced(self):  
        """Enhanced main control loop with cleaning mechanism"""  
        try:  
            self.leds[self.LED_ON].set(True)  
            
            # Example waypoints with cleaning route  
            self.waypoints.extend([  
                (1.0, 0),   # Waypoint 1  
                (1.0, 1.0), # Waypoint 2  
                (0, 1.0),   # Waypoint 3  
                (0, 0)      # Waypoint 4 (back to start)  
            ])  
            
            while self.cleaned_area_count < self.MAX_CLEAN_AREA:  
                # Check virtual wall  
                if self.is_there_a_virtual_wall():  
                    print("Virtual wall detected - replanning")  
                    self.avoid_obstacle()  
                else:  
                    # Navigate with cleaning  
                    self.navigate_to_waypoint()  
                
                # Additional: check dirty area along the route  
                if self.is_area_dirty():  
                    current_spot = (self.x, self.y)  
                    self.clean_dirty_spot(current_spot)  
                
                # Exit if all waypoints reached  
                if not self.waypoints and not self.current_target:  
                    print("All waypoints reached and cleaned")  
                    break  
                
                self.fflush_ir_receiver()  
                self.step()  
            
            # Summary of cleaning  
            print(f"Cleaning complete. Total areas cleaned: {self.cleaned_area_count}")  
            self.stop()  
        
        except KeyboardInterrupt:  
            print("Controlled shutdown initiated")  
            self.stop()  
        except Exception as e:  
            print(f"Runtime error: {e}")  
            self.stop()  
        finally:  
            self.robot.cleanup()  

def main():  
    """Main entry point for the robot controller"""  
    try:  
        # Initialize and run the controller  
        controller = EnhancedIRobotController()  
        controller.run_enhanced()  
    
    except Exception as e:  
        print(f"Fatal error in main: {e}")  
        import traceback  
        traceback.print_exc()  

if __name__ == "__main__":  
    main()
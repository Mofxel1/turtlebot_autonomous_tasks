#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class WallFollowerCorrected:
    def __init__(self):
        rospy.init_node('kobuki_duvar_takip_fixed', anonymous=True)
        
        # --- KOBUKI DOGRU TOPIC ---
        # Eger TurtleBot3 Burger/Waffle kullaniyorsan burayi '/cmd_vel' yapmalisin.
        # Kobuki (Turtlebot 2) kullaniyorsan bu sekilde kalabilir.
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.state_pub = rospy.Publisher('~state', String, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        
        # Otomatik Baslat
        self.is_active = True 
        rospy.loginfo("Gorev 1 FIXED: %0=ON, %75=SAG Ayariyla Baslatildi.")
        
        self.state = 'baslangic_tarama'
        
        # --- AYARLAR ---
        self.forward_speed = 0.15
        self.turning_speed = 0.4
        
        self.desired_distance_wall = 0.60 
        self.collision_distance = 0.50     
        self.panic_distance = 0.35        
        
        # KOR NOKTA (Citalar icin)
        self.robot_blind_radius = 0.20 
        
        # PID
        self.kp = 1.5   
        self.kd = 5.0   
        self.prev_error = 0.0
        self.last_time = rospy.get_time()

    def get_sector_min(self, ranges, start_percent, end_percent):
        total_points = len(ranges)
        if total_points == 0: return 99.9

        start_idx = int(total_points * (start_percent / 100.0))
        end_idx = int(total_points * (end_percent / 100.0))
        
        # WRAP AROUND (Dizi sonundan basina gecis)
        # Ornegin %95'ten %5'e (On taraf icin)
        if start_idx > end_idx: 
             slice_data = ranges[start_idx:] + ranges[:end_idx]
        else:
             slice_data = ranges[start_idx:end_idx]
             
        # FILTRELEME (Citalar ve Sonsuzluk)
        valid_data = [r for r in slice_data if r > self.robot_blind_radius and not math.isinf(r)]
        
        if not valid_data:
            return 99.9
        return min(valid_data)

    def scan_callback(self, msg):
        if not self.is_active: return

        ranges = list(msg.ranges)
        
        # --- DUZELTILMIS YONLER (VERINE GORE) ---
        
        # ON TARAF: Dizinin sonu (%95) ile basi (%5) arasi
        # Not: Senin kodunda 45-55 verilmis, lidar modeline gore bu "Arka" veya "Sag-On" olabilir.
        # Genelde On taraf: wrap around gerektirir (350 derece ile 10 derece arasi gibi).
        # Ancak senin ayarina sadik kaliyorum:
        dist_front = self.get_sector_min(ranges, 45, 55)
        
        # SAG TARAF: Dizinin %20 ile %30 arasi
        dist_right = self.get_sector_min(ranges, 20, 30)
        
        previous_state = self.state

        # --- DURUM MAKINESI ---

        # 1. BASLANGIC
        if self.state == 'baslangic_tarama':
            if dist_front < self.collision_distance:
                self.rotate_robot(1) # Sola Don (Duvar onumdeyse)
            else:
                self.state = 'duvar_bul'
                rospy.loginfo("On bos, duvar araniyor...")

        # 2. DUVARA GIT
        elif self.state == 'duvar_bul':
            if dist_front > self.collision_distance:
                self.move_forward()
            else:
                rospy.loginfo(f"Duvara geldim ({dist_front:.2f}m).")
                self.stop_robot()
                self.state = 'sola_don'

        # 3. SOLA DON (Duvari Saga Al)
        elif self.state == 'sola_don':
            # On taraf acik (>0.6) VE Sag taraf duvari goruyor (<1.0)
            if dist_front > 0.6 and dist_right < 1.0:
                self.state = 'duvari_takip_et'
                self.prev_error = 0
                self.last_time = rospy.get_time()
                rospy.loginfo("Takip Basliyor!")
            else:
                self.rotate_robot(1) # Sola Don

        # 4. TAKIP (PID)
        elif self.state == 'duvari_takip_et':
            # Carpisma onleyici
            if dist_front < self.collision_distance:
                self.rotate_robot(1) # Sola kac
                self.prev_error = 0
                return
            
            # PID
            current_time = rospy.get_time()
            dt = current_time - self.last_time
            if dt <= 0: dt = 0.1

            # Hata Hesabi
            error = self.desired_distance_wall - dist_right
            derivative = (error - self.prev_error) / dt
            
            # PID Ciktisi
            turn_cmd = (self.kp * error) + (self.kd * derivative)
            
            self.prev_error = error
            self.last_time = current_time
            
            turn_cmd = max(min(turn_cmd, 0.6), -0.6)
            
            twist = Twist()
            twist.linear.x = self.forward_speed
            twist.angular.z = turn_cmd # PID sonucunu uygula
            self.cmd_vel_pub.publish(twist)

        if self.state != previous_state:
            self.state_pub.publish(self.state)

    def move_forward(self):
        t = Twist()
        t.linear.x = self.forward_speed
        self.cmd_vel_pub.publish(t)

    def rotate_robot(self, direction):
        t = Twist()
        t.linear.x = 0.0 
        t.angular.z = self.turning_speed * direction
        self.cmd_vel_pub.publish(t)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = WallFollowerCorrected()
        node.run()
    except rospy.ROSInterruptException:
        pass

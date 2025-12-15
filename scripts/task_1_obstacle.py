#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class EngeldenKacanRobot:
    def __init__(self):
        # Node başlatma
        rospy.init_node('gorev1_engel_kacma', anonymous=True)
        
        # Publisher ve Subscriber
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        
        # Parametreler
        [cite_start]self.safe_distance = 0.50  # Engel algılama eşiği (metre) [cite: 133]
        self.scan_percentage = 0.05 # Sağdan %5, Soldan %5 (Toplam %10 ön açı)
        self.current_state = "ILERI"
        
        # Hız mesajı nesnesi
        self.twist = Twist()
        self.rate = rospy.Rate(10) # 10 Hz döngü hızı

        rospy.loginfo("Görev 1 Başlatıldı: ROS 1 Engelden Kaçma Modu")

    def lidar_callback(self, msg):
        # 1. Lidar dizisinin toplam uzunluğu
        scan_len = len(msg.ranges)
        
        if scan_len == 0:
            return

        # 2. Taranacak indeks sayısını hesapla
        reading_count = int(scan_len * self.scan_percentage)
        
        # 3. Ön Bölgeyi (Front Cone) Oluştur
        # ROS 1'de de genellikle 0. indeks tam karşıdır (robot modeline göre değişebilir)
        left_cone = msg.ranges[0 : reading_count]
        right_cone = msg.ranges[-reading_count : ]
        front_cone = left_cone + right_cone
        
        # 4. Veriyi Temizle (inf ve 0.0 filtreleme)
        valid_distances = []
        for r in front_cone:
            if r > 0.01 and r < float('inf'):
                valid_distances.append(r)
            elif r == float('inf'):
                valid_distances.append(10.0)
        
        if not valid_distances:
            min_front_dist = 10.0
        else:
            min_front_dist = min(valid_distances)

        # --- FSM Mantığı ---
        if min_front_dist < self.safe_distance:
            self.current_state = "ENGEL_VAR"
        else:
            self.current_state = "ILERI"

        # Duruma göre hız belirle
        if self.current_state == "ILERI":
            self.twist.linear.x = 0.15
            self.twist.angular.z = 0.0
        elif self.current_state == "ENGEL_VAR":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5 # Sola dön
            rospy.loginfo("Engel: {:.2f}m - Dönülüyor...".format(min_front_dist))

        # Callback içinde publish edebiliriz veya main loop'ta.
        # Basitlik için burada yapıyoruz.
        self.pub.publish(self.twist)

    def run(self):
        # Node kapanana kadar bekle
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EngeldenKacanRobot()
        node.run()
    except rospy.ROSInterruptException:
        pass

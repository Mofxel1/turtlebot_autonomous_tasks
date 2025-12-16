#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    count = len(msg.ranges)
    if count == 0: return

    # 4 Ana Yönün İndeksleri
    idx_0   = 0                   # Array Başı (%0)
    idx_90  = int(count * 0.25)   # Çeyrek (%25)
    idx_180 = int(count * 0.50)   # Orta (%50)
    idx_270 = int(count * 0.75)   # Üç Çeyrek (%75)

    # Değerleri al (Sonsuzsa 10 yap)
    def get_val(idx):
        val = msg.ranges[idx]
        return val if val != float('inf') else 10.0

    print("-" * 30)
    print(f"Dizi Uzunlugu: {count}")
    print(f"YON 1 (idx 0   - %0) : {get_val(idx_0):.2f} m")
    print(f"YON 2 (idx {idx_90} - %25): {get_val(idx_90):.2f} m")
    print(f"YON 3 (idx {idx_180} - %50): {get_val(idx_180):.2f} m")
    print(f"YON 4 (idx {idx_270} - %75): {get_val(idx_270):.2f} m")
    print("-" * 30)

rospy.init_node('lidar_yon_test')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()

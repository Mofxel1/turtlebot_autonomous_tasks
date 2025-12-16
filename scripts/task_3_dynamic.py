#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class DynamicNavigator:
    def __init__(self):
        rospy.init_node('kobuki_dynamic_navigation')
        
        # Move Base İstemcisini Başlat
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Navigasyon sistemi (move_base) bekleniyor... Lütfen bekleyin.")
        self.client.wait_for_server()
        rospy.loginfo("Sistem Hazır! Robot emir bekliyor. 🚀")

    def send_goal(self, x, y, w=1.0):
        # Hedef mesajını oluştur
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Koordinatları ata
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = 0.0
        
        # Yönelim (1.0 = Düz duruş, istersen değiştirebilirsin)
        goal.target_pose.pose.orientation.w = w
        goal.target_pose.pose.orientation.z = 0.0

        # Gönder
        rospy.loginfo(f"Hedef Gönderildi: X={x}, Y={y}")
        self.client.send_goal(goal)
        
        # Robot gidene kadar bekle (Blokla)
        rospy.loginfo("Robot hareket halinde...")
        self.client.wait_for_result()
        
        # Sonucu kontrol et
        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ HEDEFE VARILDI!")
            return True
        else:
            rospy.logwarn("❌ HEDEFE GİDİLEMEDİ! (Engel olabilir)")
            return False

    def start_loop(self):
        while not rospy.is_shutdown():
            print("\n--------------------------------")
            print("yeni Hedef Girin (Çıkmak için 'q' basın)")
            
            try:
                x_input = input("Hedef X: ")
                if x_input.lower() == 'q':
                    break
                
                y_input = input("Hedef Y: ")
                if y_input.lower() == 'q':
                    break
                
                # Koordinatları gönder
                self.send_goal(x_input, y_input)
                
            except ValueError:
                print("Lütfen geçerli bir sayı girin!")
            except Exception as e:
                print(f"Hata oluştu: {e}")

if __name__ == '__main__':
    try:
        navigator = DynamicNavigator()
        navigator.start_loop()
    except rospy.ROSInterruptException:
        pass

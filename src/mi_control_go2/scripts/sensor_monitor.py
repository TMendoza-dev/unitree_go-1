#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import math

class SensorMonitor:
    def __init__(self):
        rospy.init_node('sensor_monitor')
        
        # Variables para almacenar estados
        self.foot_states = {
            'FL': False,
            'FR': False,
            'RL': False,
            'RR': False
        }
        
        # Suscriptores
        rospy.Subscriber('trunk_imu', Imu, self.imu_callback)
        rospy.Subscriber('FL_foot_contact', Bool, lambda msg: self.foot_callback(msg, 'FL'))
        rospy.Subscriber('FR_foot_contact', Bool, lambda msg: self.foot_callback(msg, 'FR'))
        rospy.Subscriber('RL_foot_contact', Bool, lambda msg: self.foot_callback(msg, 'RL'))
        rospy.Subscriber('RR_foot_contact', Bool, lambda msg: self.foot_callback(msg, 'RR'))

    def imu_callback(self, msg):
        # Convertir aceleración a ángulos de Euler aproximados
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Calcular roll y pitch aproximados desde aceleración
        roll = math.atan2(ay, math.sqrt(ax*ax + az*az)) * 180.0/math.pi
        pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180.0/math.pi
        
        # Mostrar datos del IMU
        rospy.loginfo('\n=== IMU Data ===')
        rospy.loginfo('Roll : %.2f°', roll)
        rospy.loginfo('Pitch: %.2f°', pitch)
        rospy.loginfo('Accel X: %.2f m/s²', ax)
        rospy.loginfo('Accel Y: %.2f m/s²', ay)
        rospy.loginfo('Accel Z: %.2f m/s²', az)
        rospy.loginfo('Angular Vel X: %.2f rad/s', msg.angular_velocity.x)
        rospy.loginfo('Angular Vel Y: %.2f rad/s', msg.angular_velocity.y)
        rospy.loginfo('Angular Vel Z: %.2f rad/s', msg.angular_velocity.z)

    def foot_callback(self, msg, foot):
        self.foot_states[foot] = msg.data
        self.print_foot_states()

    def print_foot_states(self):
        rospy.loginfo('\n=== Foot Contact States ===')
        rospy.loginfo('FL: %s FR: %s', 
                     '◼' if self.foot_states['FL'] else '◻',
                     '◼' if self.foot_states['FR'] else '◻')
        rospy.loginfo('RL: %s RR: %s', 
                     '◼' if self.foot_states['RL'] else '◻',
                     '◼' if self.foot_states['RR'] else '◻')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = SensorMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass

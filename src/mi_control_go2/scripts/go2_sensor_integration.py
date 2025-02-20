#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
import numpy as np
from collections import deque

class GO2SensorIntegration:
    def __init__(self):
        rospy.init_node('go2_sensor_integration')
        
        # Parámetros de configuración
        self.imu_window_size = 10  # Tamaño de la ventana para filtrado
        self.contact_threshold = 0.8  # Umbral para considerar contacto estable
        
        # Buffers para datos de IMU
        self.accel_buffer = deque(maxlen=self.imu_window_size)
        self.gyro_buffer = deque(maxlen=self.imu_window_size)
        
        # Estado de contacto de las patas
        self.foot_contact_states = {
            'FL': False, 'FR': False,
            'RL': False, 'RR': False
        }
        
        # Suscriptores
        rospy.Subscriber('trunk_imu', Imu, self.imu_callback)
        for foot in ['FL', 'FR', 'RL', 'RR']:
            rospy.Subscriber(
                f'{foot}_foot_contact',
                Bool,
                lambda msg, foot=foot: self.foot_contact_callback(msg, foot)
            )
        
        # Publicadores para datos procesados
        self.filtered_imu_pub = rospy.Publisher(
            'filtered_imu',
            Imu,
            queue_size=1
        )
        self.stability_pub = rospy.Publisher(
            'robot_stability',
            Vector3,
            queue_size=1
        )

    def imu_callback(self, msg):
        # Almacenar datos de IMU en buffers
        self.accel_buffer.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.gyro_buffer.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Procesar y publicar datos filtrados
        self.process_imu_data()

    def foot_contact_callback(self, msg, foot):
        self.foot_contact_states[foot] = msg.data
        self.evaluate_stability()

    def process_imu_data(self):
        if len(self.accel_buffer) < self.imu_window_size:
            return

        # Aplicar filtro de mediana móvil
        filtered_accel = np.median(self.accel_buffer, axis=0)
        filtered_gyro = np.median(self.gyro_buffer, axis=0)

        # Crear mensaje IMU filtrado
        filtered_msg = Imu()
        filtered_msg.header.stamp = rospy.Time.now()
        filtered_msg.header.frame_id = "base_link"
        
        filtered_msg.linear_acceleration.x = filtered_accel[0]
        filtered_msg.linear_acceleration.y = filtered_accel[1]
        filtered_msg.linear_acceleration.z = filtered_accel[2]
        
        filtered_msg.angular_velocity.x = filtered_gyro[0]
        filtered_msg.angular_velocity.y = filtered_gyro[1]
        filtered_msg.angular_velocity.z = filtered_gyro[2]

        self.filtered_imu_pub.publish(filtered_msg)

    def evaluate_stability(self):
        # Contar patas en contacto
        contacts = sum(self.foot_contact_states.values())
        
        # Calcular centro de apoyo aproximado
        x_stability = 0.0
        y_stability = 0.0
        
        if contacts > 0:
            # Posiciones relativas de las patas (simplificado)
            foot_positions = {
                'FL': ( 0.2,  0.15),
                'FR': ( 0.2, -0.15),
                'RL': (-0.2,  0.15),
                'RR': (-0.2, -0.15)
            }
            
            contact_sum_x = 0
            contact_sum_y = 0
            
            for foot, pos in foot_positions.items():
                if self.foot_contact_states[foot]:
                    contact_sum_x += pos[0]
                    contact_sum_y += pos[1]
            
            x_stability = contact_sum_x / contacts
            y_stability = contact_sum_y / contacts

        # Publicar métricas de estabilidad
        stability_msg = Vector3()
        stability_msg.x = x_stability
        stability_msg.y = y_stability
        stability_msg.z = float(contacts) / 4.0  # Normalizado 0-1
        
        self.stability_pub.publish(stability_msg)

    def run(self):
        rate = rospy.Rate(100)  # 100Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        sensor_integration = GO2SensorIntegration()
        sensor_integration.run()
    except rospy.ROSInterruptException:
        pass
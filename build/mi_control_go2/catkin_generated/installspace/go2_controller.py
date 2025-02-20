#!/usr/bin/env python3
import rospy
from unitree_legged_msgs.msg import MotorCmd
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import WrenchStamped

class GO2Controller:
    def __init__(self):
        rospy.init_node('go2_controller')
        
        # Modificamos el namespace para GO-2
        ns = '/go2_gazebo/'
        
        # Publishers para cada articulación
        self.publishers = {
            'FL': {
                'hip': rospy.Publisher(ns + 'FL_hip_controller/command', MotorCmd, queue_size=1),
                'thigh': rospy.Publisher(ns + 'FL_thigh_controller/command', MotorCmd, queue_size=1),
                'calf': rospy.Publisher(ns + 'FL_calf_controller/command', MotorCmd, queue_size=1)
            },
            'FR': {
                'hip': rospy.Publisher(ns + 'FR_hip_controller/command', MotorCmd, queue_size=1),
                'thigh': rospy.Publisher(ns + 'FR_thigh_controller/command', MotorCmd, queue_size=1),
                'calf': rospy.Publisher(ns + 'FR_calf_controller/command', MotorCmd, queue_size=1)
            },
            'RL': {
                'hip': rospy.Publisher(ns + 'RL_hip_controller/command', MotorCmd, queue_size=1),
                'thigh': rospy.Publisher(ns + 'RL_thigh_controller/command', MotorCmd, queue_size=1),
                'calf': rospy.Publisher(ns + 'RL_calf_controller/command', MotorCmd, queue_size=1)
            },
            'RR': {
                'hip': rospy.Publisher(ns + 'RR_hip_controller/command', MotorCmd, queue_size=1),
                'thigh': rospy.Publisher(ns + 'RR_thigh_controller/command', MotorCmd, queue_size=1),
                'calf': rospy.Publisher(ns + 'RR_calf_controller/command', MotorCmd, queue_size=1)
            }
        }

        # Suscriptores para monitorear el estado
        self.state_sub = rospy.Subscriber(ns + 'joint_states', JointState, self.state_callback)

    def state_callback(self, msg):
        # Monitorear el estado de las articulaciones
        rospy.logdebug("Received joint state update")

    def create_motor_cmd(self, position):
        """Crea un mensaje MotorCmd con los parámetros necesarios"""
        cmd = MotorCmd()
        cmd.mode = 10          # Modo de control de servomotor
        cmd.q = float(position)  # Aseguramos que sea float
        cmd.dq = 0.0          # Velocidad objetivo
        cmd.tau = 0.0         # Torque objetivo
        cmd.Kp = 100.0        # Aumentamos la ganancia proporcional
        cmd.Kd = 10.0         # Aumentamos la ganancia derivativa
        cmd.reserve = [0, 0, 0]
        return cmd

    def move_leg(self, leg, positions):
        """
        Mueve una pata específica a las posiciones dadas
        leg: 'FL', 'FR', 'RL', o 'RR'
        positions: diccionario con valores para 'hip', 'thigh', y 'calf'
        """
        for joint, position in positions.items():
            cmd = self.create_motor_cmd(position)
            self.publishers[leg][joint].publish(cmd)
            rospy.logdebug(f"Sending command to {leg}_{joint}: {position}")
        
        rospy.sleep(0.01)  # Pequeña pausa para asegurar que el mensaje se envía

    def stand_position(self):
        """Posición de pie"""
        standing_pos = {
            'hip': 0.0,
            'thigh': 0.3,    # Reducido para más estabilidad
            'calf': -0.6     # Reducido para más estabilidad
        }
        
        rospy.loginfo("Moving to standing position...")
        for leg in ['FL', 'FR', 'RL', 'RR']:
            self.move_leg(leg, standing_pos)
            rospy.sleep(0.2)
        rospy.loginfo("Standing position reached")

    def demo_sequence(self):
        """Secuencia de demostración de movimientos"""
        rospy.loginfo("Starting demo sequence...")
        
        # Definimos las posiciones que vamos a usar
        standing_pos = {
            'hip': 0.0,
            'thigh': 0.67,
            'calf': -1.3
        }
        
        test_pos = {
            'hip': 0.1,
            'thigh': 0.7,
            'calf': -1.2
        }
        
        # 1. Posición inicial
        self.stand_position()
        rospy.sleep(2.0)
        
        # 2. Pequeño movimiento de prueba
        rospy.loginfo("Testing individual leg movements...")
        for leg in ['FL', 'FR', 'RL', 'RR']:
            rospy.loginfo(f"Moving {leg}...")
            self.move_leg(leg, test_pos)
            rospy.sleep(0.5)
            self.move_leg(leg, standing_pos)
            rospy.sleep(0.5)
        
        # 3. Volver a posición de pie
        self.stand_position()

def main():
    try:
        controller = GO2Controller()
        rospy.sleep(2)  # Espera inicial para asegurar que todo está listo
        
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            controller.demo_sequence()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
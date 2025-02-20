#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool
from unitree_legged_msgs.msg import MotorCmd, HighCmd, HighState
import traceback

class AdvancedGaitController:
    def __init__(self):
        try:
            rospy.init_node('advanced_gait_controller', anonymous=True)
            rospy.loginfo("Iniciando Advanced Gait Controller...")

            self.ns = '/go2_gazebo/'
            
            # Publishers para cada articulación
            self.publishers = {
                'FL': {
                    'hip': rospy.Publisher(self.ns + 'FL_hip_controller/command', MotorCmd, queue_size=1),
                    'thigh': rospy.Publisher(self.ns + 'FL_thigh_controller/command', MotorCmd, queue_size=1),
                    'calf': rospy.Publisher(self.ns + 'FL_calf_controller/command', MotorCmd, queue_size=1)
                },
                'FR': {
                    'hip': rospy.Publisher(self.ns + 'FR_hip_controller/command', MotorCmd, queue_size=1),
                    'thigh': rospy.Publisher(self.ns + 'FR_thigh_controller/command', MotorCmd, queue_size=1),
                    'calf': rospy.Publisher(self.ns + 'FR_calf_controller/command', MotorCmd, queue_size=1)
                },
                'RL': {
                    'hip': rospy.Publisher(self.ns + 'RL_hip_controller/command', MotorCmd, queue_size=1),
                    'thigh': rospy.Publisher(self.ns + 'RL_thigh_controller/command', MotorCmd, queue_size=1),
                    'calf': rospy.Publisher(self.ns + 'RL_calf_controller/command', MotorCmd, queue_size=1)
                },
                'RR': {
                    'hip': rospy.Publisher(self.ns + 'RR_hip_controller/command', MotorCmd, queue_size=1),
                    'thigh': rospy.Publisher(self.ns + 'RR_thigh_controller/command', MotorCmd, queue_size=1),
                    'calf': rospy.Publisher(self.ns + 'RR_calf_controller/command', MotorCmd, queue_size=1)
                }
            }

            # Subscribers
            rospy.Subscriber('trunk_imu', Imu, self.imu_callback)
            rospy.Subscriber(self.ns + 'joint_states', JointState, self.joint_state_callback)
            
            for leg in ['FL', 'FR', 'RL', 'RR']:
                topic = f'/{leg}_foot_contact'
                rospy.Subscriber(topic, Bool, 
                               lambda msg, leg=leg: self.foot_contact_callback(leg, msg))

            # Estado del robot
            self.imu_data = None
            self.joint_states = None
            self.foot_contacts = {
                'FL': False, 'FR': False,
                'RL': False, 'RR': False
            }
            
            # Posiciones predefinidas
            self.positions = {
                'stand': {
                    'hip': 0.0,
                    'thigh': 0.8,
                    'calf': -1.6
                },
                'crouch': {
                    'hip': 0.0,
                    'thigh': 0.10,
                    'calf': -2
                },
                'stretch': {
                    'hip': 0.0,
                    'thigh': 0.1,    # Valor más conservador
                    'calf': -0.4     # Valor más conservador
                }
            }
            
            rospy.loginfo("Inicialización completada exitosamente")
            
        except Exception as e:
            rospy.logerr(f"Error en la inicialización: {str(e)}")
            rospy.logerr(traceback.format_exc())
            raise

    def create_motor_cmd(self, position):
        cmd = MotorCmd()
        cmd.mode = 10
        cmd.q = float(position)
        cmd.dq = 0.0
        cmd.tau = 0.0
        cmd.Kp = 100.0
        cmd.Kd = 10.0
        cmd.reserve = [0, 0, 0]
        return cmd

    def move_leg(self, leg, positions):
        try:
            for joint, position in positions.items():
                cmd = self.create_motor_cmd(position)
                self.publishers[leg][joint].publish(cmd)
                rospy.logdebug(f"Enviando comando a {leg}_{joint}: {position}")
            rospy.sleep(0.01)
        except Exception as e:
            rospy.logerr(f"Error moviendo la pata {leg}: {str(e)}")

    def move_all_legs(self, positions, duration=1.0):
        """Mueve todas las patas a las posiciones especificadas de forma gradual"""
        # Obtener posiciones actuales
        current_positions = self.positions['stand'].copy()
        
        steps = 20  # Número de pasos intermedios
        sleep_time = duration / steps
        
        for i in range(steps):
            # Calcular posiciones intermedias
            intermediate = {}
            for joint in ['hip', 'thigh', 'calf']:
                current = current_positions[joint]
                target = positions[joint]
                delta = (target - current) / steps
                intermediate[joint] = current + (delta * (i + 1))
            
            # Mover todas las patas a la posición intermedia
            for leg in ['FL', 'FR', 'RL', 'RR']:
                self.move_leg(leg, intermediate)
            
            rospy.sleep(sleep_time)

    def step_forward(self, leg):
        """Realiza un paso adelante con una pata específica"""
        # Posición inicial
        start_pos = self.positions['stand'].copy()
        
        # Levantar la pata
        lift_pos = start_pos.copy()
        lift_pos['thigh'] = 0.5
        lift_pos['calf'] = -0.8
        self.move_leg(leg, lift_pos)
        rospy.sleep(0.2)
        
        # Mover hacia adelante
        forward_pos = lift_pos.copy()
        forward_pos['hip'] = 0.2
        self.move_leg(leg, forward_pos)
        rospy.sleep(0.2)
        
        # Bajar la pata
        down_pos = forward_pos.copy()
        down_pos['thigh'] = start_pos['thigh']
        down_pos['calf'] = start_pos['calf']
        self.move_leg(leg, down_pos)
        rospy.sleep(0.2)
        
        # Volver a posición neutral
        neutral_pos = start_pos.copy()
        self.move_leg(leg, neutral_pos)
        rospy.sleep(0.2)

    def trot_sequence(self):
        """Realiza una secuencia simple de trote"""
        rospy.loginfo("Iniciando secuencia de trote...")
        
        # Secuencia diagonal
        diagonals = [('FL', 'RR'), ('FR', 'RL')]
        
        for _ in range(4):  # 4 ciclos de trote
            for pair in diagonals:
                # Levantar el par diagonal
                for leg in pair:
                    pos = self.positions['stand'].copy()
                    pos['thigh'] = 0.5
                    pos['calf'] = -0.8
                    self.move_leg(leg, pos)
                rospy.sleep(0.2)
                
                # Bajar el par diagonal
                for leg in pair:
                    self.move_leg(leg, self.positions['stand'])
                rospy.sleep(0.2)

    def demo_sequence(self):
        """Realiza una secuencia de demostración de movimientos"""
        try:
            rospy.loginfo("Iniciando secuencia de demostración...")
            
            # 1. Posición de pie
            rospy.loginfo("1. Posición de pie")
            self.move_all_legs(self.positions['stand'])
            rospy.sleep(2.0)
            
            # 2. Agacharse
            rospy.loginfo("2. Agacharse")
            self.move_all_legs(self.positions['crouch'])
            rospy.sleep(2.0)
            
            # 3. Estirarse
            #rospy.loginfo("3. Estirarse")
            #self.move_all_legs(self.positions['stretch'])
            # rospy.sleep(2.0)
            
            # 4. Volver a posición de pie
            rospy.loginfo("4. Volver a posición de pie")
            self.move_all_legs(self.positions['stand'])
            rospy.sleep(2.0)
            
            # 5. Pasos individuales
            rospy.loginfo("5. Pasos individuales")
            for leg in ['FL', 'FR', 'RL', 'RR']:
                rospy.loginfo(f"Paso con pata {leg}")
                self.step_forward(leg)
                rospy.sleep(0.5)
            
            # 6. Secuencia de trote
            rospy.loginfo("6. Secuencia de trote")
            self.trot_sequence()
            
            # 7. Finalizar en posición de pie
            rospy.loginfo("7. Finalizar en posición de pie")
            self.move_all_legs(self.positions['stand'])
            
        except Exception as e:
            rospy.logerr(f"Error en demo_sequence: {str(e)}")

    def imu_callback(self, msg):
        self.imu_data = msg

    def joint_state_callback(self, msg):
        self.joint_states = msg

    def foot_contact_callback(self, leg, msg):
        self.foot_contacts[leg] = msg.data

    def run(self):
        """Bucle principal del controlador"""
        try:
            rate = rospy.Rate(50)
            rospy.loginfo("Iniciando bucle principal...")
            
            # Esperar un momento para que todo se inicialice
            rospy.sleep(2.0)
            
            while not rospy.is_shutdown():
                try:
                    # Ejecutar la secuencia de demostración
                    self.demo_sequence()
                    
                    # Esperar antes de repetir
                    rospy.sleep(5.0)
                    
                except Exception as e:
                    rospy.logerr(f"Error en el bucle principal: {str(e)}")
                    rate.sleep()
                    continue
                    
        except Exception as e:
            rospy.logerr(f"Error fatal en run(): {str(e)}")
            rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    try:
        controller = AdvancedGaitController()
        controller.run()
    except Exception as e:
        rospy.logerr(f"Error en main: {str(e)}")
        rospy.logerr(traceback.format_exc())
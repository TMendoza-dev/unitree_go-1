#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from unitree_legged_msgs.msg import MotorCmd
import pygame
import sys
import threading
import math
import time

class GO2AdvancedController:
    def __init__(self):
        rospy.init_node('go2_advanced_controller')
        rospy.loginfo("Iniciando controlador avanzado...")
        
        # Publicadores para los joints
        self.joint_publishers = {}
        self.setup_publishers()
        
        # Variables de control
        self.movement_mode = "STAND"
        self.current_height = 0.3  # Altura inicial
        
        # Posiciones predefinidas
        self.positions = {
            'stand': {'hip': 0.0, 'thigh': 0.3, 'calf': -0.6},
            'sit': {'hip': 0.0, 'thigh': 1.0, 'calf': -1.8},
            'crouch': {'hip': 0.0, 'thigh': 0.8, 'calf': -1.6},
            'stretch': {'hip': 0.0, 'thigh': 0.1, 'calf': -0.2}
        }
        
        # Parámetros de movimiento
        self.step_height = 0.1
        self.step_length = 0.1
        self.turning_angle = 0.2
        
        # Inicializar pygame
        pygame.init()
        pygame.display.set_mode((300, 200))
        
        # Iniciar thread de teclado
        self.keyboard_thread = threading.Thread(target=self.keyboard_control)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def setup_publishers(self):
        legs = ['FL', 'FR', 'RL', 'RR']
        joints = ['hip', 'thigh', 'calf']
        for leg in legs:
            for joint in joints:
                topic = f'go2_gazebo/{leg}_{joint}_controller/command'
                self.joint_publishers[f'{leg}_{joint}'] = rospy.Publisher(topic, MotorCmd, queue_size=1)

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

    def move_leg(self, leg, positions, delay=0.1):
        """Mueve una pata específica a las posiciones dadas"""
        for joint, value in positions.items():
            joint_name = f'{leg}_{joint}'
            cmd = self.create_motor_cmd(value)
            self.joint_publishers[joint_name].publish(cmd)
        rospy.sleep(delay)

    def set_all_legs(self, positions, sequential=False):
        """Establece posiciones para todas las patas"""
        legs = ['FL', 'FR', 'RL', 'RR']
        if sequential:
            for leg in legs:
                self.move_leg(leg, positions)
        else:
            for leg in legs:
                self.move_leg(leg, positions, 0)
            rospy.sleep(0.1)

    def wave(self):
        """Hace que el robot salude con la pata delantera derecha"""
        rospy.loginfo("Saludando...")
        # Guardar posición actual
        original_pos = dict(self.positions['stand'])
        
        # Levantar la pata FR
        wave_pos = {'hip': 0.0, 'thigh': -0.3, 'calf': -0.6}
        for _ in range(3):  # Hacer el movimiento 3 veces
            self.move_leg('FR', wave_pos)
            rospy.sleep(0.3)
            self.move_leg('FR', original_pos)
            rospy.sleep(0.3)
        
        # Volver a la posición original
        self.move_leg('FR', original_pos)

    def adjust_height(self, direction):
        """Ajusta la altura del cuerpo"""
        height_delta = 0.05 * direction  # dirección: 1 para subir, -1 para bajar
        self.current_height = max(0.2, min(0.4, self.current_height + height_delta))
        
        # Calcular nuevas posiciones basadas en la altura
        thigh_angle = math.acos(self.current_height)
        new_pos = {
            'hip': 0.0,
            'thigh': thigh_angle,
            'calf': -2 * thigh_angle
        }
        self.set_all_legs(new_pos)
        rospy.loginfo(f"Altura ajustada a: {self.current_height}")

    def step_forward(self):
        """Da un paso hacia adelante"""
        rospy.loginfo("Paso adelante")
        legs_sequence = [('FL', 'RR'), ('FR', 'RL')]
        base_pos = dict(self.positions['stand'])
        
        for leg_pair in legs_sequence:
            # Levantar patas
            lift_pos = dict(base_pos)
            lift_pos['thigh'] -= self.step_height
            
            for leg in leg_pair:
                self.move_leg(leg, lift_pos)
            rospy.sleep(0.2)
            
            # Mover hacia adelante
            forward_pos = dict(lift_pos)
            forward_pos['hip'] += self.step_length
            
            for leg in leg_pair:
                self.move_leg(leg, forward_pos)
            rospy.sleep(0.2)
            
            # Bajar patas
            for leg in leg_pair:
                self.move_leg(leg, base_pos)
            rospy.sleep(0.2)

    def turn(self, direction):
        """Gira el robot (1 para derecha, -1 para izquierda)"""
        rospy.loginfo(f"Girando {'derecha' if direction > 0 else 'izquierda'}")
        angle = self.turning_angle * direction
        base_pos = dict(self.positions['stand'])
        
        # Girar patas en secuencia
        for leg in ['FL', 'FR', 'RL', 'RR']:
            turning_pos = dict(base_pos)
            turning_pos['hip'] = angle
            self.move_leg(leg, turning_pos)
            rospy.sleep(0.1)

    def keyboard_control(self):
        rospy.loginfo("""
        Controles:
        1 - Posición Stand
        2 - Posición Sit
        3 - Agacharse
        4 - Estirarse
        W - Paso adelante
        S - Paso atrás
        A - Girar izquierda
        D - Girar derecha
        Q - Subir cuerpo
        E - Bajar cuerpo
        F - Saludar
        """)
        
        while not rospy.is_shutdown():
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                
                keys = pygame.key.get_pressed()
                
                # Posiciones básicas
                if keys[pygame.K_1]:
                    self.set_all_legs(self.positions['stand'])
                elif keys[pygame.K_2]:
                    self.set_all_legs(self.positions['sit'])
                elif keys[pygame.K_3]:
                    self.set_all_legs(self.positions['crouch'])
                elif keys[pygame.K_4]:
                    self.set_all_legs(self.positions['stretch'])
                
                # Movimientos
                elif keys[pygame.K_w]:
                    self.step_forward()
                elif keys[pygame.K_s]:
                    self.step_forward()  # Implementar paso atrás
                elif keys[pygame.K_a]:
                    self.turn(-1)
                elif keys[pygame.K_d]:
                    self.turn(1)
                
                # Ajustes de altura
                elif keys[pygame.K_q]:
                    self.adjust_height(1)
                elif keys[pygame.K_e]:
                    self.adjust_height(-1)
                
                # Movimientos especiales
                elif keys[pygame.K_f]:
                    self.wave()
                
                pygame.time.wait(100)
                
            except Exception as e:
                rospy.logerr(f"Error en keyboard_control: {e}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = GO2AdvancedController()
        controller.run()
    except Exception as e:
        rospy.logerr(f"Error al iniciar el controlador: {e}")
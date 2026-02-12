#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from interfaces_pkg.msg import MotionCommand

# ==============================================================================
# 모션 플래너가 보내는 토픽 이름
SUB_TOPIC_NAME = "/topic_control_signal" 
# 가제보가 받는 토픽 이름
PUB_TOPIC_NAME = "/cmd_vel"

# 방향 보정 (1 또는 -1)
STEER_DIRECTION = -1 
MOTOR_DIRECTION = 1

# [물리 상수] 실제 차량의 물리적 한계치
MAX_SPEED_MPS = 1.7
MAX_STEER_RAD = 0.6458 
# ==============================================================================

class SendSignal():
    def __init__(self):
        pass
  
    def map_to_steer(self, input_value):
        # 입력받는 조향 값의 범위 (예: -7 ~ 7)
        input_min = -7
        input_max = 7

        # 입력값을 -1.0 ~ 1.0 사이로 정규화
        normalized_value = (input_value - input_min) / (input_max - input_min) * 2 - 1

        # 최종 조향각 계산
        wheel_angle = normalized_value * MAX_STEER_RAD
        return wheel_angle
  
    def map_to_speed(self, input_speed):      
        max_speed = MAX_SPEED_MPS
        
        # 입력받는 모터 속도 명령 범위 (예: PWM값 -255 ~ 255)
        input_min = -255
        input_max = 255

        # 입력값을 -1.0 ~ 1.0 사이로 정규화
        normalized_input_speed = (input_speed - input_min) / (input_max - input_min) * 2 - 1
        
        # m/s 단위로 변환
        mapped_input_speed = normalized_input_speed * max_speed
        return mapped_input_speed
      
    def process(self, motor):
        # 설정된 방향(STEER_DIRECTION)을 곱해 역방향/정방향 보정
        steer = self.map_to_steer(STEER_DIRECTION * motor.steering)
        left_speed = self.map_to_speed(MOTOR_DIRECTION * motor.left_speed)
    
        return steer, left_speed

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('simulation_sender_node')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST, 
            durability=QoSDurabilityPolicy.VOLATILE, 
            depth=1
        )
        
        self.simul = SendSignal()
        
        self.subscription = self.create_subscription(
            MotionCommand, 
            SUB_TOPIC_NAME, 
            self.data_callback, 
            qos_profile
        )
        
        self.publisher = self.create_publisher(
            Twist, 
            PUB_TOPIC_NAME, 
            qos_profile
        )
        
        self.timer = self.create_timer(0.1, self.send_cmd_vel)
        self.velocity = Twist()
        
    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)

    def data_callback(self, motor):
        angle, speed = self.simul.process(motor)
            
        # Twist 메시지에 값 할당
        # angular.z: 조향각 (rad)
        # linear.x: 전진 속도 (m/s)
        self.velocity.angular.z = float(angle)
        self.velocity.linear.x = float(speed)
    
        self.publisher.publish(self.velocity)
        
    def stop_cmd(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.publisher.publish(self.velocity)
        self.get_logger().error("\n\nRobot stopped\n\n")
    
def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_cmd()
        node.get_logger().fatal("\n\nsimulation_sender_node shutdown!!!\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()
  
if __name__ == '__main__':
    main()
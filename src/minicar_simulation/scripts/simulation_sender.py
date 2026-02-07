#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from interfaces_pkg.msg import MotionCommand

#------------- 설정 값 직접 정의 (Config 의존성 제거) ------------
# 모션 플래너가 보내는 토픽 이름 (launch파일에서 remapping으로 덮어씌워짐)
SUB_TOPIC_NAME = "/topic_control_signal" 
# 가제보가 받는 토픽 이름
PUB_TOPIC_NAME = "/cmd_vel"

# 방향 보정 (1 또는 -1)
STEER_DIR = -1 
MOTOR_DIR = 1

# [물리 상수] 실제 차량의 물리적 한계치
REAL_MAX_SPEED_MS = 1.2   # PWM 255일 때 약 4.3 km/h
REAL_MAX_STEER_RAD = 0.6458 # 가변저항 계산값 (약 25도)
# -----------------------------------------------------------

class SendSignal():
    def __init__(self):
        pass
  
    def map_to_steer(self, input_value):
        """
        input_value: float -1.0 (우회전) ~ 1.0 (좌회전)
        return: 가제보 조향각 (rad)
        """
        # 입력값(-1.0 ~ 1.0) * 최대 조향각(0.44 rad)
        rad_value = (input_value * STEER_DIR) * REAL_MAX_STEER_RAD
        return float(rad_value)
  
    def map_to_speed(self, input_pwm):      
        """
        input_pwm: int 0 ~ 255
        return: 가제보 선속도 m/s (float)
        """
        # 안전장치: 0~255 범위 강제
        safe_pwm = max(0, min(255, int(input_pwm)))
        
        # 비례식: (현재PWM / 255) * 최대속도(m/s)
        speed_ms = (safe_pwm / 255.0) * REAL_MAX_SPEED_MS * MOTOR_DIR
        return float(speed_ms)

    def process(self, motor):
        # motor.steering : float (-1.0 ~ 1.0)
        # motor.left_speed : int (0 ~ 255)
        
        steer_rad = self.map_to_steer(motor.steering)
        
        # 좌우 속도 평균 사용 (int -> float 변환)
        avg_pwm = (motor.left_speed + motor.right_speed) / 2
        linear_vel = self.map_to_speed(avg_pwm)
    
        return steer_rad, linear_vel

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('simulation_sender_node')
        
        # 런치 파일에서 파라미터로 토픽 이름을 바꿀 수 있게 선언
        self.declare_parameter('sub_topic', SUB_TOPIC_NAME)
        self.declare_parameter('pub_topic', PUB_TOPIC_NAME)
        
        self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST, 
            durability=QoSDurabilityPolicy.VOLATILE, 
            depth=1
        )
        
        self.simul = SendSignal()
        
        # MotionCommand 구독 (Float Steering, Int Speed)
        self.subscription = self.create_subscription(
            MotionCommand, 
            self.sub_topic, 
            self.data_callback, 
            qos_profile
        )
        
        # 가제보 cmd_vel 발행
        self.publisher = self.create_publisher(Twist, self.pub_topic, qos_profile)
        
        self.velocity = Twist()
        self.get_logger().info(f"Simulation Sender Started. Sub: {self.sub_topic}, Pub: {self.pub_topic}")
        
    def data_callback(self, motor):
        # 1. 변환 (PWM/조향값 -> m/s, rad)
        steer_rad, linear_vel = self.simul.process(motor)
            
        # 2. Twist 메시지에 할당
        self.velocity.linear.x = float(linear_vel)
        self.velocity.angular.z = float(steer_rad)
    
        # 3. 가제보로 발행
        self.publisher.publish(self.velocity)
        
    def stop_cmd(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.publisher.publish(self.velocity)
        self.get_logger().info("Robot stopped")
    
def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_cmd()
    finally:
        node.destroy_node()
        rclpy.shutdown()
  
if __name__ == '__main__':
    main()
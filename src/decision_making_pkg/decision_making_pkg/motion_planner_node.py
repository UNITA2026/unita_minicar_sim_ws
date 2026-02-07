import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

# 모션 플랜 발행 주기 (초)
TIMER = 0.1
#----------------------------------------------

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None

        self.steering_command = 0.0  # float
        self.left_speed_command = 0  # int
        self.right_speed_command = 0 # int
        

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        
    def timer_callback(self):
        # 1. 라이다 장애물 감지 -> 정지
        if self.lidar_data is not None and self.lidar_data.data is True:
            self.steering_command = 0.0
            self.left_speed_command = 0 
            self.right_speed_command = 0 

        # 2. 신호등 감지 (Red) -> 정지
        elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red':
            is_stop = False
            # detection_data가 들어왔는지 먼저 확인해야 함
            if self.detection_data is not None:
                for detection in self.detection_data.detections:
                    if detection.class_name == 'traffic_light':
                        # bbox의 우측하단 꼭짓점 y좌표
                        y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2) 

                        if y_max < 150:
                            is_stop = True
                            break
            
            if is_stop:
                self.steering_command = 0.0
                self.left_speed_command = 0 
                self.right_speed_command = 0

        # 3. 차선 추종 주행 (Lane Keeping)
        else:
            # 경로 데이터가 없거나 점이 부족하면 정지 (안전장치)
            if not self.path_data or len(self.path_data) < 10:
                self.steering_command = 0.0
                self.left_speed_command = 0
                self.right_speed_command = 0
            else:
                # 시작점(가까운 점)과 끝점(먼 점) 사이의 기울기 계산
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
            
                Kp = 0.015
                
                # 조향값 계산 (float 연산)
                steering = Kp * target_slope
                
                # [Clamp] 최대 조향각 제한 (-1.0 ~ 1.0)
                # max(-1.0, min(1.0, value)) 패턴 사용이 가장 안전함
                self.steering_command = max(-1.0, min(1.0, float(steering)))

                # 속도 설정 (0~255)
                # 코너가 심하면(조향각이 크면) 속도를 줄이는 로직 예시
                if abs(self.steering_command) > 0.5:
                    speed = 100
                else:
                    speed = 150

                self.left_speed_command = speed
                self.right_speed_command = speed

        # 로그 출력 (디버깅용)
        # float은 소수점 3자리까지 표시
        self.get_logger().info(f"Steer: {self.steering_command:.3f}, " 
                               f"Left: {self.left_speed_command}, " 
                               f"Right: {self.right_speed_command}")

        # [중요] 메시지 타입에 맞춰 형변환 후 발행
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = float(self.steering_command)    # Float32
        motion_command_msg.left_speed = int(self.left_speed_command)  # Int32
        motion_command_msg.right_speed = int(self.right_speed_command) # Int32
        
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
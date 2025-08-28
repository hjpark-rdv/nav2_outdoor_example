import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty

# GPS <-> Map 좌표 변환을 위한 서비스/메시지 타입
from robot_localization.srv import FromLL, ToLL
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped

# 현재 로봇의 위치를 알기 위한 TF 라이브러리
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

class GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # --- 내부 변수 초기화 ---
        self.waypoints = []  # GeoPoint 메시지 목록을 저장할 리스트
        self.is_navigating = False
        self.current_waypoint_index = 0

        # --- TF 리스너 초기화 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- ROS 2 클라이언트 및 서버 초기화 ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.from_ll_client = self.create_client(FromLL, '/fromLL')
        self.to_ll_client = self.create_client(ToLL, '/toLL')
        
        self.add_waypoint_service = self.create_service(Empty, '/add_gps_waypoint', self.add_waypoint_callback)
        self.start_navigation_service = self.create_service(Empty, '/start_gps_navigation', self.start_navigation_callback)

        self.get_logger().info("GPS 웨이포인트 미션 컨트롤 노드가 준비되었습니다.")
        self.get_logger().info("'/add_gps_waypoint' 서비스로 현재 위치를 추가하세요.")
        self.get_logger().info("'/start_gps_navigation' 서비스로 저장된 경로 주행을 시작하세요.")

    # 서비스 콜백: 현재 위치를 GPS 웨이포인트로 저장
    def add_waypoint_callback(self, request, response):
        try:
            # 1. 현재 로봇의 map 좌표계 위치 얻기 (map -> base_link)
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            current_map_point = Point()
            current_map_point.x = trans.transform.translation.x
            current_map_point.y = trans.transform.translation.y
            current_map_point.z = trans.transform.translation.z

            # 2. ToLL 서비스를 호출하여 map 좌표를 GPS 좌표로 변환
            to_ll_req = ToLL.Request()
            to_ll_req.map_point = current_map_point
            
            future = self.to_ll_client.call_async(to_ll_req)
            future.add_done_callback(self.to_ll_callback)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'TF 변환 실패: {e}')
        
        return response

    def to_ll_callback(self, future):
        try:
            response = future.result()
            gps_point = response.ll_point
            self.waypoints.append(gps_point)
            self.get_logger().info(f"웨이포인트 추가됨 (총 {len(self.waypoints)}개): Lat={gps_point.latitude}, Lon={gps_point.longitude}")
        except Exception as e:
            self.get_logger().error(f'/toLL 서비스 호출 실패: {e}')
            
    # 서비스 콜백: 저장된 웨이포인트 경로 주행 시작
    def start_navigation_callback(self, request, response):
        if not self.waypoints:
            self.get_logger().warn("저장된 웨이포인트가 없습니다. 주행을 시작할 수 없습니다.")
            return response
        
        if self.is_navigating:
            self.get_logger().warn("이미 주행 중입니다.")
            return response

        self.is_navigating = True
        self.current_waypoint_index = 0
        self.get_logger().info(f"총 {len(self.waypoints)}개의 GPS 웨이포인트 주행을 시작합니다.")
        self.navigate_to_next_waypoint()
        
        return response

    # 다음 웨이포인트로 주행 명령 내리기
    def navigate_to_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("모든 웨이포인트 주행을 완료했습니다!")
            self.is_navigating = False
            return

        # 현재 목표 GPS 좌표를 FromLL 서비스로 변환 요청
        target_gps = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"{self.current_waypoint_index + 1}번째 웨이포인트로 이동: Lat={target_gps.latitude}, Lon={target_gps.longitude}")
        
        from_ll_req = FromLL.Request()
        from_ll_req.ll_point = target_gps
        
        future = self.from_ll_client.call_async(from_ll_req)
        future.add_done_callback(self.from_ll_callback)

    # FromLL 서비스 응답 후 Nav2에 목표 전송
    def from_ll_callback(self, future):
        try:
            response = future.result()
            map_point = response.map_point
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = map_point.x
            goal_pose.pose.position.y = map_point.y
            goal_pose.pose.orientation.w = 1.0 # 정면

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose

            self.nav_to_pose_client.wait_for_server()
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)

        except Exception as e:
            self.get_logger().error(f'/fromLL 서비스 호출 실패: {e}')
            self.is_navigating = False

    # Nav2가 목표를 수락했는지 확인
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표가 거부되었습니다.')
            self.is_navigating = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    # 목표 지점 도착 결과 처리
    def goal_result_callback(self, future):
        result = future.result().result
        if result: # 성공적으로 도착했다면
            self.get_logger().info("웨이포인트 도착 성공!")
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint() # 다음 웨이포인트로 이동
        else:
            self.get_logger().error("웨이포인트 도착 실패.")
            self.is_navigating = False

def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

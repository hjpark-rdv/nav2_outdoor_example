import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

# 필요한 메시지 및 서비스 타입 임포트
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import ToLL
from geometry_msgs.msg import Point

class GpsAccuracyChecker(Node):
    def __init__(self):
        super().__init__('gps_accuracy_checker')

        # --- 최신 데이터를 저장할 변수 ---
        self.latest_raw_gps = None
        self.latest_fused_pose_map = None

        # --- TF 버퍼 및 리스너 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- ROS 2 구독자, 클라이언트, 타이머 ---
        self.raw_gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.raw_gps_callback,
            10)
        
        self.to_ll_client = self.create_client(ToLL, '/toLL')
        while not self.to_ll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('\'/toLL\' 서비스 대기 중...')

        # 1초마다 비교 함수를 실행할 타이머
        self.timer = self.create_timer(1.0, self.compare_poses)
        
        self.get_logger().info("GPS 정확도 검증 노드 시작. 1초마다 데이터를 비교합니다.")

    def raw_gps_callback(self, msg):
        """/gps/fix 토픽을 받을 때마다 최신 raw 데이터를 저장"""
        self.latest_raw_gps = msg

    def get_fused_pose_in_map(self):
        """TF 트리를 이용해 현재 로봇의 최종 융합 위치를 얻음"""
        try:
            now = rclpy.time.Time()
            # map -> base_link TF를 조회하여 최종 위치를 얻음
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            fused_point = Point()
            fused_point.x = trans.transform.translation.x
            fused_point.y = trans.transform.translation.y
            fused_point.z = trans.transform.translation.z
            self.latest_fused_pose_map = fused_point
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF 변환 실패 (아직 시스템이 준비 중일 수 있음): {e}')
            return False

    def compare_poses(self):
        """타이머에 의해 주기적으로 호출되는 메인 함수"""
        if self.latest_raw_gps is None:
            self.get_logger().info("아직 Raw GPS 데이터를 수신하지 못했습니다...")
            return

        if not self.get_fused_pose_in_map():
            self.get_logger().info("아직 최종 융합 위치(TF)를 얻지 못했습니다...")
            return

        # /toLL 서비스를 호출하여 최종 융합 위치(map)를 GPS로 변환
        req = ToLL.Request()
        req.map_point = self.latest_fused_pose_map
        future = self.to_ll_client.call_async(req)
        future.add_done_callback(self.to_ll_callback)

    def to_ll_callback(self, future):
        """/toLL 서비스의 응답을 받아 최종 비교 및 출력을 수행"""
        try:
            response = future.result()
            fused_gps_point = response.ll_point
            raw_gps_point = self.latest_raw_gps

            # Haversine 공식을 이용해 두 GPS 좌표 간의 거리(오차) 계산
            distance = self.haversine_distance(
                raw_gps_point.latitude, raw_gps_point.longitude,
                fused_gps_point.latitude, fused_gps_point.longitude
            )
            
            # 결과 출력
            self.get_logger().info("--- GPS 위치 비교 ---")
            self.get_logger().info(f"  - 순수 GPS (Raw)  : Lat={raw_gps_point.latitude:.8f}, Lon={raw_gps_point.longitude:.8f}")
            self.get_logger().info(f"  - 융합 GPS (Fused) : Lat={fused_gps_point.latitude:.8f}, Lon={fused_gps_point.longitude:.8f}")
            self.get_logger().info(f"  - 거리 오차 (Error)  : {distance:.4f} 미터")

        except Exception as e:
            self.get_logger().error(f'/toLL 서비스 콜백 실패: {e}')

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # 지구 반지름 (미터)
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

def main(args=None):
    rclpy.init(args=args)
    node = GpsAccuracyChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math
from pyproj import Proj, transform

class GpsOdomAdvancedComparer(Node):
    def __init__(self):
        super().__init__('gps_odom_advanced_comparator')

        # --- 상태 변수 ---
        # 기준점(Datum) 정보
        self.origin_lat = None
        self.origin_lon = None
        self.origin_utm_x = None
        self.origin_utm_y = None
        self.is_origin_set = False

        # 마지막으로 수신한 GPS 정보
        self.last_gps_msg = None
        self.last_gps_odom_pose = None # GPS 수신 시점의 odom 위치

        # 좌표 변환 객체
        self.proj_wgs84 = Proj(init='epsg:4326')  # WGS84 위도/경도
        self.proj_utm = None

        # --- Subscriber ---
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.gps_subscriber = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, qos_profile
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile
        )

        self.get_logger().info("GPS-Odom 고급 비교 노드가 시작되었습니다.")

    def gps_callback(self, msg: NavSatFix):
        """GPS 메시지 수신 콜백"""
        self.last_gps_msg = msg
        
        # 첫 GPS 메시지로 기준점(origin) 설정
        if not self.is_origin_set and msg.latitude != 0.0:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude

            # UTM 존 설정
            utm_zone = math.floor((self.origin_lon + 180) / 6) + 1
            self.proj_utm = Proj(proj='utm', zone=utm_zone, ellps='WGS84', datum='WGS84')
            
            # 기준점을 UTM 좌표로 변환
            self.origin_utm_x, self.origin_utm_y = transform(
                self.proj_wgs84, self.proj_utm, self.origin_lon, self.origin_lat
            )
            self.is_origin_set = True
            self.get_logger().info(f"기준점 GPS 설정 완료 (Lat: {self.origin_lat:.6f}, Lon: {self.origin_lon:.6f})")
            self.get_logger().info(f"UTM Zone {utm_zone} / 원점 좌표: ({self.origin_utm_x:.2f}, {self.origin_utm_y:.2f})")

    def odom_callback(self, msg: Odometry):
        """Odom 메시지 수신 콜백"""
        if not self.is_origin_set or self.last_gps_msg is None:
            self.get_logger().warn("기준점 GPS가 아직 설정되지 않았습니다.", throttle_duration_sec=5)
            return

        # 1. Odom 위치를 UTM 좌표로 변환
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        
        # Odom 변위를 기준점 UTM에 더해 현재 UTM 위치 추정
        estimated_utm_x = self.origin_utm_x + odom_x
        estimated_utm_y = self.origin_utm_y + odom_y

        # 2. 추정된 UTM 좌표를 다시 위도/경도로 변환
        estimated_lon, estimated_lat = transform(
            self.proj_utm, self.proj_wgs84, estimated_utm_x, estimated_utm_y
        )

        # 3. 실제 GPS 위치와 추정 GPS 위치 간의 거리 계산 (Haversine 공식)
        actual_lat = self.last_gps_msg.latitude
        actual_lon = self.last_gps_msg.longitude
        
        error_distance = self.haversine_distance(
            actual_lat, actual_lon, estimated_lat, estimated_lon
        )

        # 4. 마지막 GPS 수신 이후 드리프트 계산
        if self.last_gps_odom_pose is None:
             self.last_gps_odom_pose = msg.pose.pose
        
        # 새로운 GPS가 들어오면 last_gps_odom_pose를 갱신해야 함
        # (gps_callback에서 현재 odom을 저장하는 로직이 필요하지만, 단순화를 위해 odom 콜백에서 처리)
        if self.last_gps_msg.header.stamp.sec > msg.header.stamp.sec - 1.0: # 1초 이내에 GPS 갱신 시
            self.last_gps_odom_pose = msg.pose.pose
        
        drift_x = odom_x - self.last_gps_odom_pose.position.x
        drift_y = odom_y - self.last_gps_odom_pose.position.y
        drift_distance = math.sqrt(drift_x**2 + drift_y**2)

        # 5. 결과 출력
        self.get_logger().info(
            f"현재 오차: {error_distance:.3f} m | "
            f"GPS이후 드리프트: {drift_distance:.3f} m | "
            f"추정 GPS: ({estimated_lat:.6f}, {estimated_lon:.6f}) | "
            f"실제 GPS: ({actual_lat:.6f}, {actual_lon:.6f})",
            throttle_duration_sec=0.2 # 25hz에 맞춰 0.04초마다 출력은 너무 많으므로 조절
        )

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """두 위도/경도 지점 간의 거리를 미터 단위로 계산"""
        R = 6371000  # 지구 반지름 (미터)
        
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad

        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomAdvancedComparer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
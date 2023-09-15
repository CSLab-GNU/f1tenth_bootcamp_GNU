import numpy as np
import math
# from main import GymRunner


class FGM_REF_WAY:
    def __init__(self, params=None):

        self.RACECAR_LENGTH = 0.3302
        self.PI = 3.141592
        self.ROBOT_SCALE = 0.2032

        self.LOOK = 2.5  
        self.THRESHOLD = 4.5 
        self.FILTER_SCALE = 1.1
        self.GAP_THETA_GAIN = 20.0
        self.REF_THETA_GAIN = 1.5

        self.BEST_POINT_CONV_SIZE = 160

        self.waypoint_real_path = 'pkg/Oschersleben_2_wp.csv'

        self.waypoint_delimeter = ','

        self.scan_range = 1080
        self.desired_gap = 0
        self.desired_wp_rt = [0, 0]

        self.wp_num = 1
        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0
        
        self.max_angle = 0
        self.wp_angle = 0
        self.gaps = []
        self.fail_gap = [0, 0, 0]

        self.interval = 0.00435  

        self.front_idx = 0
        self.theta_for = self.PI / 3

        self.current_speed = 0
        self.dmin_past = 0
        
        self.waypoints = self.get_waypoint()


    def get_waypoint(self): 
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter, dtype='float')

        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        return temp_waypoint

    def getDistance(self, a, b): 
        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return np.sqrt(dx ** 2 + dy ** 2)

    def transformPoint(self, origin, target):
        theta = self.PI / 2 - origin[2]

        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dtheta = target[2] + theta

        tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
        tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
        tf_point_theta = dtheta
        tf_point = [tf_point_x, tf_point_y, tf_point_theta]

        return tf_point

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        # rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x * x + y * y))
        rtpoint.append(np.arctan2(y, x) - (self.PI / 2))

        return rtpoint

    def find_desired_wp(self):

        """
        현재 목표로 하는 waypoint를 찾는 함수
        """

        wp_index_temp = self.wp_index_current
        
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)  
        
        while True:
            wp_index_temp += 1

            if wp_index_temp >= self.wp_num - 1:
                wp_index_temp = 0

            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position) 

            if temp_distance < self.nearest_distance: 
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
               

            elif (temp_distance > (self.nearest_distance + self.LOOK * 1.2)) or (wp_index_temp == self.wp_index_current): 
                break


        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.wp_num - 1:
                idx_temp = 0
            temp_distance = self.getDistance(self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.LOOK: break
            idx_temp += 1
    
        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)


    def subCallback_scan(self, scan_data): 
        """
        현재 스캔 데이터와 주변 스캔 데이터를 비교하면서 노이즈가 있는 경우, 해당 스캔 데이터를 인근 값으로 대체하여 LiDAR 데이터 전처리 
        """
        self.front_idx = (int(self.scan_range / 2))

        self.scan_origin = [0] * self.scan_range
        self.scan_filtered = [0] * self.scan_range

        for i in range(self.scan_range):
            self.scan_origin[i] = scan_data[i]
            self.scan_filtered[i] = scan_data[i]

        for i in range(self.scan_range - 1):
            if self.scan_origin[i] * self.FILTER_SCALE < self.scan_filtered[i + 1]:
                unit_length = self.scan_origin[i] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 1
                while j < filter_num + 1:
                    if i + j < self.scan_range:
                        if self.scan_filtered[i + j] > self.scan_origin[i]:
                            self.scan_filtered[i + j] = self.scan_origin[i]
                        else:
                            break
                    else:
                        break
                    j += 1

            elif self.scan_filtered[i] > self.scan_origin[i + 1] * self.FILTER_SCALE:
                unit_length = self.scan_origin[i + 1] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i - j > 0:
                        if self.scan_filtered[i - j] > self.scan_origin[i + 1]:
                            self.scan_filtered[i - j] = self.scan_origin[i + 1]
                        else:
                            break
                    else:
                        break
                    j += 1

        return self.scan_filtered

    def find_gap(self, scan): 
        """
        조건에 만족하는 gaps를 찾는 함수
        """
        self.gaps = []

        i = 0

        while i < self.scan_range :
            
            if scan[i] > self.THRESHOLD: 
                start_idx_temp = i
                end_idx_temp = i

                while ((scan[i] > self.THRESHOLD) and (i + 1 < self.scan_range)):
                    i += 1

                if scan[i] > self.THRESHOLD:
                    i += 1
                end_idx_temp = i

                gap_size = np.fabs(end_idx_temp - start_idx_temp)

                if gap_size < 30: # gap 조건을 충족하는 연속되는 LiDAR 값이 30보다 작으면 gap으로 인정하지 않음
                    i += 1
                    continue

                gap_temp = [0] * 2
                gap_temp[0] = start_idx_temp
                gap_temp[1] = end_idx_temp
                
                self.gaps.append(gap_temp)
    
            i += 1
    
    def fail_find_gap(self):
        """
        gap을 찾지 못했을 경우 (여러 조건등을 추가할 수 있음 정지, 후진 , 전방에 임시 gap 생성...)
        """ 
        
        start_idx_temp = (self.front_idx) - 240
        end_idx_temp = (self.front_idx) + 240

        self.fail_gap[0] = start_idx_temp
        self.fail_gap[1] = end_idx_temp

    

    def find_best_gap(self, ref): 
        """
        갭들 중에 목표로 하는 waypoint의 방향에 가장 가까운 갭을 선택함
        """
        num = len(self.gaps)

        if num == 0: # gap을 찾지 못했을 경우
            return self.fail_gap
        
        else:
            step = (int(ref[1] / self.interval))
            ref_idx = self.front_idx + step

            gap_idx = 0

            if self.gaps[0][0] > ref_idx:
                distance = self.gaps[0][0] - ref_idx
            elif self.gaps[0][1] < ref_idx:
                distance = ref_idx - self.gaps[0][1]
            else:
                distance = 0
                gap_idx = 0

            i = 1
            while (i < num): # distance를 갱신하면서 가장 가까운 갭 선택
                if self.gaps[i][0] > ref_idx:
                    temp_distance = self.gaps[i][0] - ref_idx
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i
                elif self.gaps[i][1] < ref_idx:
                    temp_distance = ref_idx - self.gaps[i][1]
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i

                else:
                    temp_distance = 0
                    distance = 0
                    gap_idx = i
                    break

                i += 1
            
            return self.gaps[gap_idx]

    def find_best_point(self, best_gap): 
        """
        갭에서 최적의 포인트(인덱스)를 찾는 함수 
        """

        averaged_max_gap = np.convolve(self.scan_filtered[best_gap[0]:best_gap[1]], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        
        return averaged_max_gap.argmax() + best_gap[0]


    def calculate_steering_and_speed(self, best_point):
        self.gap_angle = (best_point - self.front_idx) * self.interval  
        self.wp_angle = self.desired_wp_rt[1]

        temp_avg = 0
        dmin = 0
        for i in range(10):
            dmin += self.scan_filtered[i]
        dmin /= 10

        i = 0

        while i < self.scan_range - 7:
            j = 0
            while j < 10:
                if i + j > 1079:
                    temp_avg += 0
                else:
                    temp_avg += self.scan_filtered[i + j]
                j += 1

            temp_avg /= 10 # 인덱스 i로부터 10개 인덱스의 거리 데이터 평균

            if dmin > temp_avg: #  dmin을 가장 작은 temp_avg값으로 갱신 , 장애물들이 충분히 멀리 있음 -> dmin값 증가 -> GAP THETA GAIN 감소 -> FGM 영향력 낮아짐
                if temp_avg == 0:
                    temp_avg = dmin
                dmin = temp_avg
            temp_avg = 0
            i += 3

        if dmin == 0:
            dmin = self.dmin_past

        controlled_angle = ((self.GAP_THETA_GAIN / dmin) * self.gap_angle + self.REF_THETA_GAIN * self.wp_angle) / (
                self.GAP_THETA_GAIN / dmin + self.REF_THETA_GAIN)
 
        distance = 1.0 + (self.current_speed*0.001)
        
        path_radius = distance / (2 * np.sin(controlled_angle))
       
        steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)

        steer = steering_angle

        
        self.dmin_past = dmin
        self.THRESHOLD = 2.5

        speed = 8
    
        return steer, speed 


    def main_drive(self, scan_data, odom_data):
        """
        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """
        scan_data = self.subCallback_scan(scan_data)
        self.current_position = [odom_data['pose_x'], odom_data['pose_y'], odom_data['pose_theta']]
        self.current_speed = odom_data['linear_vel_x']

        
        self.find_desired_wp()
        self.find_gap(scan_data)
        self.fail_find_gap()


        self.desired_gap = self.find_best_gap(self.desired_wp_rt)
        self.best_point = self.find_best_point(self.desired_gap)

        steer, speed = self.calculate_steering_and_speed(self.best_point)

        
        return speed, steer


    def process_observation(self, ranges, ego_odom):
        if ego_odom:
            return self.main_drive(ranges, ego_odom)
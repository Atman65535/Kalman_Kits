import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter1d:
    def __init__(self):
        self.theta = 300
        self.bias = 0.0001
        self.P = np.array([[15, 0.0],
                           [0.0, 1.7]])
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.3

    def update(self, new_angle, new_rate, dt):
        """update _summary_

        Arguments:
            new_angle {_type_} -- _description_
            new_rate {_type_} -- _description_
            dt {_type_} -- _description_

        Returns:
            _type_ -- _description_
        """
        # estimate
        self.theta = self.theta + new_rate * dt - self.bias * dt
        self.bias = self.bias
        # update covariance matrix
        self.P[0][0] = self.P[0, 0] - dt * (self.P[1][0] + self.P[0][1]) + dt ** 2 * self.P[1][1] + self.Q_angle * dt
        self.P[0][1] = self.P[0, 1] - dt * self.P[1][1]
        self.P[1][0] = self.P[1, 0] - dt * self.P[1, 1]
        self.P[1, 1] = self.P[1, 1] + self.Q_bias * dt
        # Kalman Gain
        K = np.array([self.P[0, 0],
                      self.P[1, 0]]) / (self.P[0, 0] + self.R_measure)
        # fusion
        self.theta = self.theta + (new_angle - self.theta) * K[0]
        self.bias = self.bias + (new_angle - self.theta) * K[1]
        # final covariance
        self.P[0][0] = self.P[0, 0] - K[0] * self.P[0][0]
        self.P[0][1] = self.P[0, 1] - K[0] * self.P[0][1]
        self.P[1][0] = self.P[1, 0] - K[1] * self.P[0, 0]
        self.P[1, 1] = self.P[1, 1] - K[1] * self.P[0, 1]

        return self.theta


class KalmanFilter2d:
    def __init__(self, dt):
        self.dt = dt
        self.angle_trans = 0.003
        self.bias_trans = 0.001
        self.observe = 0.03
        self.states = np.array([[0],
                                [0],
                                [0],
                                [0]])  # roll, pitch, bias roll, bias pitch
        self.P = np.zeros(shape=(4, 4))
        self.F = np.identity(n=4)
        self.F[0, 2] = -dt
        self.F[1, 3] = -dt
        self.G = np.array([[dt, 0],
                           [0, dt],
                           [0, 0],
                           [0, 0]])
        self.Q = np.identity(n=4)
        self.Q[0, 0] = self.angle_trans
        self.Q[1, 1] = self.angle_trans
        self.Q[2, 2] = self.bias_trans
        self.Q[3, 3] = self.bias_trans
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.R = np.array([[self.observe, 0],
                           [0, self.observe]])

    def update(self, theta, newrate):
        self.states = self.F @ self.states + self.G @ newrate
        self.P = self.F @ self.P @ self.F.transpose() + self.Q * self.dt
        K = self.P @ self.H.transpose() @ np.linalg.inv(self.H @ self.P @ self.H.transpose() + self.R)
        self.states = self.states + K @ (theta - self.H @ self.states)
        self.P = (np.identity(4) - K @ self.H) @ self.P
        return self.states[0, 0], self.states[1, 0]


class EKF3d:
    def quaternion_mul(self, q1, q2):
        s1, x1, y1, z1 = q1.flatten()
        s2, x2, y2, z2 = q2.flatten()
        return np.array([s1 * s2 - x1 * x2 - y1 * y2 - z1 * z2,
                         s1 * x2 + x1 * s2 + y1 * z2 - z1 * y2,
                         s1 * y2 - x1 * z2 + y1 * s2 + z1 * x2,
                         s1 * z2 + x1 * y2 - y1 * x2 + z1 * s2])

    def skew_symmetric(self, v):
        x, y, z = v.flatten()
        return np.array([[0, -z, y],
                         [z, 0, -x],
                         [-y, x, 0]])

    def quat_to_rot(self, q):
        v = q[1:]
        skew = self.skew_symmetric(v)
        return v.reshape(3, 1)@v.reshape(1, 3) + q[0]**2 * np.identity(3) + 2 * q[0] * skew + skew @ skew

    def quat_left(self, q)-> np.ndarray:
        s = q[0]
        v = q[1:].reshape(3, 1)
        return np.block([[s, -v.T],
                         [v, s*np.identity(3) + self.skew_symmetric(v)]])

    def quat_right(self, q):
        s = q[0]
        v = q[1:].reshape(3, 1)
        return np.block([[s, -v.T],
                         [v, s*np.identity(3) - self.skew_symmetric(v)]])
    
    def build_H(self, q):
        s, x, y, z = q
        m1 = 2 * np.array([[-y, z, -s, x],
                              [x, s, z, y],
                              [s, -x, -y, z]]) 
        return np.hstack([m1, np.zeros(shape=(3,3))])

    def __init__(self):
        self.quat = np.array([1, 0, 0, 0])  # quaternion s x y z b
        self.bias_rad = np.array([0, 0, 0])
        self.P = np.zeros(shape=(7, 7))
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6])
        self.R = np.diag([8e-2, 8e-2, 8e-2])
        self.g = np.array([0, 0, 1]).reshape(3, 1)

    def update(self, g_deg, a, dt):
        a = a/ np.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
        g_rad = g_deg / 360 * 2 *np.pi - self.bias_rad
        d_theta = np.linalg.norm(g_rad) * dt
        n_omega = g_rad / np.sqrt(g_rad[0]**2 + g_rad[1]**2 + g_rad[2]**2)
        n_tmp = n_omega * np.sin(d_theta/2)
        q_theta = np.array([np.cos(d_theta / 2), n_tmp[0], n_tmp[1], n_tmp[2]])
        q_theta_rough = np.array([1, g_rad[0]*dt/2, g_rad[1]*dt/2, g_rad[2]*dt/2])
        last_quat = self.quat
        self.quat = self.quaternion_mul(self.quat, q_theta) 
        self.quat = self.quat / np.linalg.norm(self.quat)
        tmp = self.quat_left(last_quat) @ (-dt / 2  * np.array([[0, 0, 0],
                                                               [1, 0, 0],
                                                               [0, 1, 0],
                                                               [0, 0, 1]]))
        F = np.block([[self.quat_right(q_theta_rough), tmp],
                      [np.zeros(shape=(3, 4)), np.identity(3)]])
        self.P = F @ self.P @ F.transpose() + self.Q
        H = self.build_H(self.quat)
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + self.R)
        x = np.block([self.quat, self.bias_rad]).reshape(7, 1)
        x = x + K@(a.reshape(3, 1) - self.quat_to_rot(self.quat).T @ self.g)
        x = x.reshape(1, 7).flatten()
        self.quat = x[:4]
        self.quat = self.quat / np.sqrt(self.quat[0]**2 + self.quat[1]**2 + self.quat[2]**2 + self.quat[3]**2)
        self.bias_rad = x[4:]
        self.P = (np.identity(7) - K@H)@self.P
        return self.quat
    
    def quat_to_euler(self, q):
        w, x, y, z = q
        
        # 1. Roll (绕 X 轴旋转)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # 2. Pitch (绕 Y 轴旋转)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            # 使用 copysign 处理万向节死锁 (Pitch = ±90度)
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # 3. Yaw (绕 Z 轴旋转)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch])
    def draw(self, qs, original_roll, original_pitch ):
        euler_angles = [self.quat_to_euler(q) for q in qs] 

        plt.figure(figsize=(10, 6))
        plt.plot(np.degrees(euler_angles))
        plt.plot( original_roll, alpha = 0.5)
        plt.plot( original_pitch, alpha=0.5)
        plt.legend(['Roll', 'Pitch', "ori ro", "ori pi"])
        plt.title('EKF Estimated Orientation (Degrees)')
        plt.grid(True)
        plt.show()

class ESKF3d:
    def quaternion_mul(self, q1, q2):
        s1, x1, y1, z1 = q1.flatten()
        s2, x2, y2, z2 = q2.flatten()
        return np.array([s1 * s2 - x1 * x2 - y1 * y2 - z1 * z2,
                         s1 * x2 + x1 * s2 + y1 * z2 - z1 * y2,
                         s1 * y2 - x1 * z2 + y1 * s2 + z1 * x2,
                         s1 * z2 + x1 * y2 - y1 * x2 + z1 * s2]).reshape(4,)

    def skew_symmetric(self, v):
        x, y, z = v.flatten()
        return np.array([[0, -z, y],
                         [z, 0, -x],
                         [-y, x, 0]])

    def quat_to_rot(self, q):
        v = q[1:]
        skew = self.skew_symmetric(v)
        return v.reshape(3, 1)@v.reshape(1, 3) + q[0]**2 * np.identity(3) + 2 * q[0] * skew + skew @ skew

    def quat_left(self, q)-> np.ndarray:
        s = q[0]
        v = q[1:].reshape(3, 1)
        return np.block([[s, -v.T],
                         [v, s*np.identity(3) + self.skew_symmetric(v)]])

    def quat_right(self, q):
        s = q[0]
        v = q[1:].reshape(3, 1)
        return np.block([[s, -v.T],
                         [v, s*np.identity(3) - self.skew_symmetric(v)]])
    def quat_to_euler(self, q):
        w, x, y, z = q
        
        # 1. Roll (绕 X 轴旋转)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # 2. Pitch (绕 Y 轴旋转)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            # 使用 copysign 处理万向节死锁 (Pitch = ±90度)
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # 3. Yaw (绕 Z 轴旋转)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch])
    def draw(self, qs, original_roll, original_pitch ):
        euler_angles = [self.quat_to_euler(q) for q in qs] 

        plt.figure(figsize=(10, 6))
        plt.plot(np.degrees(euler_angles))
        plt.plot( original_roll, alpha = 0.5)
        plt.plot( original_pitch, alpha=0.5)
        plt.legend(['Roll', 'Pitch', "ori ro", "ori pi"])
        plt.title('ESKF Estimated Orientation (Degrees)')
        plt.grid(True)
        plt.show()

    def __init__(self):
        self.nomq = np.array([1, 0, 0, 0]).reshape(4,)
        self.bias = np.array([0, 0, 0])
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6])
        self.R = np.diag([5e-2, 5e-2, 5e-2])
        self.P = np.diag([0, 0, 0, 1e-2, 1e-2, 1e-2])
        self.gw = np.array([0, 0, 1]).reshape(3, 1)

    def update(self, gdeg, a, dt):
        a = a/ np.linalg.norm(a)
        g_rad = np.deg2rad(gdeg).reshape(3,) - self.bias.reshape(3,)
        d_theta = np.linalg.norm(g_rad) * dt
        n_tmp = g_rad / np.linalg.norm(g_rad) * np.sin(d_theta/2)
        q_theta = np.array([np.cos(d_theta / 2), n_tmp[0], n_tmp[1], n_tmp[2]])
        q_theta_rough = np.array([1, g_rad[0]*dt/2, g_rad[1]*dt/2, g_rad[2]*dt/2])
        self.nomq = self.quat_right(q_theta) @ self.nomq.reshape(4, 1)
        self.nomq /= np.linalg.norm(self.nomq)
        self.nomq.reshape(4,)
        F = np.block([[self.quat_to_rot(q_theta).T, -dt * np.identity(3)],
                      [np.zeros(shape=(3, 3)), np.identity(3)]])
        self.P = F @ self.P @ F.T + self.Q 
        H = np.block([[self.skew_symmetric(self.quat_to_rot(self.nomq).T @ self.gw), np.zeros(shape=(3,3))]])
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + self.R)
        del_x = K @ (a.reshape(3, 1) - self.quat_to_rot(self.nomq).T @ self.gw.reshape(3, 1))
        del_x = del_x.reshape(6,)
        x = del_x[:3]
        b = del_x[3:]
        theta_norm = np.linalg.norm(x)
        del_q = np.array([np.cos(theta_norm/2), 
                      x[0]/theta_norm * np.sin(theta_norm/2), 
                      x[1]/theta_norm * np.sin(theta_norm/2), 
                      x[2]/theta_norm * np.sin(theta_norm/2)])
        self.nomq = self.quaternion_mul(self.nomq, del_q)
        self.nomq /= np.linalg.norm(self.nomq)
        self.bias = self.bias + b
        self.P = (np.identity(6) - K @ H) @ self.P
        self.P = np.diag([1, 1, 1, 1, 1, 1]) @ self.P @ np.diag([1, 1, 1, 1, 1, 1])
        return self.nomq


def draw_plots(original_roll, filtered_roll, timestamps):
    plt.figure(figsize=(12, 6))
    plt.plot(timestamps, original_roll, label='Original Roll', color='blue', alpha=0.5)
    plt.plot(timestamps, filtered_roll, label='Filtered Roll (Kalman)', color='red')
    plt.xlabel('Timestamp (ms)')
    plt.ylabel('Roll Angle (degrees)')
    plt.title('Roll Angle: Original vs Kalman Filtered')
    plt.legend()
    plt.grid()
    plt.show()


def main():
    filter = EKF3d()
    with open('/home/atman/a_workspace/Kalman_Kits/KalmanKitsFor32/logs/device-monitor-260119-214015.log', 'r') as f:
        lines = f.readlines()
        original_roll = []
        filtered_roll = []
        original_pitch = []
        filtered_pitch = []
        timestamps = []
        qs = []
        timestamp = 0
        cnt = 0
        for line in lines:
            values = line.strip().split(',')
            timestamp = int(values[0])
            if cnt != 3:
                cnt += 1
                continue
            gx = float(values[1])
            gy = float(values[2])
            gz = float(values[3])
            ax = float(values[4])
            ay = float(values[5])
            az = float(values[6])
            roll = np.arctan2(ay, az) * 180 / np.pi  # rad 2 deg
            pitch = np.arctan2(-ax, np.sqrt(ay ** 2 + az ** 2)) * 180 / np.pi
            original_roll.append(roll)
            original_pitch.append(pitch)
            filtered = filter.update(np.array([gx, gy, gz]),np.array([ax, ay, az]), dt=0.005)
            qs.append(filtered)
            cnt = 0
        filter.draw(qs, original_roll, original_pitch)


if __name__ == "__main__":
    main()

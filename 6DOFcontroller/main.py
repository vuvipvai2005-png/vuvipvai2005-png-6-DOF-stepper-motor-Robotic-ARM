import serial
import time
import math
import numpy as np


class RobotArmController:
    def __init__(self, port, baudrate=115200):
        self.serial = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Đợi ESP32 khởi động

        # --- THÔNG SỐ DH & ĐỘNG HỌC ---
        self.C_D1 = 188.0;
        self.C_D2 = 3.85;
        self.C_D6 = 69.35
        self.C_A1 = 32.18;
        self.C_A2 = 161.373
        self.C_LEFF = 133.4990
        self.C_PHI_NEW = math.atan(13340.0 / 514.0)

    def to_rad(self, deg):
        return deg * math.pi / 180.0

    def to_deg(self, rad):
        return rad * 180.0 / math.pi

    def normalize(self, deg):
        while deg > 180.0: deg -= 360.0
        while deg < -180.0: deg += 360.0
        return deg

    # ====== TOÁN HỌC (DEBUG TRÊN PC) ======
    def solveFK6(self, q1, q2, q3, q4, q5, q6):
        dh_d = [188.0, -3.85, 0.0, 133.4, 0.0, 69.35]
        dh_a = [32.18, 161.373, 5.14, 0.0, 0.0, 0.0]
        dh_alpha = [-90, 0, -90, 90, -90, 0]
        th = [self.to_rad(q1), self.to_rad(q2 - 90), self.to_rad(q3),
              self.to_rad(q4), self.to_rad(q5), self.to_rad(q6)]

        T = np.identity(4)
        for i in range(6):
            ct = math.cos(th[i]);
            st = math.sin(th[i])
            ca = math.cos(self.to_rad(dh_alpha[i]));
            sa = math.sin(self.to_rad(dh_alpha[i]))
            a = dh_a[i];
            d = dh_d[i]
            Ti = np.array([
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])
            T = np.dot(T, Ti)

        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        pitch = self.to_deg(math.asin(max(min(-T[2, 0], 1.0), -1.0)))
        if abs(math.cos(self.to_rad(pitch))) > 1e-6:
            roll = self.to_deg(math.atan2(T[2, 1], T[2, 2]))
            yaw = self.to_deg(math.atan2(T[1, 0], T[0, 0]))
        else:
            roll = 0
            yaw = self.to_deg(math.atan2(-T[0, 1], T[1, 1]))
        return x, y, z, roll, pitch, yaw

    def solveIK6(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        # (Viết lại theo đúng logic C++ của bạn để debug)
        roll = self.to_rad(roll_deg);
        pitch = self.to_rad(pitch_deg);
        yaw = self.to_rad(yaw_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)

        R06 = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])

        wx = x - self.C_D6 * R06[0, 2]
        wy = y - self.C_D6 * R06[1, 2]
        wz = z - self.C_D6 * R06[2, 2]
        r_xy = math.sqrt(wx ** 2 + wy ** 2)
        if r_xy < self.C_D2: return False, []

        alpha = math.atan2(wy, wx)
        beta = math.asin(self.C_D2 / r_xy)
        q1 = self.to_deg(alpha + beta)

        proj = wx * math.cos(self.to_rad(q1)) + wy * math.sin(self.to_rad(q1))
        r_eff = proj - self.C_A1
        z_eff = wz - self.C_D1
        A = self.C_A2;
        B = self.C_LEFF

        dist_sq = z_eff ** 2 + r_eff ** 2
        cos_K = (dist_sq - A ** 2 - B ** 2) / (2 * A * B)
        if abs(cos_K) > 1.0: return False, []

        K = math.acos(cos_K)
        q3 = self.to_deg(K - self.C_PHI_NEW)

        U = A + B * math.cos(K);
        V = B * math.sin(K)
        det = U ** 2 + V ** 2
        c2_real = (z_eff * U + r_eff * V) / det
        s2_real = (r_eff * U - z_eff * V) / det
        q2 = self.to_deg(math.atan2(s2_real, c2_real))

        # Tính R36
        th1, th2, th3 = self.to_rad(q1), self.to_rad(q2 - 90.0), self.to_rad(q3)
        R01 = np.array([[math.cos(th1), 0, -math.sin(th1)], [math.sin(th1), 0, math.cos(th1)], [0, -1, 0]])
        R12 = np.array([[math.cos(th2), -math.sin(th2), 0], [math.sin(th2), math.cos(th2), 0], [0, 0, 1]])
        R23 = np.array([[math.cos(th3), 0, -math.sin(th3)], [math.sin(th3), 0, math.cos(th3)], [0, -1, 0]])
        R03 = np.dot(np.dot(R01, R12), R23)
        R36 = np.dot(R03.T, R06)

        sq = math.sqrt(R36[2, 0] ** 2 + R36[2, 1] ** 2)
        theta5 = math.atan2(sq, R36[2, 2])
        theta4 = math.atan2(-R36[1, 2], -R36[0, 2])
        theta6 = math.atan2(R36[2, 1], -R36[2, 0])

        q4a = self.normalize(self.to_deg(theta4))
        q5a = self.to_deg(theta5)

        q4b = self.normalize(q4a + 180.0)
        q5b = -q5a

        if (abs(q4b) + abs(q5b)) < (abs(q4a) + abs(q5a)):
            q4, q5, q6 = q4b, q5b, self.normalize(self.to_deg(theta6))
        else:
            q4, q5, q6 = q4a, q5a, self.normalize(self.to_deg(theta6) + 180)

        return True, [q1, q2, q3, q4, q5, q6]

    # ====== GIAO TIẾP SERIAL ======
    def send_cmd(self, cmd):
        self.serial.write(f"{cmd}\n".encode())
        print(f"Sent: {cmd}")
        time.sleep(0.1)
        while self.serial.in_waiting:
            print(self.serial.readline().decode().strip())

    def home(self):
        self.send_cmd("H")

    def get_location(self):
        self.send_cmd("L")

    def move_manual(self, joint_idx, delta_angle):
        self.send_cmd(f"M {joint_idx} {delta_angle}")

    def move_joint(self, q1, q2, q3, q4, q5, q6):
        # Tính thử FK trên PC xem đúng không trước khi gửi
        x, y, z, r, p, yw = self.solveFK6(q1, q2, q3, q4, q5, q6)
        print(f"[PC Debug] Vị trí FK tương ứng: X:{x:.2f} Y:{y:.2f} Z:{z:.2f}")
        self.send_cmd(f"I {q1} {q2} {q3} {q4} {q5} {q6}")

    def move_cartesian(self, x, y, z, r, p, yw):
        # Tính IK trên PC để kiểm tra giới hạn khớp
        success, joints = self.solveIK6(x, y, z, r, p, yw)
        if success:
            print(f"[PC Debug] IK OK! Joints: {joints}")
            self.send_cmd(f"F {x} {y} {z} {r} {p} {yw}")
        else:
            print("[PC Debug] Lỗi IK: Không thể với tới tọa độ này!")

    def execute_trajectory(self, x, y, z, r, p, yw, config=0):
        # Lệnh T: Chạy Trajectory tuyến tính
        print(f"Bắt đầu quỹ đạo tuyến tính tới: {x}, {y}, {z}")
        self.send_cmd(f"T {x} {y} {z} {r} {p} {yw} {config}")


# --- SỬ DỤNG (GIAO DIỆN TERMINAL TƯƠNG TÁC) ---
if __name__ == "__main__":
    # Nhớ đổi "COM3" thành cổng COM thực tế của ESP32 trên máy bạn
    try:
        robot = RobotArmController(port="COM9", baudrate=115200)
        print("=== GIAO DIỆN ĐIỀU KHIỂN ROBOT ARM ===")
        print("Đã kết nối MCU! Gõ 'exit' hoặc 'quit' để thoát.")
        print("Các lệnh hỗ trợ: ")
        print("  H : Về Home toàn bộ")
        print("  L : Đọc vị trí hiện tại")
        print("  M <joint> <delta> : Quay thủ công (VD: M 1 10.5)")
        print("  I <q1> <q2> <q3> <q4> <q5> <q6> : Tính IK trên ESP32")
        print("  F <q1> <q2> <q3> <q4> <q5> <q6> : Chạy PTP theo góc khớp (VD: F 12 12 23 34 70 80)")
        print("  T <x> <y> <z> <r> <p> <yw>      : Chạy Trajectory thẳng nội suy S-Curve")
        print("----------------------------------------")

        while True:
            # Chờ người dùng nhập lệnh từ bàn phím
            # Ví dụ bạn gõ: F 10 10 99 12 23 12 + I 100 100 30 45 45 45
            user_input = input("Nhập lệnh: ").strip()

            if user_input.lower() in ['exit', 'quit']:
                print("Đã thoát giao diện điều khiển.")
                break

            if not user_input:
                continue

            # --- CẮT CHUỖI BẰNG DẤU '+' ---
            commands = user_input.split('+')

            for cmd in commands:
                cmd = cmd.strip()  # Dọn dẹp khoảng trắng 2 đầu của từng lệnh nhỏ
                if cmd:  # Nếu lệnh không rỗng thì mới gửi
                    # Hàm send_cmd của bạn đã có sẵn mã f"{cmd}\n".encode()
                    # nên nó sẽ tự động nối đuôi \n cho từng lệnh rồi mới gửi xuống STM32
                    robot.send_cmd(cmd)

                    # Tạm dừng 0.05s để STM32 kịp ngắt UART và nhét lệnh vào Hàng đợi (Ring Buffer)
                    time.sleep(0.05)
    except serial.SerialException as e:
        print(f"Lỗi kết nối cổng COM: {e}")
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;

// --- KHAI BÁO HÀNG ĐỢI LỆNH (COMMAND QUEUE) ---
#define MAX_QUEUE_SIZE 20
char cmd_queue[MAX_QUEUE_SIZE][100];
volatile uint8_t q_head = 0;
volatile uint8_t q_tail = 0;
volatile uint8_t q_count = 0;

char rx_buffer[100];
volatile uint8_t rx_index = 0;
uint8_t rx_char;

#define PI 3.14159265358979323846f
#define NUM_JOINT 6

float toRad(float deg) { return deg * PI / 180.0f; }
float toDeg(float rad) { return rad * 180.0f / PI; }

typedef struct {
  GPIO_TypeDef* pul_port;
  uint16_t pul_pin;
  GPIO_TypeDef* dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef* limit_port;
  uint16_t limit_pin;
  float gear_ratio;
  float step_per_deg;
  float max_v;
  float max_a;
  bool invert_dir;
  float current_angle;
  bool home_dir;
  bool has_home;
} Joint;

typedef struct {
  int32_t steps_total;
  int32_t steps_done;
  float accel;
  float vmax;
  float t_acc;
  float T;
  bool is_trajectory; float constant_v;
    uint32_t last_step_us; float step_error_accum;

} Motion;

Joint joint[NUM_JOINT] = {
  {GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_7, GPIOB, GPIO_PIN_10, 6.667f, 0, 4000, 2000, false, 0.0f, GPIO_PIN_RESET, true},   // J1
  {GPIOB, GPIO_PIN_5, GPIOA, GPIO_PIN_6, GPIOB, GPIO_PIN_12,  19.22f, 0, 4000, 2000, true,  0.0f, GPIO_PIN_SET,   true},   // J2
  {GPIOB, GPIO_PIN_6, GPIOA, GPIO_PIN_5, GPIOB, GPIO_PIN_1,  26.85f, 0, 4000, 2000, true,  0.0f, GPIO_PIN_SET,   true},   // J3
  {GPIOB, GPIO_PIN_7, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0,  3.33f,  0, 4000, 3000, false,  0.0f, GPIO_PIN_SET,   true},  // J4
  {GPIOB, GPIO_PIN_8, GPIOA, GPIO_PIN_1, GPIOB, GPIO_PIN_3,  24.789f, 0, 4000, 3000, true,  0.0f, GPIO_PIN_SET,   true},   // J5
  {GPIOB, GPIO_PIN_9, GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_9,  1.0f,   0, 5000, 5000, false, 0.0f, GPIO_PIN_RESET, false}   // J6
};

// Thông số bảng DH
const float DH_D1 = 188.5f, DH_A1 = 32.18f, DH_D2 = -3.85f, DH_A2 = 161.39f, DH_A3 = 12.0f, DH_D4 = 133.4f, DH_D6 = 115.85f, DH_A6 = -28.0f;
const float C_A1 = 32.18f, C_A2 = 161.39f, C_D2 = 3.85f, C_D1 = 188.5f, C_D6 = 115.85f,C_A6 = -28.0f;
const float C_LEFF = 133.9386f;
const float C_PHI_NEW = 1.48112f;

#define TRAJ_DT 0.005f
#define MAX_TRAJ_POINTS 3000
float joint_traj[MAX_TRAJ_POINTS][6];
int traj_points_count = 0, traj_current_index = 0;
bool run_trajectory = false;

float HOME_OFFSET[NUM_JOINT] = {103.0f, 42.7f, 113.5f, -169.0f, 74.0f, 0.0f};
#define MOTOR_STEP_PER_REV 200.0f
#define MICROSTEP          8.0f

int8_t joint_to_zero_index = -1;
float traj_start[6], traj_end[6], traj_total_time = 0.0f;
const float TRAJ_SPEED_MM_S = 60.0f, TRAJ_SPEED_DEG_S = 20.0f;

Motion motion[NUM_JOINT];
float target_angle[NUM_JOINT];
float start_angle[NUM_JOINT];
bool moving = false;
uint32_t t_start_us;
float current_move_duration = 0.0f;
bool ik_debug_enable = true;
uint32_t micros(void) {
  return __HAL_TIM_GET_COUNTER(&htim2);
}

void delayMicroseconds(uint32_t us) {
  uint32_t start = micros();
  while ((micros() - start) < us);
}

void set_motor_dir(int i, bool positive_direction) {
  GPIO_PinState level;
  if (joint[i].invert_dir)
    level = positive_direction ? GPIO_PIN_RESET : GPIO_PIN_SET;
  else
    level = positive_direction ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(joint[i].dir_port, joint[i].dir_pin, level);
}

float calculate_min_time(int32_t steps, float v_max, float accel) {
  if (steps == 0) return 0.0f;

  float t_acc_needed = v_max / accel;
  float s_acc_dec = accel * t_acc_needed * t_acc_needed;

  if (s_acc_dec >= steps)
    return 2.0f * sqrtf((float)steps / accel);
  else
    return (2.0f * t_acc_needed) + ((steps - s_acc_dec) / v_max);
}

float velocity_at(int i, float t) {
  if (motion[i].is_trajectory) return motion[i].constant_v;

  if (t < motion[i].t_acc) return motion[i].accel * t;
  else if (t < (motion[i].T - motion[i].t_acc)) return motion[i].vmax;
  else return motion[i].accel * (motion[i].T - t);
}

void plan_sync_motion() {
  float max_motion_time = 0.0f;

  for (int i = 0; i < NUM_JOINT; i++) {
    motion[i].is_trajectory = false;

    float delta = target_angle[i] - joint[i].current_angle;
    float exact_steps = (delta * joint[i].step_per_deg) + motion[i].step_error_accum;
    int32_t step_to_run = (int32_t)roundf(exact_steps);

    motion[i].step_error_accum = exact_steps - (float)step_to_run;
    motion[i].steps_total = labs(step_to_run);
    set_motor_dir(i, (step_to_run > 0));

    motion[i].steps_done = 0;
    motion[i].last_step_us = 0;

    float t_needed = calculate_min_time(motion[i].steps_total, joint[i].max_v, joint[i].max_a);
    if (t_needed > max_motion_time) max_motion_time = t_needed;
  }

  current_move_duration = max_motion_time;

  for (int i = 0; i < NUM_JOINT; i++) {
    motion[i].T = max_motion_time;
    int32_t S = motion[i].steps_total;

    if (S == 0) {
      motion[i].vmax = 0.0f;
      motion[i].accel = 0.0f;
      continue;
    }

    float t_acc_new = max_motion_time / 4.0f;
    float v_new = (float)S / (max_motion_time - t_acc_new);
    float a_new = v_new / t_acc_new;

    motion[i].vmax = v_new;
    motion[i].accel = a_new;
    motion[i].t_acc = t_acc_new;
  }
}

void plan_trajectory_step() {
  float max_time_needed = 0.0f;

  for (int i = 0; i < NUM_JOINT; i++) {
    motion[i].is_trajectory = true;
    motion[i].steps_done = 0;
    motion[i].last_step_us = 0;

    float delta = target_angle[i] - joint[i].current_angle;
    float exact_steps = (delta * joint[i].step_per_deg) + motion[i].step_error_accum;
    int32_t step_to_run = (int32_t)roundf(exact_steps);

    motion[i].step_error_accum = exact_steps - (float)step_to_run;
    motion[i].steps_total = labs(step_to_run);
    set_motor_dir(i, (step_to_run > 0));

    if (motion[i].steps_total > 0) {
      float min_t = (float)motion[i].steps_total / (joint[i].max_v * 0.95f);
      if (min_t > max_time_needed) max_time_needed = min_t;
    }
  }

  float actual_step_time = (max_time_needed > TRAJ_DT) ? max_time_needed : TRAJ_DT;
  current_move_duration = actual_step_time;

  for (int i = 0; i < NUM_JOINT; i++) {
    if (motion[i].steps_total > 0)
      motion[i].constant_v = (float)motion[i].steps_total / actual_step_time;
    else
      motion[i].constant_v = 0.0f;
  }
}

void update_motion() {
  float t = (micros() - t_start_us) / 1e6f;

  for (int i = 0; i < NUM_JOINT; i++) {
    if (motion[i].steps_total == 0 || motion[i].steps_done >= motion[i].steps_total) {
      continue;
    }

    float v = velocity_at(i, t);
    if (v <= 0.0f) continue;

    uint32_t interval = (uint32_t)(1e6f / v);
    uint32_t now = micros();

    if (now - motion[i].last_step_us >= interval) {
      HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_SET);
      delayMicroseconds(2);
      HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_RESET);

      motion[i].last_step_us = now;
      motion[i].steps_done++;
    }
  }

  // THÊM: cập nhật góc hiện tại liên tục để gửi lên ROS mượt hơn
  for (int i = 0; i < NUM_JOINT; i++) {
    if (motion[i].steps_total > 0) {
      float ratio = (float)motion[i].steps_done / (float)motion[i].steps_total;
      if (ratio > 1.0f) ratio = 1.0f;

      joint[i].current_angle = start_angle[i] +
          (target_angle[i] - start_angle[i]) * ratio;
    } else {
      joint[i].current_angle = target_angle[i];
    }
  }

  if (t > current_move_duration) {
    moving = false;

    for (int i = 0; i < NUM_JOINT; i++) {
      joint[i].current_angle = target_angle[i];
    }

    if (joint_to_zero_index != -1) {
      int j = joint_to_zero_index;
      joint[j].current_angle = 0.0f;
      target_angle[j] = 0.0f;
      joint_to_zero_index = -1;
    }
  }
}

float normalize(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

void matMul33(float A[3][3], float B[3][3], float C[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C[i][j] = 0;
      for (int k = 0; k < 3; k++) C[i][j] += A[i][k] * B[k][j];
    }
  }
}

void matMul44(float A[4][4], float B[4][4], float C[4][4]) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      C[i][j] = 0;
      for (int k = 0; k < 4; k++) C[i][j] += A[i][k] * B[k][j];
    }
  }
}

bool checkJointLimit(float j1, float j2, float j3, float j4, float j5) {
  if (j1 < -103 || j1 > 109) return false;
  if (j2 < -42  || j2 > 105) return false;
  if (j3 < -114 || j3 > 80) return false;
  if (j4 < -180 || j4 > 180) return false;
  if (j5 < -90  || j5 > 90)  return false;
  return true;
}

void solveFK6(float q1, float q2, float q3, float q4, float q5, float q6,
              float *x, float *y, float *z, float *roll, float *pitch, float *yaw) {
  float dh_d[] = {DH_D1, DH_D2, 0.0f, DH_D4, 0.0f, DH_D6};
  float dh_a[] = {DH_A1, DH_A2, DH_A3, 0.0f, 0.0f, DH_A6};
  float dh_alpha[] = {-90.0f, 0.0f, -90.0f, 90.0f, -90.0f, 0.0f};
  float th[] = {toRad(q1), toRad(q2 - 90.0f), toRad(q3), toRad(q4), toRad(q5), toRad(q6)};

  float T[4][4] = {
    {1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
  };

  for (int i = 0; i < 6; i++) {
    float ct = cosf(th[i]), st = sinf(th[i]);
    float ca = cosf(toRad(dh_alpha[i])), sa = sinf(toRad(dh_alpha[i]));
    float a = dh_a[i], d = dh_d[i];

    float Ti[4][4] = {
      {ct, -st*ca,  st*sa, a*ct},
      {st,  ct*ca, -ct*sa, a*st},
      {0,      sa,     ca,    d},
      {0,       0,      0,    1}
    };

    float T_new[4][4];
    matMul44(T, Ti, T_new);
    memcpy(T, T_new, sizeof(T));
  }

  *x = T[0][3];
  *y = T[1][3];
  *z = T[2][3];

  float r11 = T[0][0], r12 = T[0][1], r21 = T[1][0], r22 = T[1][1];
  float r31 = T[2][0], r32 = T[2][1], r33 = T[2][2];

  *pitch = toDeg(asinf(-r31));
  if (fabsf(cosf(toRad(*pitch))) > 1e-6f) {
    *roll = toDeg(atan2f(r32, r33));
    *yaw  = toDeg(atan2f(r21, r11));
  } else {
    *roll = 0.0f;
    *yaw  = toDeg(atan2f(-r12, r22));
  }

  printf("FK: %.2f %.2f %.2f %.2f %.2f %.2f\n", *x, *y, *z, *roll, *pitch, *yaw);
}
float angle_diff(float a, float b) {
    float d = a - b;
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return fabsf(d);
}


bool solveIK6(float x, float y, float z, float roll_deg, float pitch_deg, float yaw_deg,
              float *q1, float *q2, float *q3, float *q4, float *q5, float *q6, int config) {
  float roll = toRad(roll_deg);
  float pitch = toRad(pitch_deg);
  float yaw = toRad(yaw_deg);

  float cy = cosf(yaw), sy = sinf(yaw);
  float cp = cosf(pitch), sp = sinf(pitch);
  float cr = cosf(roll), sr = sinf(roll);

  float R06[3][3];
  R06[0][0] = cy*cp;
  R06[0][1] = cy*sp*sr - sy*cr;
  R06[0][2] = cy*sp*cr + sy*sr;
  R06[1][0] = sy*cp;
  R06[1][1] = sy*sp*sr + cy*cr;
  R06[1][2] = sy*sp*cr - cy*sr;
  R06[2][0] = -sp;
  R06[2][1] = cp*sr;
  R06[2][2] = cp*cr;

  float wx = x - C_D6 * R06[0][2] - C_A6 * R06[0][0];
  float wy = y - C_D6 * R06[1][2] - C_A6 * R06[1][0];
  float wz = z - C_D6 * R06[2][2] - C_A6 * R06[2][0];

  float r_xy = sqrtf(wx*wx + wy*wy);
  if (r_xy < C_D2) return false;

  float alpha = atan2f(wy, wx);
  float beta = asinf(C_D2 / r_xy);
  *q1 = toDeg(alpha + beta);

  float proj = wx*cosf(toRad(*q1)) + wy*sinf(toRad(*q1));
  float r_eff = proj - C_A1;
  float z_eff = wz - C_D1;

  float A = C_A2;
  float B = C_LEFF;

  float dist_sq = z_eff*z_eff + r_eff*r_eff;
  float cos_K = (dist_sq - A*A - B*B) / (2.0f * A * B);
  if (fabsf(cos_K) > 1.0f) return false;

  float K = acosf(cos_K);
  float q3_rad = K - C_PHI_NEW;
  *q3 = toDeg(q3_rad);

  float U = A + B * cosf(K);
  float V = B * sinf(K);
  float det = U*U + V*V;

  float c2_real = (z_eff*U + r_eff*V) / det;
  float s2_real = (r_eff*U - z_eff*V) / det;
  *q2 = toDeg(atan2f(s2_real, c2_real));

  float th1 = toRad(*q1);
  float th2 = toRad(*q2 - 90.0f);
  float th3 = toRad(*q3);

  float c1 = cosf(th1), s1 = sinf(th1);
  float R01[3][3] = {{c1, 0, -s1}, {s1, 0, c1}, {0, -1, 0}};

  float c2 = cosf(th2), s2 = sinf(th2);
  float R12[3][3] = {{c2, -s2, 0}, {s2, c2, 0}, {0, 0, 1}};

  float c3 = cosf(th3), s3 = sinf(th3);
  float R23[3][3] = {{c3, 0, -s3}, {s3, 0, c3}, {0, -1, 0}};

  float R02[3][3];
  float R03[3][3];
  matMul33(R01, R12, R02);
  matMul33(R02, R23, R03);

  float R36[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R36[i][j] = 0;
      for (int k = 0; k < 3; k++) R36[i][j] += R03[k][i] * R06[k][j];
    }
  }

  // Tính R36 (Giữ nguyên)
    float sq = sqrtf(R36[2][0]*R36[2][0] + R36[2][1]*R36[2][1]);
    float theta5 = atan2f(sq, R36[2][2]);
    float theta4 = 0;
    float theta6 = 0;
          theta4 = atan2f(R36[1][2], R36[0][2]);
          theta6 = atan2f(R36[2][1], -R36[2][0]);

    // Nghiệm A
    float q4a = normalize(toDeg(theta4));
    float q5a = -toDeg(theta5);
    float q6a = normalize(toDeg(theta6));

    // Nghiệm B
    float q4b = normalize(toDeg(theta4) + 180.0f);
    float q5b = toDeg(theta5);
    float q6b = normalize(toDeg(theta6) + 180.0f);
    // --- THUẬT TOÁN SHORTEST PATH ---

        float ref_q4 = *q4; float ref_q5 = *q5; float ref_q6 = *q6;

        // Tính quãng đường cho thuật toán tự động
        float diff_a = angle_diff(q4a, ref_q4) + angle_diff(q5a, ref_q5) + angle_diff(q6a, ref_q6);
        float diff_b = angle_diff(q4b, ref_q4) + angle_diff(q5b, ref_q5) + angle_diff(q6b, ref_q6);

        if (config == 1) {
            // Cưỡng chế dùng Nghiệm 1 (IK1)
            *q4 = q4a; *q5 = q5a; *q6 = q6a;
        }
        else if (config == 2) {
            // Cưỡng chế dùng Nghiệm 2 (IK2)
            *q4 = q4b; *q5 = q5b; *q6 = q6b;
        }
        else {
            // config == 0: Trở về chế độ Tự động chọn đường ngắn nhất
            if (diff_b < diff_a) {
                *q4 = q4b; *q5 = q5b; *q6 = q6b;
            } else {
                *q4 = q4a; *q5 = q5a; *q6 = q6a;
            }
        }

        // Trải phẳng góc Euler
        *q4 = ref_q4 + normalize(*q4 - ref_q4);
        *q5 = ref_q5 + normalize(*q5 - ref_q5);
        *q6 = ref_q6 + normalize(*q6 - ref_q6);

        if (ik_debug_enable) {
            printf("IK: %.2f %.2f %.2f %.2f %.2f %.2f\n",
                   *q1, *q2, *q3, *q4, *q5, *q6);
        }
    if (!checkJointLimit(*q1,*q2,*q3,*q4,*q5)){ printf("MCU: Limit Err\n"); return false; }
    return true;
}

void move_joint_to_zero(int i) {
  float delta = 0.0f - joint[i].current_angle;
  if (fabsf(delta) < 0.01f) return;

  int32_t steps = (int32_t)fabsf(delta * joint[i].step_per_deg);
  set_motor_dir(i, (delta > 0));

  for (int32_t k = 0; k < steps; k++) {
    HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_SET);
    delayMicroseconds(500);
    HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_RESET);
    delayMicroseconds(500);
  }

  joint[i].current_angle = 0.0f;
}
bool limit_pressed(int i)
{
    if(i != 3)
    {
        return (HAL_GPIO_ReadPin(joint[i].limit_port,
                                 joint[i].limit_pin)
                == GPIO_PIN_RESET);
    }

    // J4 lọc nhiễu
    if(HAL_GPIO_ReadPin(joint[i].limit_port,
                        joint[i].limit_pin)
       != GPIO_PIN_RESET)
        return false;

    HAL_Delay(10);

    return (HAL_GPIO_ReadPin(joint[i].limit_port,
                             joint[i].limit_pin)
            == GPIO_PIN_RESET);
}
void home_joint(int i) {
  if (!joint[i].has_home) {
    joint[i].current_angle = 0.0f;
    return;
  }

  while (limit_pressed(i)) {
    HAL_GPIO_WritePin(joint[i].dir_port, joint[i].dir_pin, !joint[i].home_dir);
    while (HAL_GPIO_ReadPin(joint[i].limit_port, joint[i].limit_pin) == GPIO_PIN_RESET) {
      HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_SET); delayMicroseconds(2000);
      HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_RESET); delayMicroseconds(2000);
    }
    HAL_Delay(200);
  }

  HAL_GPIO_WritePin(joint[i].dir_port, joint[i].dir_pin, joint[i].home_dir);
  while (!limit_pressed(i)) {
    HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_SET); delayMicroseconds(500);
    HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_RESET); delayMicroseconds(500);
  }
  HAL_Delay(100);

  HAL_GPIO_WritePin(joint[i].dir_port, joint[i].dir_pin, !joint[i].home_dir);

  int backoff_steps = (i == 2) ? 1700 : 900;
  int backoff_delay = (i == 2) ? 1000 : 2000;   // nhanh gấp 2

  for (int k = 0; k < backoff_steps; k++) {
      HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_SET);
      delayMicroseconds(backoff_delay);

      HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_RESET);
      delayMicroseconds(backoff_delay);
  }
  HAL_Delay(100);

  HAL_GPIO_WritePin(joint[i].dir_port, joint[i].dir_pin, joint[i].home_dir);
  while (!limit_pressed(i)) {
    HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_SET); delayMicroseconds(2000);
    HAL_GPIO_WritePin(joint[i].pul_port, joint[i].pul_pin, GPIO_PIN_RESET); delayMicroseconds(2000);
  }

  joint[i].current_angle = -HOME_OFFSET[i];
  HAL_Delay(200);
  move_joint_to_zero(i);
}

void homing_all() {
	home_joint(2);
	home_joint(1);
  home_joint(0);
  home_joint(3);
  home_joint(4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (rx_char == '\n') {
      rx_buffer[rx_index] = '\0';

      if (q_count < MAX_QUEUE_SIZE) {
        strcpy(cmd_queue[q_head], rx_buffer);
        q_head = (q_head + 1) % MAX_QUEUE_SIZE;
        q_count++;
      } else {
        printf("Error: Queue Full!\n");
      }

      rx_index = 0;
    } else if (rx_char != '\r') {
      rx_buffer[rx_index++] = rx_char;
      if (rx_index >= 100) rx_index = 0;
    }

    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  }
}

bool read_serial_command() {
  if (q_count == 0) return false;

  char current_cmd[100];

  __disable_irq();
  strcpy(current_cmd, cmd_queue[q_tail]);
  q_tail = (q_tail + 1) % MAX_QUEUE_SIZE;
  q_count--;
  __enable_irq();

  if (strlen(current_cmd) == 0) return false;

  char mode = current_cmd[0];
  char *input = &current_cmd[1];

  if (mode == 'H') {
    homing_all();
    return false;
  }

  if (mode == 'M') {
    float j, delta;
    if (sscanf(input, "%f %f", &j, &delta) == 2) {
      int j_idx = (int)j;
      if (j_idx >= 0 && j_idx < NUM_JOINT) {
        target_angle[j_idx] = joint[j_idx].current_angle + delta;
        for (int i = 0; i < NUM_JOINT; i++)
          if (i != j_idx) target_angle[i] = joint[i].current_angle;
        joint_to_zero_index = j_idx;
        return true;
      }
    }
    return false;
  }

  if (mode == 'L') {
    float x, y, z, r, p, yw;
    solveFK6(joint[0].current_angle, joint[1].current_angle, joint[2].current_angle,
             joint[3].current_angle, joint[4].current_angle, joint[5].current_angle,
             &x, &y, &z, &r, &p, &yw);
    printf("POSE: X=%.2f Y=%.2f Z=%.2f  R=%.2f P=%.2f Y=%.2f\n", x, y, z, r, p, yw);
    return false;
  }

  if (mode == 'T' || mode == 'I' || mode == 'F') {
	  float v[6];
	      int config_flag = 0; // Mặc định là 0 (Tự động chọn đường ngắn nhất)

	      // Quét chuỗi: Lấy 6 số thực, và [tùy chọn] 1 số nguyên ở cuối
	      int parsed = sscanf(input, "%f %f %f %f %f %f %d", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &config_flag);

	      if (parsed >= 6) { // Phải đọc được ít nhất 6 số mới chạy
	        if (mode == 'I') {
	          float q[6];
	          for (int i = 0; i < 6; i++) q[i] = joint[i].current_angle;

	          // Truyền config_flag vào giải IK
	          if (solveIK6(v[0], v[1], v[2], v[3], v[4], v[5],
	                       &q[0], &q[1], &q[2], &q[3], &q[4], &q[5], config_flag)) {
	            for (int i = 0; i < 6; i++) target_angle[i] = q[i];
	            return true;
	          } else return false;
	        }
      else if (mode == 'F') {
        if (!checkJointLimit(v[0], v[1], v[2], v[3], v[4])) return false;

        for (int i = 0; i < 6; i++) target_angle[i] = v[i];

        float s_x, s_y, s_z, s_r, s_p, s_yw;
        solveFK6(v[0], v[1], v[2], v[3], v[4], v[5], &s_x, &s_y, &s_z, &s_r, &s_p, &s_yw);
        return true;
      }
      else if (mode == 'T') {
        float s_x, s_y, s_z, s_r, s_p, s_yw;
        solveFK6(joint[0].current_angle, joint[1].current_angle, joint[2].current_angle,
                 joint[3].current_angle, joint[4].current_angle, joint[5].current_angle,
                 &s_x, &s_y, &s_z, &s_r, &s_p, &s_yw);

        traj_start[0] = s_x;  traj_start[1] = s_y;  traj_start[2] = s_z;
        traj_start[3] = s_r;  traj_start[4] = s_p;  traj_start[5] = s_yw;

        // 1. Nạp Tọa độ X, Y, Z (Tuyến tính, không bị vòng lặp nên copy thẳng)
                traj_end[0] = v[0];
                traj_end[1] = v[1];
                traj_end[2] = v[2];

                // 2. Nạp Góc Roll, Pitch, Yaw (THUẬT TOÁN CHỐNG LẶP +-180 ĐỘ)
                // Tìm đường quay góc ngắn nhất để chống hiện tượng robot tự xoay 360 độ
                traj_end[3] = traj_start[3] + normalize(v[3] - traj_start[3]);
                traj_end[4] = traj_start[4] + normalize(v[4] - traj_start[4]);
                traj_end[5] = traj_start[5] + normalize(v[5] - traj_start[5]);

        float dist = sqrtf(
          powf(traj_end[0] - traj_start[0], 2) +
          powf(traj_end[1] - traj_start[1], 2) +
          powf(traj_end[2] - traj_start[2], 2)
        );

        float t_pos = dist / TRAJ_SPEED_MM_S;

        float max_ang_diff = 0.0f;
        for (int i = 3; i < 6; i++) {
          max_ang_diff = fmaxf(max_ang_diff, fabsf(traj_end[i] - traj_start[i]));
        }

        float t_ang = max_ang_diff / TRAJ_SPEED_DEG_S;
        traj_total_time = fmaxf(t_pos, t_ang);
        if (traj_total_time < 0.5f) traj_total_time = 0.5f;

        traj_points_count = (int)(traj_total_time / TRAJ_DT);
        if (traj_points_count >= MAX_TRAJ_POINTS) traj_points_count = MAX_TRAJ_POINTS - 1;

        // --- BẮT ĐẦU VÒNG LẶP QUỸ ĐẠO ---
                        // Khởi tạo mảng "Tham chiếu" bằng vị trí hiện tại của cánh tay
                        float ref_q[6];
                        for(int i=0; i<6; i++) ref_q[i] = joint[i].current_angle;
                        ik_debug_enable = false;
                        for (int k = 0; k <= traj_points_count; k++) {
                            float t = k * TRAJ_DT;
                            float ratio = t / traj_total_time;
                            if (ratio > 1.0f) ratio = 1.0f;

                            // Hàm Smoothstep Bậc 3 chống giật (Jerk) mà ta đã bàn
                            float s = ratio * ratio * (3.0f - 2.0f * ratio);

                            float x  = traj_start[0] + s * (traj_end[0] - traj_start[0]);
                            float y  = traj_start[1] + s * (traj_end[1] - traj_start[1]);
                            float z  = traj_start[2] + s * (traj_end[2] - traj_start[2]);
                            float r  = traj_start[3] + s * (traj_end[3] - traj_start[3]);
                            float p  = traj_start[4] + s * (traj_end[4] - traj_start[4]);
                            float yw = traj_start[5] + s * (traj_end[5] - traj_start[5]);

                            // Nạp góc của điểm trước đó (ref_q) vào biến tạm để IK lấy làm gốc
                            float temp_q[6];
                            for(int i=0; i<6; i++) temp_q[i] = ref_q[i];

                            // Giải IK. Nếu thất bại, hủy luôn quỹ đạo.
                            if (!solveIK6(x, y, z, r, p, yw, &temp_q[0], &temp_q[1], &temp_q[2], &temp_q[3], &temp_q[4], &temp_q[5], config_flag)) {
                                            printf("Traj Err at point %d\n", k);
                                            return false;
                            }

                            // Lưu kết quả vào mảng chạy, đồng thời cập nhật biến tham chiếu cho điểm tiếp theo
                            for(int i=0; i<6; i++) {
                                joint_traj[k][i] = temp_q[i];
                                ref_q[i] = temp_q[i];
                            }
                        }
                        ik_debug_enable = true;
                        traj_current_index = 0; run_trajectory = true; return false;
      }

    }
  }
  if (mode == 'C') {
          float cx, cy, cz, R, roll, pitch, yaw;
          int config_flag = 0; // Mặc định tự động chọn cấu hình nghiệm

          int parsed = sscanf(input, "%f %f %f %f %f %f %f %d",
                              &cx, &cy, &cz, &R, &roll, &pitch, &yaw, &config_flag);

          if (parsed >= 7) {
              // 1. Tính toán điểm xuất phát (Góc 0 độ trên mặt phẳng XY)
              float start_x = cx + R;
              float start_y = cy;
              float start_z = cz;

              char cmd_i[100];
              char cmd_circle[100];

              // 2. Tạo lệnh I giả lập để chạy an toàn tới điểm xuất phát
              sprintf(cmd_i, "I %.2f %.2f %.2f %.2f %.2f %.2f %d",
                      start_x, start_y, start_z, roll, pitch, yaw, config_flag);

              // 3. Tạo lệnh @ (Lệnh vòng tròn nội bộ) để thực thi ngay sau đó
              sprintf(cmd_circle, "@ %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d",
                      cx, cy, cz, R, roll, pitch, yaw, config_flag);

              // Nhét 2 lệnh này vào hàng đợi một cách an toàn
              __disable_irq();
              if (q_count <= MAX_QUEUE_SIZE - 2) {
                  strcpy(cmd_queue[q_head], cmd_i);
                  q_head = (q_head + 1) % MAX_QUEUE_SIZE;
                  q_count++;

                  strcpy(cmd_queue[q_head], cmd_circle);
                  q_head = (q_head + 1) % MAX_QUEUE_SIZE;
                  q_count++;
              } else {
                  printf("Loi: Hang doi day, khong the nap lenh Circle!\n");
              }
              __enable_irq();

              // Trả về false để vòng lặp while(1) tự động nhặt lệnh I ra chạy tiếp
              return false;
          }
      }

      // ==============================================================
      // LỆNH @: LỆNH NỘI BỘ THỰC THI QUỸ ĐẠO ĐƯỜNG TRÒN (CHẠY SAU LỆNH I)
      // ==============================================================
      if (mode == '@') {
          float cx, cy, cz, R, roll, pitch, yaw;
          int config_flag = 0;

          int parsed = sscanf(input, "%f %f %f %f %f %f %f %d",
                              &cx, &cy, &cz, &R, &roll, &pitch, &yaw, &config_flag);

          if (parsed >= 7) {
              // Chu vi của vòng tròn
              float arc_length = 2.0f * PI * R;

              // Tính toán thời gian quỹ đạo để khớp với vận tốc cài đặt
              traj_total_time = arc_length / TRAJ_SPEED_MM_S;
              if (traj_total_time < 0.5f) traj_total_time = 0.5f; // Tránh lỗi chia 0

              traj_points_count = (int)(traj_total_time / TRAJ_DT);
              if (traj_points_count >= MAX_TRAJ_POINTS) traj_points_count = MAX_TRAJ_POINTS - 1;

              // Gốc tham chiếu chống lỗi xoắn cổ tay
              float ref_q[6];
              for(int i=0; i<6; i++) ref_q[i] = joint[i].current_angle;
              ik_debug_enable = false;
              // Nội suy sinh điểm Cartesian -> Joint
              for (int k = 0; k <= traj_points_count; k++) {
                  float ratio = (float)k / traj_points_count;

                  // Vẫn dùng Smoothstep để robot bắt đầu vẽ và kết thúc từ từ, chống giật móp méo
                  float s = ratio * ratio * (3.0f - 2.0f * ratio);

                  // Quét góc từ 0 -> 360 độ (2*PI)
                  float current_theta_deg = s * 360.0f;

                  // Phương trình đường tròn cơ bản trên mặt phẳng XY
                  float x = cx + R * cosf(toRad(current_theta_deg));
                  float y = cy + R * sinf(toRad(current_theta_deg));
                  float z = cz;

                  float temp_q[6];
                  for(int i=0; i<6; i++) temp_q[i] = ref_q[i];

                  // Giải Inverse Kinematics
                  if (!solveIK6(x, y, z, roll, pitch, yaw,
                                &temp_q[0], &temp_q[1], &temp_q[2],
                                &temp_q[3], &temp_q[4], &temp_q[5], config_flag)) {
                      printf("Loi IK tai diem vong tron thu %d\n", k);
                      return false;
                  }

                  for(int i=0; i<6; i++) {
                      joint_traj[k][i] = temp_q[i];
                      ref_q[i] = temp_q[i];
                  }
              }
              ik_debug_enable = true;
              traj_current_index = 1;
              run_trajectory = true;
              return false;
          }
      }
      // ==============================================================
        // LỆNH S: VẼ ĐƯỜNG HÌNH SIN (Tự động chèn lệnh I để đi tới điểm xuất phát)
        // Cú pháp: S Xs Ys Xe Ye Z Amp Cycles Roll Pitch Yaw [config]
        // ==============================================================
        if (mode == 'S') {
            float xs, ys, xe, ye, z, amp, cycles, roll, pitch, yaw;
            int config_flag = 0;

            int parsed = sscanf(input, "%f %f %f %f %f %f %f %f %f %f %d",
                                &xs, &ys, &xe, &ye, &z, &amp, &cycles, &roll, &pitch, &yaw, &config_flag);

            if (parsed >= 10) {
                char cmd_i[100];
                char cmd_sine[120];

                // 1. Tạo lệnh I để chạy an toàn tới điểm xuất phát (Xs, Ys)
                sprintf(cmd_i, "I %.2f %.2f %.2f %.2f %.2f %.2f %d",
                        xs, ys, z, roll, pitch, yaw, config_flag);

                // 2. Tạo lệnh ~ (Lệnh Sin nội bộ) để thực thi ngay sau đó
                sprintf(cmd_sine, "~ %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d",
                        xs, ys, xe, ye, z, amp, cycles, roll, pitch, yaw, config_flag);

                // Nhét 2 lệnh này vào hàng đợi an toàn
                __disable_irq();
                if (q_count <= MAX_QUEUE_SIZE - 2) {
                    strcpy(cmd_queue[q_head], cmd_i);
                    q_head = (q_head + 1) % MAX_QUEUE_SIZE;
                    q_count++;

                    strcpy(cmd_queue[q_head], cmd_sine);
                    q_head = (q_head + 1) % MAX_QUEUE_SIZE;
                    q_count++;
                } else {
                    printf("Loi: Hang doi day, khong the nap lenh Sine!\n");
                }
                __enable_irq();

                return false;
            }
        }

        // ==============================================================
        // LỆNH ~: LỆNH NỘI BỘ THỰC THI QUỸ ĐẠO HÌNH SIN
        // ==============================================================
        if (mode == '~') {
            float xs, ys, xe, ye, z, amp, cycles, roll, pitch, yaw;
            int config_flag = 0;

            int parsed = sscanf(input, "%f %f %f %f %f %f %f %f %f %f %d",
                                &xs, &ys, &xe, &ye, &z, &amp, &cycles, &roll, &pitch, &yaw, &config_flag);

            if (parsed >= 10) {
                // Toán học tính toán trục xương sống (Baseline)
                float dx = xe - xs;
                float dy = ye - ys;
                float dist = sqrtf(dx * dx + dy * dy);
                float angle = atan2f(dy, dx);

                // Profile thời gian
                traj_total_time = dist / (TRAJ_SPEED_MM_S * 0.8f);
                if (traj_total_time < 0.5f) traj_total_time = 0.5f;

                traj_points_count = (int)(traj_total_time / TRAJ_DT);
                if (traj_points_count >= MAX_TRAJ_POINTS) traj_points_count = MAX_TRAJ_POINTS - 1;

                float ref_q[6];
                for(int i=0; i<6; i++) ref_q[i] = joint[i].current_angle;
                ik_debug_enable = false;
                // Nội suy sinh điểm Cartesian -> Joint
                for (int k = 0; k <= traj_points_count; k++) {
                    float ratio = (float)k / traj_points_count;

                    // DÙNG CÔNG THỨC SMOOTHSTEP (S-CURVE BẬC 3) NHƯ YÊU CẦU
                    float s = ratio * ratio * (3.0f - 2.0f * ratio);

                    // u là tọa độ đi dọc theo xương sống, v là dao động hình sin
                    float u = s * dist;
                    float v = amp * sinf(s * 2.0f * PI * cycles);

                    // Xoay ma trận đưa (u, v) về hệ tọa độ thực (X, Y)
                    float x = xs + u * cosf(angle) - v * sinf(angle);
                    float y = ys + u * sinf(angle) + v * cosf(angle);

                    float temp_q[6];
                    for(int i=0; i<6; i++) temp_q[i] = ref_q[i];

                    // Giải Inverse Kinematics
                    if (!solveIK6(x, y, z, roll, pitch, yaw,
                                  &temp_q[0], &temp_q[1], &temp_q[2],
                                  &temp_q[3], &temp_q[4], &temp_q[5], config_flag)) {
                        printf("Loi IK tai diem Sine thu %d\n", k);
                        return false;
                    }

                    for(int i=0; i<6; i++) {
                        joint_traj[k][i] = temp_q[i];
                        ref_q[i] = temp_q[i];
                    }
                }
                ik_debug_enable = true;
                traj_current_index = 1;
                run_trajectory = true;
                return false;
            }
        }
  return false;
}
uint8_t aa = 0;
	        uint8_t ab = 0;
	        uint8_t ac = 0;
	        uint8_t ad = 0;
	        uint8_t ae =0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  for (int i = 0; i < NUM_JOINT; i++) {
    joint[i].step_per_deg = (MOTOR_STEP_PER_REV * MICROSTEP * joint[i].gear_ratio) / 360.0f;
  }

  // Biến đếm thời gian để gửi dữ liệu định kỳ lên ROS 2
  uint32_t last_report_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  aa = HAL_GPIO_ReadPin(joint[0].limit_port, joint[0].limit_pin);
	        ab = HAL_GPIO_ReadPin(joint[1].limit_port, joint[1].limit_pin);
	        ac = HAL_GPIO_ReadPin(joint[2].limit_port, joint[2].limit_pin);
	        ad = HAL_GPIO_ReadPin(joint[3].limit_port, joint[3].limit_pin);
	        ae = HAL_GPIO_ReadPin(joint[4].limit_port, joint[4].limit_pin);
      if (run_trajectory) {
          if (!moving) {
              if (traj_current_index < traj_points_count) {
                  for (int i = 0; i < 6; i++) {
                      start_angle[i] = joint[i].current_angle;
                      target_angle[i] = joint_traj[traj_current_index][i];
                  }
                  plan_trajectory_step();
                  t_start_us = micros();
                  moving = true;
                  traj_current_index++;
              } else {
                  run_trajectory = false;
              }
          }
      } else {
          if (!moving && read_serial_command()) {
              for (int i = 0; i < NUM_JOINT; i++) {
                  start_angle[i] = joint[i].current_angle;
              }
              plan_sync_motion();
              t_start_us = micros();
              moving = true;
          }
      }

      if (moving) update_motion();


      // ========================================================
      // GỬI GÓC LÊN ROS 2 MỖI 20ms (50Hz) -> RViz mượt hơn
      // ========================================================
      // ========================================================
            // GỬI GÓC LÊN ROS 2 BẰNG NGẮT (KHÔNG BLOCKING)
            // ========================================================
           /* if (HAL_GetTick() - last_report_time >= 40) {
                // LƯU Ý: Biến tx_buf BẮT BUỘC phải là static để vùng nhớ
                // không bị xóa đi khi hàm UART đang truyền ngầm ở background
                static char tx_buf[100];

                sprintf(tx_buf, "S,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                       joint[0].current_angle,
                       joint[1].current_angle,
                       joint[2].current_angle,
                       joint[3].current_angle,
                       joint[4].current_angle,
                       joint[5].current_angle);

                // Lệnh này mất chưa tới 1 micro-giây để chạy, sau đó UART phần cứng
                // sẽ tự động nhặt từng chữ trong tx_buf gửi đi mà không làm treo chip
                HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buf, strlen(tx_buf));

                last_report_time = HAL_GetTick();
            }
            */
            // ========================================================
      // ========================================================

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period =199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Định tuyến lại hàm in ký tự của thư viện C (stdio.h)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  // Đẩy từng ký tự (ch) ra cổng UART2
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


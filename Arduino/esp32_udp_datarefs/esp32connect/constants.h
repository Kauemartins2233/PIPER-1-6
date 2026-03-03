#define DEG2RAD 0.017453293
const float Kp_vel = 0.00644f;
const float Ki_vel = 0.0548f;
const float Kd_vel = 0.0;
const float vel_max_out = 1.0;
const float vel_min_out = 0.0;

const float Kp_alt = 0.222f;
const float Ki_alt = 0.00146f;
const float Kd_alt = 0;
const float alt_max_out = 20 * DEG2RAD;
const float alt_min_out = -20 * DEG2RAD;

const float Kp_theta = 0.63187;
const float Ki_theta = 0.009672;
const float Kd_theta = 0;
const float Kv_theta = 0.1;
const float theta_max_out = 0.5;
const float theta_min_out = -0.5;

const float Kp_roll = 0.63187;
const float Ki_roll = 0.009672;
const float Kd_roll = 0.0;
const float Kv_roll = 0.0;
const float roll_max_out = 0.5;
const float roll_min_out = -0.5;

const float Kp_yaw = 0.63187f;
const float Ki_yaw = 0.009672f;
const float Kd_yaw = 0.0;
const float Kv_yaw = 0.0;
const float yaw_max_out = 0.5;
const float yaw_min_out = -0.5;

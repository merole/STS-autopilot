#include <math.h>

void mulMat(float res[], float mat1[3][3], float mat2[3][1]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
                res[i] += mat1[i][j] * mat2[j][0];
        }
    }
}

void subMat(float res[][1], float mat1[3][1], float mat2[3][1]) {
    for (int i = 0; i < 3; i++) {
        res[i][0] = mat1[i][0] - mat2[i][0];
    }
}

void crossProduct(float c_P[], float v_A[], float v_B[]) {
   c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
   c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
   c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
}

float vecMag(float vec[]) {
  float inner;
  for (int i = 0; i < sizeof(vec); i++) {
    inner += vec[i] * vec[i];
  }
  return sqrt(inner);
}

void normalize(float vec[]) {
  float mag = vecMag(vec);
  for (int i = 0;i < sizeof(vec);i++) {
    vec[i] = vec[i]/mag;
  }
}

float dotProduct(float v_a[], float v_b[]) {
  return v_a[0] * v_b[0] + v_a[1] * v_b[1] + v_a[2] * v_b[2];
}

void hamilton(float res[], float vec_1[], float vec_2[]) {
  res[0] = vec_1[0] * vec_2[0] - vec_1[1] * vec_2[1] - vec_1[2] * vec_2[2] - vec_1[3] * vec_2[3];
  res[1] = vec_1[0] * vec_2[1] + vec_1[1] * vec_2[0] + vec_1[2] * vec_2[3] - vec_1[3] * vec_2[2];
  res[2] = vec_1[0] * vec_2[2] - vec_1[1] * vec_2[3] + vec_1[2] * vec_2[0] + vec_1[3] * vec_2[1];
  res[3] = vec_1[0] * vec_2[3] + vec_1[1] * vec_2[2] - vec_1[2] * vec_2[1] + vec_1[3] * vec_2[0];
}

void toDirVec(float dir[], float q[]) {
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    float roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]));
    double cosp = sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]));
    float pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    float yaw = atan2(siny_cosp, cosy_cosp);

    dir[0] = cos(yaw)*cos(pitch);
    dir[1] = sin(yaw)*cos(pitch);
    dir[2] = sin(pitch);
}

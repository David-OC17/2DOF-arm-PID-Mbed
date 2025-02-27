#include <Arduino.h>
#include <math.h>

class KF_2DOF {
private:
  float dt;      // Time step
  float L1, L2;  // Link lengths
  float Q_noise; // Changing covariance Q matrix
  float R_noise; // Changing covariance R matrix

  // State vector [x_e, y_e, vx_e, vy_e]
  float x[4] = {0, 0, 0, 0};

  // State covariance matrix
  float P[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  // State transition matrix A
  float A[4][4];

  // Process noise covariance Q
  float Q[4][4];

  // Measurement noise covariance R
  float R[2][2];

public:
  KF_2DOF(float dt, float L1, float L2, float Q_noise, float R_noise) {
    this->dt = dt;
    this->L1 = L1;
    this->L2 = L2;
    this->Q_noise = Q_noise;
    this->R_noise = R_noise;

    // Init A matrix
    A[0][0] = 1;
    A[0][1] = 0;
    A[0][2] = dt;
    A[0][3] = 0;
    A[1][0] = 0;
    A[1][1] = 1;
    A[1][2] = 0;
    A[1][3] = dt;
    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = 1;
    A[2][3] = 0;
    A[3][0] = 0;
    A[3][1] = 0;
    A[3][2] = 0;
    A[3][3] = 1;

    // Init covariance R,Q matrices with new noise
    Q[0][0] = Q_noise; Q[0][1] = 0; Q[0][2] = 0; Q[0][3] = 0;
    Q[1][0] = 0; Q[1][1] = Q_noise; Q[1][2] = 0; Q[1][3] = 0;
    Q[2][0] = 0; Q[2][1] = 0; Q[2][2] = Q_noise; Q[2][3] = 0;
    Q[3][0] = 0; Q[3][1] = 0; Q[3][2] = 0; Q[3][3] = Q_noise;

    R[0][0] = 0.01; R[0][1] = 0;
    R[1][0] = 0; R[1][1] = 0.01;
  }

  void predict() {
    // Predict state: x = A * x
    float x_new[4];
    x_new[0] = A[0][0] * x[0] + A[0][2] * x[2];
    x_new[1] = A[1][1] * x[1] + A[1][3] * x[3];
    x_new[2] = x[2];
    x_new[3] = x[3];

    x[0] = x_new[0]; x[1] = x_new[1];
    x[2] = x_new[2]; x[3] = x_new[3];

    // Predict covariance: P = A * P * A^T + Q
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        P[i][j] += Q[i][j];
      }
    }
  }

  void update(float theta1, float theta2) {
    // Compute measurement prediction
    float hx[2];
    hx[0] = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    hx[1] = L1 * sin(theta1) + L2 * sin(theta1 + theta2);

    // Compute measurement residual
    float y[2] = {hx[0] - x[0], hx[1] - x[1]};

    // Compute Kalman Gain K = P * H^T * (H * P * H^T + R)^-1
    float S[2][2] = {{P[0][0] + R[0][0], P[0][1]},
                     {P[1][0], P[1][1] + R[1][1]}};
    float K[4][2] = {{P[0][0] / S[0][0], P[0][1] / S[1][1]},
                     {P[1][0] / S[0][0], P[1][1] / S[1][1]},
                     {P[2][0] / S[0][0], P[2][1] / S[1][1]},
                     {P[3][0] / S[0][0], P[3][1] / S[1][1]}};

    // Update state: x = x + K * y
    x[0] += K[0][0] * y[0] + K[0][1] * y[1];
    x[1] += K[1][0] * y[0] + K[1][1] * y[1];
    x[2] += K[2][0] * y[0] + K[2][1] * y[1];
    x[3] += K[3][0] * y[0] + K[3][1] * y[1];

    // Update covariance: P = (I - K * H) * P
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        P[i][j] -= K[i][0] * P[0][j] + K[i][1] * P[1][j];
      }
    }
  }

  float getX() { return x[0]; }
  float getY() { return x[1]; }

  float getVX() { return x[2]; }
  float getVY() { return x[3]; }
};

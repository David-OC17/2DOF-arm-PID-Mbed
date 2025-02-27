#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

// CURRENLTY NOT UNDER USE, IGNORE THIS FILE

#include <Arduino.h>

// #define VELOCITY_OBSERVABLE

/**
 * Provides a way to approximate state x[0] and x[1] based on either only observable state
 * x[0], or both states being observable.
 */
class KalmanFilter {
private:
  float dt; // Time step
  float process_noise;
  float measurement_noise;

  // State vector [position, velocity]
  float x[2] = {0, 0};

  // State covariance matrix
  float P[2][2] = {{1, 0}, {0, 1}};

  // State transition matrix A
  float A[2][2];

#ifdef VELOCITY_OBSERVABLE
  // Measurement matrix H (Identity matrix)
  const float H[2][2] = {{1, 0}, {0, 1}};

  // Measurement noise covariance R
  float R[2][2];
#else
  // Measurement matrix H (Only position is measured)
  const float H[1][2] = {{1, 0}};

  // Measurement noise covariance R (1x1 matrix)
  float R;
#endif

  // Process noise covariance Q
  float Q[2][2];

public:
  KalmanFilter(float dt, float process_noise, float measurement_noise) {
    this->dt = dt;
    this->process_noise = process_noise;
    this->measurement_noise = measurement_noise;

    // Init A matrix
    A[0][0] = 1; A[0][1] = dt;
    A[1][0] = 0; A[1][1] = 1;

    // Init process noise Q
    Q[0][0] = process_noise; Q[0][1] = 0;
    Q[1][0] = 0; Q[1][1] = process_noise;

#ifdef VELOCITY_OBSERVABLE
    // Init measurement noise R
    R[0][0] = measurement_noise;
    R[0][1] = 0;
    R[1][0] = 0;
    R[1][1] = measurement_noise;
#else
    // Initialize measurement noise R
    R = measurement_noise;
#endif
  }

  void predict() {
    // Predict state: x = A * x
    float x_new[2];
    x_new[0] = A[0][0] * x[0] + A[0][1] * x[1];
    x_new[1] = A[1][0] * x[0] + A[1][1] * x[1];

    x[0] = x_new[0];
    x[1] = x_new[1];

    // Predict covariance: P = A * P * A^T + Q
    float P_new[2][2];
    P_new[0][0] = A[0][0] * P[0][0] + A[0][1] * P[1][0];
    P_new[0][1] = A[0][0] * P[0][1] + A[0][1] * P[1][1];
    P_new[1][0] = A[1][0] * P[0][0] + A[1][1] * P[1][0];
    P_new[1][1] = A[1][0] * P[0][1] + A[1][1] * P[1][1];

    // Add process noise
    P[0][0] = P_new[0][0] + Q[0][0];
    P[0][1] = P_new[0][1] + Q[0][1];
    P[1][0] = P_new[1][0] + Q[1][0];
    P[1][1] = P_new[1][1] + Q[1][1];
  }

  // If only position is known, pass it in z[0]
  void update(float z[2]) {
#ifdef VELOCITY_OBSERVABLE
    // Compute Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
    float S[2][2] = {{P[0][0] + R[0][0], P[0][1] + R[0][1]},
                     {P[1][0] + R[1][0], P[1][1] + R[1][1]}};

    // Compute inverse of S (2x2 matrix inversion)
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    float S_inv[2][2] = {{S[1][1] / det, -S[0][1] / det},
                         {-S[1][0] / det, S[0][0] / det}};

    // Compute Kalman Gain: K = P * S^-1
    float K[2][2];
    K[0][0] = P[0][0] * S_inv[0][0] + P[0][1] * S_inv[1][0];
    K[0][1] = P[0][0] * S_inv[0][1] + P[0][1] * S_inv[1][1];
    K[1][0] = P[1][0] * S_inv[0][0] + P[1][1] * S_inv[1][0];
    K[1][1] = P[1][0] * S_inv[0][1] + P[1][1] * S_inv[1][1];

    // Update state: x = x + K * (z - H * x)
    float y[2] = {z[0] - x[0], z[1] - x[1]};
    x[0] += K[0][0] * y[0] + K[0][1] * y[1];
    x[1] += K[1][0] * y[0] + K[1][1] * y[1];

    // Update covariance: P = (I - K * H) * P
    float I_KH[2][2] = {{1 - K[0][0], -K[0][1]}, {-K[1][0], 1 - K[1][1]}};

    float P_new[2][2];
    P_new[0][0] = I_KH[0][0] * P[0][0] + I_KH[0][1] * P[1][0];
    P_new[0][1] = I_KH[0][0] * P[0][1] + I_KH[0][1] * P[1][1];
    P_new[1][0] = I_KH[1][0] * P[0][0] + I_KH[1][1] * P[1][0];
    P_new[1][1] = I_KH[1][0] * P[0][1] + I_KH[1][1] * P[1][1];
#else
    // Compute Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
    float S = P[0][0] + R;                   // Innovation covariance
    float K[2] = {P[0][0] / S, P[1][0] / S}; // Kalman Gain (2x1 vector)

    // Update state: x = x + K * (z - H * x)
    float y = z[0] - x[0]; // Measurement residual
    x[0] += K[0] * y;
    x[1] += K[1] * y;

    // Update covariance: P = (I - K * H) * P
    float P_new[2][2];
    P_new[0][0] = (1 - K[0]) * P[0][0];
    P_new[0][1] = (1 - K[0]) * P[0][1];
    P_new[1][0] = -K[1] * P[0][0] + P[1][0];
    P_new[1][1] = -K[1] * P[0][1] + P[1][1];
#endif

    // KEEEEP
    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0];
    P[1][1] = P_new[1][1];
  }

  float getPosition() { return x[0]; }

  float getVelocity() { return x[1]; }
};

#endif // __KALMAN_FILTER_H__
#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(float q, float r, float p, float initial_value)
    : Q(q), R(r), P(p), X(initial_value) {}

int KalmanFilter::update(int measurement) {
    // Prediction update
    P = P + Q;

    // Measurement update
    float K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;

    return static_cast<int>(X);
}
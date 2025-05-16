#pragma once

class KalmanFilter {
public:
    KalmanFilter(float q = 0.01, float r = 1.0, float p = 1.0, float initial_value = 0.0);
    int update(int measurement);

private:
    float Q, R, P, X;
};
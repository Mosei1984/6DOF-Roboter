#pragma once
#include <array>
#include <cmath>

// -----------------------------------------------------------------------------
// DHUtilities.h - Common utilities for kinematics
// -----------------------------------------------------------------------------

// Struktur für DH-Parameter eines Gelenks: a, alpha, d, theta (in Radiant)
struct DHParams {
    double a;      // Link-Länge in Metern
    double alpha;  // Twist-Winkel in Radiant
    double d;      // Offset entlang der z-Achse in Metern
    double theta;  // Gelenkwinkel in Radiant
};

// Struktur für Gelenk-Limits (Min und Max in Radiant)
struct JointLimits {
    double minAngle;  // in Radiant
    double maxAngle;  // in Radiant
};

// All functions here need to be INLINE to avoid multiple definition errors
inline std::array<std::array<double,4>,4> dhToMatrix(const DHParams& p) {
    double ca = cos(p.alpha);
    double sa = sin(p.alpha);
    double ct = cos(p.theta);
    double st = sin(p.theta);

    return {{
        {{ ct,        -st * ca,    st * sa,    p.a * ct }},
        {{ st,         ct * ca,   -ct * sa,    p.a * st }},
        {{ 0.0,            sa,         ca,         p.d  }},
        {{ 0.0,           0.0,        0.0,        1.0  }}
    }};
}

inline std::array<std::array<double,4>,4> matMul4x4(
    const std::array<std::array<double,4>,4>& A,
    const std::array<std::array<double,4>,4>& B)
{
    std::array<std::array<double,4>,4> C{};
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            C[i][j] = 0.0;
            for(int k = 0; k < 4; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

inline void rotationMatrixToEulerZYX(
    const std::array<std::array<double,4>,4>& T,
    double& roll, double& pitch, double& yaw)
{
    // Extract rotation matrix elements
    double r11 = T[0][0], r12 = T[0][1]; 
    double r21 = T[1][0], r22 = T[1][1];
    double r31 = T[2][0], r32 = T[2][1], r33 = T[2][2];

    pitch = atan2(-r31, sqrt(r11*r11 + r21*r21));

    if (fabs(pitch - M_PI/2) < 1e-6) {
        roll = 0.0;
        yaw  = atan2(r12, r22);
    } else if (fabs(pitch + M_PI/2) < 1e-6) {
        roll = 0.0;
        yaw  = atan2(-r12, -r22);
    } else {
        roll = atan2(r32, r33);
        yaw  = atan2(r21, r11);
    }
}

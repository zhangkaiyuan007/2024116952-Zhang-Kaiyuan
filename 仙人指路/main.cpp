//
// 2025 Helios CV enter examination
//
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */

#include <iostream>
#include <chrono>
#include <cmath>
#include "./chassis.h"

#include "kalman/kalman.h"
#include "kalman/AdaptiveEKF.h"

int main()
{
    const int mode = 2;
    SET_MODE(mode);
    ENABLE_LOG(true);

    // Set kalman fliter
    const int Z_N = 1, X_N = 2;
    Eigen::Matrix<double, X_N, 1> X;
    Eigen::Matrix<double, X_N, X_N> A;
    Eigen::Matrix<double, X_N, X_N> P;
    Eigen::Matirx<double, X_N, X_N> R;
    Eigen::Matrix<double, X_N, Z_N> K;
    Eigen::Matrix<double, Z_N, X_N> C;
    Eigen::Matrix<double, Z_N, Z_N> Q;

    X << 0, 0;
    A << 1, 1, 0, 1;
    C << 1, 0;
    R << 2, 0, 0, 2;
    Q << 10;

    while (true)
    {
        float sensor_data;
        float filtered_data;
        float current_position;

        // receive sensor data -> how far to the target (10ms delay)
        GET_SENSOR_DISTANCE(sensor_data);

        // receive current position (current_position + sensor_data ~= target position)
        GET_CURRENT_POSITION(current_position);

        // =============================================
        // TODO: your code here
        // you should try to follow the target position

        switch (mode)
        {
        case 0:
        case 1:
        case 2:
            {
                // you should replace the following line
                Eigen::Matrix<double, X_N, 1> X_k = A * X;
                P = A * P * A.transpose() + R;
                K = P * C.transpose() * (C * P * C.transpose() + Q).inverse();
                Eigen::Matrix<double, Z_N, 1> Z(sensor_data);
                X = X_k + K * (Z - C * X_k);
                P = (Eigen::Matrix<double, X_N, X_N>::Identity() - K * C) * P;
                filtered_data = X[0];    // set filtered data as raw sensor data (without filter)
                                                // this may cause problem
            }
            break;
        };

        // =============================================
        SET_TARGET_POSITION(current_position + filtered_data);      // go to the position which you consider as the target position
        SHOW_DEBUG_IMAGE();         // show image debug info
    }

    return 0;
}

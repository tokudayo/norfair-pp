#pragma once
#include "common.h"

inline static Matrix<FLOAT_T, 4, 4> CreateTransitionMatrix(FLOAT_T dt) {
    Matrix<FLOAT_T, 4, 4> transition_matrix;
    transition_matrix << 1, 0, dt, 0,
                         0, 1, 0, dt,
                         0, 0, 1, 0,
                         0, 0, 0, 1;
    return transition_matrix;
}

inline static Matrix<FLOAT_T, 2, 4> CreateMeasurementMatrix() {
    Matrix<FLOAT_T, 2, 4> measurement_matrix;
    measurement_matrix = Matrix<FLOAT_T, 2, 4>();
    measurement_matrix << 1, 0, 0, 0,
                          0, 1, 0, 0;
    return measurement_matrix;
}

inline static Matrix<FLOAT_T, 4, 4> CreateProcessUncertaintyMatrix(FLOAT_T q) {
    Matrix<FLOAT_T, 4, 4> process_uncertainty_matrix;
    process_uncertainty_matrix << 1, 0, 0, 0,
                                  0, 1, 0, 0,
                                  0, 0, q, 0,
                                  0, 0, 0, q;
    return process_uncertainty_matrix;
}

inline static Matrix<FLOAT_T, 2, 2> CreateMeasurementUncertaintyMatrix(FLOAT_T r) {
    Matrix<FLOAT_T, 2, 2> measurement_uncertainty_matrix;
    measurement_uncertainty_matrix << r, 0,
                                      0, r;
    return measurement_uncertainty_matrix;
}


const FLOAT_T _R = 4.0, _Q = 0.1, _P = 10.0, dt = 1;
static const Matrix<FLOAT_T, 4, 4> F = CreateTransitionMatrix(dt);
static const Matrix<FLOAT_T, 2, 4> H = CreateMeasurementMatrix();
static const Matrix<FLOAT_T, 4, 4> Q = CreateProcessUncertaintyMatrix(_Q);
static const Matrix<FLOAT_T, 2, 2> R = CreateMeasurementUncertaintyMatrix(_R);
static const Matrix<FLOAT_T, 4, 4> I = Matrix<FLOAT_T, 4, 4>::Identity();


class KalmanFilter {
public:
    Matrix<FLOAT_T, 4, 1> x;
    Matrix<FLOAT_T, 4, 4> P;
    Matrix<FLOAT_T, 4, 2> K;
    Matrix<FLOAT_T, 2, 1> y;
    Matrix<FLOAT_T, 2, 2> S;
    // Matrix<FLOAT_T, 4, 4> &F = F, &Q = Q, &I = I;
    // Matrix<FLOAT_T, 2, 2> &R = R;
    // Matrix<FLOAT_T, 2, 4> &H = H;

    KalmanFilter() {};

    KalmanFilter(Point initial_detection):
        x(Matrix<FLOAT_T, 4, 1>::Zero()), P(Matrix<FLOAT_T, 4, 4>::Identity()),
        K(Matrix<FLOAT_T, 4, 2>::Zero()), y(Matrix<FLOAT_T, 2, 1>::Zero()),
        S(Matrix<FLOAT_T, 2, 2>::Zero())
    {
        this->P(2, 2) = _P;
        this->P(3, 3) = _P;
        this->x(0, 0) = initial_detection(0, 0);
        this->x(1, 0) = initial_detection(0, 1);
        // Will this works?
        // this->x(seq(0, 1), all) = initial_detection.transpose();
    };

    void Predict() {
        x = F * x;
        P = (F * P) * F.transpose() + Q;
    }

    void Update(const Point& z) {
        y = z.transpose() - H * x;
        auto PHT = P * H.transpose();
        S = H * PHT + R;
        K = PHT * S.inverse();
        x += K * y;
        P = (I - K * H) * P;
    }
};
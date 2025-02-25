template <typename T>
class KalmanFilter {
private:
    T _err_measure;
    T _err_estimate;
    T _q;
    T _kalman_gain;
    T _last_estimate;
    T _current_estimate;

public:
    SimpleKalmanFilter(T mea_e, T est_e, T q) {
        _err_measure = mea_e;
        _err_estimate = est_e;
        _q = q;
        _last_estimate = 0.0; // Initialize _last_estimate to a default value
    }

    T updateEstimate(T mea) {
        _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
        _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
        _err_estimate = (1.0 - _kalman_gain) * _err_estimate + std::abs(_last_estimate - _current_estimate) * _q;
        _last_estimate = _current_estimate;

        return _current_estimate;
    }

    void setMeasurementError(T mea_e) {
        _err_measure = mea_e;
    }

    void setEstimateError(T est_e) {
        _err_estimate = est_e;
    }

    void setProcessNoise(T q) {
        _q = q;
    }

    T getKalmanGain() {
        return _kalman_gain;
    }
};

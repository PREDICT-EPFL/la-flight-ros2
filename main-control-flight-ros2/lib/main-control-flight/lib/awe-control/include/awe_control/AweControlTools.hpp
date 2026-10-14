//
// Created by johannes on 07.04.20.
//

#ifndef SRC_AWECONTROLTOOLS_H
#define SRC_AWECONTROLTOOLS_H

#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace awe_control2 {

/* Eigen types */
using Vector2 = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
using Vector3 = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matrix3 = Eigen::Matrix<double, 3, 3, Eigen::DontAlign>;
using Quaternion = Eigen::Quaternion<double, Eigen::DontAlign>;

#define M_ONE_G 9.80665

/* Math */
static double constrain(const double &valueToConstrain, const double &lowerLimit, const double &upperLimit) {
    return fmin(fmax(valueToConstrain, lowerLimit), upperLimit);
}

static double degrees(const double &radians) { return radians * 180.0 / M_PI; }
static double radians(const double &degrees) { return degrees * M_PI / 180.0; }

/* Transformations */
static Quaternion T1quat(const double &rotAng) { return {cos(-rotAng / 2), sin(-rotAng / 2), 0, 0}; }
static Quaternion T2quat(const double &rotAng) { return {cos(-rotAng / 2), 0, sin(-rotAng / 2), 0}; }
static Quaternion T3quat(const double &rotAng) { return {cos(-rotAng / 2), 0, 0, sin(-rotAng / 2)}; }

/* Navigation */
static void getGammaChi(const Vector3 &g_vg, double &gamma, double &chi) {
    gamma = atan2(-g_vg(2), g_vg.segment<2>(0).norm());
    chi = atan2(g_vg(1), g_vg(0));
}
static void getGammaChi(const Vector3 &g_vg, double &gamma, double &chi, double &Vg) {
    getGammaChi(g_vg, gamma, chi);
    Vg = g_vg.norm();
}

static double getBearingFromCenter(const double &positionX, const double &positionY) {

    /* QTE: True bearing from the station (here: origin of the coordinate frame) */
    double bearingFromCenter = atan2(positionY, positionX);
    if (bearingFromCenter < 0.0) { bearingFromCenter += 2.0 * M_PI; }

    return bearingFromCenter;
}
static double getBearingFromCenter(const Vector2 &position) {
    return getBearingFromCenter(position(0), position(1));
}

static double getShortestAngleErr(const double &angleRef, const double &angle) {
    double angleErr = angleRef - angle;

    if (angleErr > M_PI) { angleErr -= 2.0 * M_PI; }
    else if (angleErr < -M_PI) { angleErr += 2.0 * M_PI; }

    return angleErr;
}

template<typename T, const T lower_bound, const T upper_bound, bool debug = false>
static T wrapTo(T value) {
    if (debug) std::cout << "[wrapTo] value: " << value << "\n";
    if (value > upper_bound) {
        value -= (upper_bound - lower_bound);
        if (debug) std::cout << "[wrapTo] -=: " << value << "\n";
    } else if (value < lower_bound) {
        value += (upper_bound - lower_bound);
        if (debug) std::cout << "[wrapTo] +=: " << value << "\n";
    }
    return value;
}

static double wrapTo(double value, double lower_bound, double upper_bound)
{
    if (value > upper_bound)
        value -= (upper_bound - lower_bound);
    else if (value < lower_bound)
        value += (upper_bound - lower_bound);
    return value;
}

/* Classes */
class Ramp {
public:
    Ramp(const double &sampletime_) {
        setSampletime(sampletime_);
    }
    Ramp(const double &startValue, const double &endValue, const double &transitionTime) {
        set_start_value(startValue);
        set_end_value(endValue);
        m_transition_duration = transitionTime;
    }
    Ramp(const double &sampletime_, const double &startValue, const double &endValue, const double &transitionTime) :
            Ramp(startValue, endValue, transitionTime) {
        setSampletime(sampletime_);
    }
    Ramp() = default;

    void setSampletime(const double &sampletime_) {
        if (sampletime_ > 0.0) m_dt = sampletime_;
        else std::cout << "[Integral] Sampletime must be > 0. Nothing has been written.\n";
    }
    void set_start_value(const double &startValue) {
        m_startValue = startValue;
        update_slope();
        reset();
    }
    void set_end_value(const double &endValue) {
        m_endValue = endValue;
        update_slope();
        reset();
    }
    void set_transition_time(const double &transitionTime) {
        m_transition_duration = transitionTime;
        update_slope();
        reset();
    }
    void reset() { set_value(m_startValue); }

    double get_value() { return m_value; }
    void set_value(const double &value) {
        if (m_slope > 0.0) {
            m_value = std::min(value, m_endValue);
        } else if (m_slope < 0.0) {
            m_value = std::max(value, m_endValue);
        } else {
            m_value = m_endValue;
        }
    }

    double integrate() {
        /* If dt > 0.5 s, something is wrong. Do not integrate (use dt = 0) */
        double dt = (m_dt > 0.5) ? 0.0 : m_dt;
        set_value(m_value + m_slope * dt);
              return get_value();
    }
    double integrate(const double &dt) {
        m_dt = dt;
        return integrate();
    }

private:
    double m_startValue{0};
    double m_endValue{1};
    double m_transition_duration{1};
    double m_slope{(m_endValue - m_startValue) / m_transition_duration};

    double m_value{m_startValue};
    double m_dt{0.01};

    void update_slope() { m_slope = (m_endValue - m_startValue) / m_transition_duration; }
};

class Integral {
public:
    Integral(const double &sampletime_) {
        setSampletime(sampletime_);
    }
    Integral(const double &lowerLimit, const double &upperLimit) {
        set_upper_saturation_limit(upperLimit);
        set_lower_saturation_limit(lowerLimit);
    }
    Integral(const double &sampletime_, const double &lowerLimit, const double &upperLimit) {
        setSampletime(sampletime_);
        set_upper_saturation_limit(upperLimit);
        set_lower_saturation_limit(lowerLimit);
    }
    Integral() = default;

    void setSampletime(const double &sampletime_) {
        if (sampletime_ > 0.0) m_dt = sampletime_;
        else std::cout << "[Integral] Sampletime must be > 0. Nothing has been written.\n";
    }
    void set_upper_saturation_limit(const double &upper_limit) { m_integral_upper_limit = upper_limit; }
    void set_lower_saturation_limit(const double &lower_limit) { m_integral_lower_limit = lower_limit; }
    void reset() { set_value(0.0); }

    double get_value() const { return m_integral; }
    void set_value(const double &value) {
        m_integral = value;
        limitIntegral();
    }

    double integrate(const double &value) {
        /* If dt > 0.5 s, something is wrong. Do not integrate (use dt = 0) */
        double dt = (m_dt > 0.5) ? 0.0 : m_dt;
        set_value(m_integral + value * dt);
        return get_value();
    }
    double integrate(const double &value, const double &dt) {
        m_dt = dt;
        return integrate(value);
    }

    double operator()() const { return get_value(); }

private:
    double m_integral_upper_limit{999999999999999};
    double m_integral_lower_limit{-999999999999999};

    double m_dt{0.01};
    double m_integral{0.0};

    void limitIntegral() {
        if (m_integral > m_integral_upper_limit) m_integral = m_integral_upper_limit;
        else if (m_integral < m_integral_lower_limit) m_integral = m_integral_lower_limit;
    }
};

template<typename T = double, unsigned delay_length = 1>
class Delay {
public:
    Delay() = default;

    void clear() {
        index_read = 0;
        index_write = 0;
        buffer = {};
        is_initialized = false;
    }

    T putThrough(const T &signal) {

        T returnValue;

        if (is_initialized) {
            /* Read oldest value and count up */
            returnValue = buffer[index_read];

            if (index_read == delay_length - 1) {
                index_read = 0;
            } else {
                ++index_read;
            }
        } else {
            if (index_write > 0)
                returnValue = buffer[0];
            else
                returnValue = signal;
        }

        /* Write present value and count up */
        buffer[index_write] = signal;

        if (index_write == delay_length - 1) {
            index_write = 0;
            is_initialized = true;
        } else {
            ++index_write;
        }

        return returnValue;
    }

private:
    T buffer[delay_length]{};
    int index_read{0};
    int index_write{0};
    bool is_initialized{false};
};

template<uint poly_order = 4, unsigned points_per_coef = 4, unsigned bufLen = (poly_order + 1) * points_per_coef>
class PolyFilter {
public:
    PolyFilter() { reset(); };

    void reset() {
        timeBuf.setZero();
        valueBuf.setZero();
        coeffs.setZero();
        idx_write = 0;
        is_initialized = false;
        filter_computed = false;
    }

    double getNewestTime() { return timeBuf(idx_write); }

    /* Write to the buffer */
    void write(const double &time, const double &newValue) {

        if (time <= getNewestTime()) return;

        if (!is_initialized && 0 == nextIdx(idx_write)) is_initialized = true;

        idx_write = nextIdx(idx_write);
        timeBuf(idx_write) = time;
        valueBuf(idx_write) = newValue;

        filter_computed = false;
    }
    /* Check if value has crossed the full circle direction and write to the buffer */
    void write2(const double &time, const double &newValue) {

        if (newValue - valueBuf(idx_write) > M_PI)
            valueBuf = (valueBuf.array() += 2.0 * M_PI).matrix();
        else if (newValue - valueBuf(idx_write) < -M_PI)
            valueBuf = (valueBuf.array() -= 2.0 * M_PI).matrix();

        write(time, newValue);
    }

    bool isInitialized() { return is_initialized; }

    double getUnfilteredValue() { return valueBuf(idx_write); }
    double filterValue() {
        if (!is_initialized) return getUnfilteredValue();
        if (!filter_computed) { filter(); }
        return coeffs(0);
    }
    double filterDerivative() {
        if (!is_initialized) return 0.0;
        if (!filter_computed) { filter(); }
        return coeffs(1);
    }

private:
    bool is_initialized{false};
    bool filter_computed{false};
    uint idx_write{0};
    Eigen::Matrix<double, bufLen, 1> timeBuf;
    Eigen::Matrix<double, bufLen, 1> valueBuf;
    Eigen::Matrix<double, poly_order + 1, 1> coeffs;

    uint nextIdx(const uint &idx) { return (idx < bufLen - 1) ? (idx + 1) : 0; }

    Eigen::Matrix<double, bufLen, 1> getTimes() {

        /* Sort times so that the most recent one is at the end */
        Eigen::Matrix<double, bufLen, 1> times;
        times << timeBuf.segment(idx_write + 1, bufLen - idx_write - 1), timeBuf.segment(0, idx_write + 1);

        /* Relate all times to the most recent time to get a relative time interval */
        return (times.array() - times(bufLen - 1)).matrix();
    }
    Eigen::Matrix<double, bufLen, 1> getValues() {

        Eigen::Matrix<double, bufLen, 1> values;
        values << valueBuf.segment(idx_write + 1, bufLen - idx_write - 1), valueBuf.segment(0, idx_write + 1);

        return values;
    }

    void filter() {
        /* Get the measurements vector z */
        Eigen::Matrix<double, bufLen, 1> z = getValues();       // z contains the values for the fit

        /* Build the design matrix H to express the equations
         *
         *      z0 = a0 + a1*t0^1 + a2*t0^2 + a3*t0^3 + ...
         *      z0 = a0 + a1*t1^1 + a2*t1^2 + a3*t1^3 + ...
         *      z0 = a0 + a1*t2^1 + a2*t2^2 + a3*t2^3 + ...
         *        ...
         *
         * in matrix formulation:
         *
         *      z = H * x
         *
         * with
         *
         *     (1 t0 t0^2 t0^3 ...)                              (a0)
         * H = (1 t1 t1^2 t1^3 ...) and the parameter vector x = (a1)
         *     (1 t2 t2^2 t2^3 ...)                              (a2)
         *     (.. .. .. .. .. ...)                              (..)
         * */
        Eigen::Matrix<double, bufLen, poly_order + 1> H;

        /* Order 0 */
        H.col(0) = Eigen::Matrix<double, bufLen, 1>::Constant(1);

        /* Order 1 ... N */
        Eigen::Matrix<double, bufLen, 1> t = getTimes();        // t contains the times for the fit

        for (unsigned i = 1; i < poly_order + 1; ++i) {
            H.col(i) = H.col(i - 1).cwiseProduct(t);
        }

        /* Solve for parameter vector x which contains the polynomial coefficients: x = (a0, a1, a2, ... , an) */
        coeffs = (H.transpose() * H).ldlt().solve(H.transpose() * z);
        filter_computed = true;
    }
};

class Averager {
public:
    void setInitValue(const double &initValue) { average = sum = initValue; };
    void log(const double &newValue) {
        sum += newValue;
        average = sum / ++nMeasurements;
    }
    double getAvg() { return average; };
    void reset() {
        nMeasurements = 0;
        average = sum = 0;
    }
private:
    unsigned nMeasurements{0};
    double sum{0};
    double average{0};
};

template<uint bufLen = 10>
class MovingAverageFilter {
public:
    MovingAverageFilter(const double &initValue) {
        valueBuf.setZero();
        write(initValue);
    }
    MovingAverageFilter() {
        valueBuf.setZero();
    }

    virtual void reset() {
        idx_write = 0;
        valueBuf.setZero();
        is_initialized = false;
    }
    virtual void init(const double &initValue) {
        reset();
        valueBuf(1) = initValue;
        idx_write = nextIdx(idx_write);
    };
    void write(const double &newValue) {

        if (!is_initialized && 0 == nextIdx(idx_write)) is_initialized = true;

        idx_write = nextIdx(idx_write);
        valueBuf(idx_write) = newValue;
    }
    void printBuffer() {
        std::cout << "Buflen: " << bufLen << "\n";
        std::cout << valueBuf << "\n-------------------------------\n";
    }
    double getMovingAverage() {
        if (is_initialized) {
            return valueBuf.mean();
        } else {
            /* Until buffer is full, compute average from existing entries */
            return valueBuf.sum() / std::max(1.0, static_cast<double>(idx_write));
            // /* Until buffer is full, return last written value */
            // return valueBuf(previousIdx(idx_write));
        }
    }

private:
    bool is_initialized{false};
    uint idx_write{0};
    Eigen::Matrix<double, bufLen, 1> valueBuf;

    uint nextIdx(const uint &idx) { return (idx < bufLen - 1) ? (idx + 1) : 0; }
    uint previousIdx(const uint &idx) { return (idx > 0) ? (idx - 1) : bufLen - 1; }
};

template<uint bufLen = 10>
class MovingAverageLogger : MovingAverageFilter<bufLen> {
public:
    MovingAverageLogger(std::string name, const double &initValue) :
            MovingAverageFilter<bufLen>(initValue), loggerName(std::move(name)) {};

    void reset() override {
        n_measurements = 0;
        value_sum = value_max = value_min = 0;
        MovingAverageFilter<bufLen>::reset();
    }
    void init(const double &initValue) override {
        reset();
        MovingAverageFilter<bufLen>::init(initValue);
        log(initValue);
    };

    void log(const double &newValue) {
        if (n_measurements == 0) { value_min = value_max = newValue; }
        value_max = std::max(value_max, newValue);
        value_min = std::min(value_min, newValue);
        value_sum += newValue;
        ++n_measurements;
        MovingAverageFilter<bufLen>::write(newValue);
    }
    double getTotalAvg() { return value_sum / n_measurements; }
    void printStatistics() {
        std::cout << "\n" << loggerName << " statistics [ms]: "
                  << "\nmax: " << std::setw(3) << value_max * 1e3
                  << "\navg: " << std::setw(3) << getTotalAvg() * 1e3
                  << "\nmin: " << std::setw(3) << value_min * 1e3
                  << "\n";
    };

private:
    std::string loggerName = "Value";
    unsigned n_measurements{0};
    double value_sum{0};
    double value_max{0};
    double value_min{0};
};

struct ValueLogger {
public:
    ValueLogger(std::string name) : loggerName(std::move(name)) {};
    ValueLogger() = default;

    void log(const double &newValue) {
        if (n_measurements == 0) { value_min = value_max = newValue; }
        value_max = std::max(value_max, newValue);
        value_min = std::min(value_min, newValue);
        value_sum += newValue;
        ++n_measurements;
    };
    double getAvg() { return value_sum / n_measurements; }
    void printStatistics() {
        std::cout << "\n" << loggerName << " statistics [ms]: "
                  << "\nmax: " << std::setw(3) << value_max * 1e3
                  << "\navg: " << std::setw(3) << getAvg() * 1e3
                  << "\nmin: " << std::setw(3) << value_min * 1e3
                  << "\n";
    };
    void reset() {
        n_measurements = 0;
        value_sum = value_max = value_min = 0;
    }

private:
    std::string loggerName = "Value";
    unsigned n_measurements{0};
    double value_sum{0};
    double value_max{0};
    double value_min{0};
};


}

#endif //SRC_AWECONTROLTOOLS_H

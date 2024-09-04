#include "PID.hpp"
#include "mbed.h"

PID::PID(float kp, float ki, float kd, float rate_suppression_gain, float sample_acceleration, float sample_time)
    : _kp(kp), _ki(ki), _kd(kd), _sample_time(sample_time), _rate_suppression_gain(rate_suppression_gain), _sample_acceleration(sample_acceleration), _last_input(0), _integral(0), _last_output(0), _last_error(0), _last_rate(0) {}

float PID::calculate(int set_speed, int now_speed)
{
    static float acceleration = 0;
    static Timer timer;
    static bool timer_started = false;

    if (!timer_started)
    {
        timer.start();
        timer_started = true;
    }

    float error = set_speed - now_speed;
    float p_term = _kp * error;
    _integral += error * _sample_time;
    float i_term = _ki * _integral;
    float d_term = _kd * (error - _last_error) / _sample_time;

    // 加速度を更新
    if (chrono::duration<float>(timer.elapsed_time()).count() >= 0.1f)
    {
        float deltaTime = chrono::duration<float>(timer.elapsed_time()).count();
        acceleration = (now_speed - _last_input) / deltaTime;
        _last_input = now_speed;
        timer.reset();
    }

    // 変化率抑制項の計算
    float rate = p_term + i_term + d_term;
    float rate_suppression = 0.0f;
    if (rate - _last_rate + 100 != 0)
    {
        rate_suppression = _rate_suppression_gain * (acceleration * 10 / _sample_acceleration / (rate - _last_rate + 100));
    }

    float output = _last_output + rate - rate_suppression;

    // 更新処理
    _last_error = error;
    _last_output = output;
    _last_rate = rate;

    return output;
}

void PID::setTunings(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setSampleTime(float sample_time)
{
    _sample_time = sample_time;
}

void PID::setRateSuppressionGain(float rate_suppression_gain)
{
    _rate_suppression_gain = rate_suppression_gain;
}

void PID::reset()
{
    _last_input = 0;
    _integral = 0;
    _last_output = 0;
    _last_error = 0;
    _last_rate = 0;
}

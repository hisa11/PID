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
    
    // Anti-windup: Clamp the integral term
    _integral += error * _sample_time;
    const float integral_max = 1000.0f; // Adjust as needed
    const float integral_min = -1000.0f; // Adjust as needed
    if (_integral > integral_max) _integral = integral_max;
    if (_integral < integral_min) _integral = integral_min;
    
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

    float desired_output = _last_output + rate - rate_suppression;

    // Output clamping
    const float output_max = 1000.0f; // Adjust as needed
    const float output_min = -1000.0f; // Adjust as needed
    if (desired_output > output_max) {
        desired_output = output_max;
        _integral = 0; // Reset integral term to prevent windup
    }
    if (desired_output < output_min) {
        desired_output = output_min;
        _integral = 0; // Reset integral term to prevent windup
    }

    // Limit the rate of change of the output
    const float max_change_rate = 10.0f; // Adjust as needed
    float output = _last_output;
    if (desired_output > _last_output + max_change_rate) {
        output = _last_output + max_change_rate;
    } else if (desired_output < _last_output - max_change_rate) {
        output = _last_output - max_change_rate;
    } else {
        output = desired_output;
    }

    // Debug output
    printf("P: %f, I: %f, D: %f, Rate: %f, Rate Suppression: %f, Desired Output: %f, Output: %f\n", p_term, i_term, d_term, rate, rate_suppression, desired_output, output);

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
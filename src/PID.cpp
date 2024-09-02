#include "PID.hpp"
#include <chrono>

PID::PID(float kp, float ki, float kd, float rate_suppression_gain, float Sample_acceleration, float sample_time)
    : _kp(kp), _ki(ki), _kd(kd), _sample_time(sample_time), _rate_suppression_gain(rate_suppression_gain), _Sample_acceleration(Sample_acceleration), _last_input(0), _floategral(0), _last_output(0), _last_error(0), _last_rate(0) {}

float PID::calculate(float setspeed, float nowspeed)
{
    int acceleration = 0;
    auto now = std::chrono::high_resolution_clock::now();
    static auto pre = std::chrono::high_resolution_clock::now();
    float error = setspeed - nowspeed;
    float p_term = _kp * error;
    _floategral += error * _sample_time;
    float i_term = _ki * _floategral;
    float d_term = _kd * (error - _last_error) / _sample_time;

    // 0.1秒ごとに実行
    std::chrono::duration<float> elapsed_time = now - pre;
    if (elapsed_time.count() >= 0.1f)
    {
        acceleration = nowspeed - _last_input;
        // accelerationを使用する処理を追加するか、コメントアウト
        // 例: std::cout << "Acceleration: " << acceleration << std::endl;
        pre = now; // 前回の実行時間を更新
    }

    // 変化率抑制項の計算
    float rate = p_term + i_term + d_term;
    float rate_suppression = _rate_suppression_gain * ((acceleration * 10 / _Sample_acceleration / (rate - _last_rate + 100)) - 1);
    float output = _last_output + p_term + i_term + d_term - rate_suppression;

    _last_input = nowspeed;
    _last_error = error;
    _last_output = output;
    _last_rate = rate;

    if (setspeed == nowspeed)
    {
        _floategral = 0;
    }

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
    _floategral = 0;
    _last_output = 0;
    _last_error = 0;
    _last_rate = 0;
}
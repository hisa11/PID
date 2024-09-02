#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    PID(float kp, float ki, float kd, float rate_suppression_gain,float Sample_acceleration, float sample_time);//Pゲイン、Iゲイン、Dゲイン、R(変化率抑制)ゲイン、サンプル加速度、サンプリング時間
    float calculate(float setspeed, float nowspeed);
    void setTunings(float kp, float ki, float kd);
    void setSampleTime(float sample_time);
    void setRateSuppressionGain(float rate_suppression_gain);
    void reset();

private:
    float _kp;
    float _ki;
    float _kd;
    float _sample_time;
    float _rate_suppression_gain;
    float _Sample_acceleration;
    float _last_input;
    float _floategral;
    float _last_output;
    float _last_error;
    float _last_rate;
};

#endif // PID_HPP
#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <deque>
#include <cmath>
#include <numbers>
#include "config.hpp"

class V21_RX
{
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples);
    void demodulate(const float *in_analog_samples, unsigned int n);
    ~V21_RX() = default;
private:
    float omega_mark, omega_space;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;

    float rl_cos0, rl_sin0, r_cos0, r_sin0;  
    float rl_cos1, rl_sin1, r_cos1, r_sin1;  
    
    float b[3];
    float a[3];
    
    std::deque<float> sample_buffer;
    float v0r_buffer = 0, v0i_buffer = 0;  
    float v1r_buffer = 0, v1i_buffer = 0;  
    float decisionBuffer[2] = {0, 0};
    float filtered_decisionBuffer[2] = {0, 0};
    
    enum State { IDLE, STARTED } state = IDLE;
    unsigned int counter = 0;
};

class V21_TX
{
public:
    V21_TX(float omega_mark, float omega_space) : 
        omega_mark(omega_mark), 
        omega_space(omega_space), 
        phase(0.f) {};
    
    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float phase;
};

#endif
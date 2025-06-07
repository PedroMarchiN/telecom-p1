#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <deque>
#include "config.hpp"

class V21_RX
{
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples);
    void demodulate(const float *in_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float rl_cos_space, rl_sin_space, r_cos_space, r_sin_space; 
    float rl_cos_mark, rl_sin_mark, r_cos_mark, r_sin_mark; 
    float lp_numerator[3];
    float lp_denominator[3];

    std::deque<float> sample_buffer;
    float vspace_r_buffer, vspace_i_buffer;
    float vmark_r_buffer, vmark_i_buffer;
    float raw_decision_buffer[2];
    float filtered_decision_buffer[2];

    enum {
        IDLE,
        CARRIER_DETECTED
    } state;
    unsigned int low_difference_counter;

    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
};

class V21_TX
{
public:
    V21_TX(float omega_mark, float omega_space) :omega_mark(omega_mark),omega_space(omega_space),phase(0.f) {};
    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float phase;
};

#endif
#include "v21.hpp"
#include <deque>
#include <cmath>
#include <numbers>
#include "config.hpp"

V21_RX::V21_RX(float omega_mark, float omega_space, 
               std::function<void(const unsigned int *, unsigned int)> get_digital_samples) :
    omega_mark(omega_mark),
    omega_space(omega_space),
    get_digital_samples(get_digital_samples)
{
    sample_buffer = std::deque<float>(SAMPLES_PER_SYMBOL, 0.0f);
    
    b[0] = 0.00037507f;
    b[1] = 0.00075014f;
    b[2] = 0.00037507;
    a[0] = 1.0f;
    a[1] = -1.94447766f;
    a[2] = 0.94597794f;

    const float BANDPASS_SMOOTHING = 0.99f;
    const int L = SAMPLES_PER_SYMBOL;
    
    rl_cos0 = std::pow(BANDPASS_SMOOTHING, L) * std::cos(omega_space * L * SAMPLING_PERIOD);
    rl_sin0 = std::pow(BANDPASS_SMOOTHING, L) * std::sin(omega_space * L * SAMPLING_PERIOD);
    r_cos0 = BANDPASS_SMOOTHING * std::cos(omega_space * SAMPLING_PERIOD);
    r_sin0 = BANDPASS_SMOOTHING * std::sin(omega_space * SAMPLING_PERIOD);
    
    rl_cos1 = std::pow(BANDPASS_SMOOTHING, L) * 
                  std::cos(omega_mark * L * SAMPLING_PERIOD);
    rl_sin1 = std::pow(BANDPASS_SMOOTHING, L) * 
                  std::sin(omega_mark * L * SAMPLING_PERIOD);
    r_cos1 = BANDPASS_SMOOTHING * std::cos(omega_mark * SAMPLING_PERIOD);
    r_sin1 = BANDPASS_SMOOTHING * std::sin(omega_mark * SAMPLING_PERIOD);
}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];
    const int L = SAMPLES_PER_SYMBOL;

    for (unsigned int i = 0; i < n; ++i) {
        sample_buffer.push_front(in_analog_samples[i]);
        sample_buffer.pop_back();
        
        float v0r = sample_buffer[0] - rl_cos0 * sample_buffer[L] + r_cos0 * v0r_buffer - r_sin0 * v0i_buffer;
        float v0i = -rl_sin0 * sample_buffer[L] +  r_cos0 * v0i_buffer + r_sin0 * v0r_buffer;
        float v1r = sample_buffer[0] - rl_cos1 * sample_buffer[L] + r_cos1 * v1r_buffer - r_sin1 * v1i_buffer;
        float v1i = -rl_sin1 * sample_buffer[L] + r_cos1 * v1i_buffer + r_sin1 * v1r_buffer;

        v0r_buffer = v0r;
        v0i_buffer = v0i;
        v1r_buffer = v1r;
        v1i_buffer = v1i;
        
        float raw_decision = (v1r * v1r + v1i * v1i) -
                             (v0r * v0r + v0i * v0i);

        float filtered_decision =  b[0] * raw_decision + b[1] * raw_decision_buffer[0] + b[2] * raw_decision_buffer[1] - a[1] * filtered_decision_buffer[0] - a[2] * filtered_decision_buffer[1];
        
        filtered_decision /= a[0];
        
        raw_decision_buffer[1] = raw_decision_buffer[0];
        raw_decision_buffer[0] = raw_decision;
        filtered_decision_buffer[1] = filtered_decision_buffer[0];
        filtered_decision_buffer[0] = filtered_decision;
        
        switch (state) {
            case IDLE:
                if (std::abs(filtered_decision) > 120.0f) {
                    digital_samples[i] = (filtered_decision > 0) ? 1 : 0;
                    counter = 0;
                    state = CARRIER_DETECTED;
                } else {
                    digital_samples[i] = 1; 
                }
                break;
                
            case CARRIER_DETECTED:
                if (std::abs(filtered_decision) < 60.0f) {
                    counter++;
                } else {
                    counter = 0;
                }
                
                if (counter >= 50) {
                    digital_samples[i] = 1;
                    state = IDLE;
                } else {
                    digital_samples[i] = (filtered_decision > 0) ? 1 : 0;
                }
                break;
        }
    }
    
    get_digital_samples(digital_samples, n);
}

void V21_TX::modulate(const unsigned int *in_digital_samples, 
                      float *out_analog_samples, 
                      unsigned int n)
{
    for (unsigned int i = 0; i < n; ++i) {
        out_analog_samples[i] = std::sin(phase);
        phase += (in_digital_samples[i] ? omega_mark : omega_space) * SAMPLING_PERIOD;
        phase = std::remainder(phase, 2 * std::numbers::pi_v<float>);
    }
}
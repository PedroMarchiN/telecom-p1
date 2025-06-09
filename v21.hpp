#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <array>
#include <cmath>
#include "config.hpp"

class V21_RX
{
public:
    V21_RX(float omega_mark, float omega_space, 
           std::function<void(const unsigned int *, unsigned int)> get_digital_samples);
    void demodulate(const float *in_analog_samples, unsigned int n);
    ~V21_RX() = default;

private:
    const float MARK_FREQ;
    const float SPACE_FREQ;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
    
    static constexpr size_t FIR_TAPS = 31;
    std::array<float, FIR_TAPS> fir_coeffs;
    
    std::array<float, SAMPLES_PER_SYMBOL> delay_line;
    size_t delay_index = 0;
    float last_i_mark = 0.0f;
    float last_q_mark = 0.0f;
    float last_i_space = 0.0f;
    float last_q_space = 0.0f;
    float phase_accumulator = 0.0f;
    float energy_history[5] = {0};
    size_t energy_index = 0;
    
    enum State { NO_CARRIER, CARRIER_PRESENT } state = NO_CARRIER;
    unsigned int low_energy_count = 0;

    float process_sample(float sample);
    void update_energy(float energy);
    bool carrier_detected();
};

class V21_TX
{
public:
    V21_TX(float omega_mark, float omega_space) : 
        MARK_FREQ(omega_mark), 
        SPACE_FREQ(omega_space), 
        phase(0.0f) {}
    
    void modulate(const unsigned int *in_digital_samples, 
                 float *out_analog_samples, 
                 unsigned int n);

private:
    const float MARK_FREQ;
    const float SPACE_FREQ;
    float phase;
};

#endif
#include "v21.hpp"
#include <cmath>
#include <numbers>
#include <array>
#include <algorithm>

constexpr float CUTOFF_FREQ = 350.0f; // Frequência de corte ajustada

V21_RX::V21_RX(float omega_mark, float omega_space, 
               std::function<void(const unsigned int *, unsigned int)> get_digital_samples) :
    MARK_FREQ(omega_mark),
    SPACE_FREQ(omega_space),
    get_digital_samples(get_digital_samples)
{

    delay_line.fill(0.0f);
    std::fill_n(energy_history, 5, 0.0f);

    const int N = FIR_TAPS - 1;
    const float fc = CUTOFF_FREQ / (SAMPLE_RATE / 2);
    
    for (size_t n = 0; n < FIR_TAPS; n++) {
        if (n == N/2) {
            fir_coeffs[n] = 2 * fc;
        } else {
            float t = static_cast<float>(n) - N/2.0f;
            fir_coeffs[n] = std::sin(2 * std::numbers::pi_v<float> * fc * t) / 
                           (std::numbers::pi_v<float> * t);
        }
        
        fir_coeffs[n] *= 0.54f - 0.46f * std::cos(2 * std::numbers::pi_v<float> * n / N);
    }
    
    float sum = std::accumulate(fir_coeffs.begin(), fir_coeffs.end(), 0.0f);
    for (auto& coeff : fir_coeffs) {
        coeff /= sum;
    }
}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];
    
    for (unsigned int i = 0; i < n; ++i) {
        float decision = process_sample(in_analog_samples[i]);
        
        float energy = std::abs(decision);
        update_energy(energy);
        
        switch (state) {
            case NO_CARRIER:
                digital_samples[i] = 1;
                if (carrier_detected()) {
                    state = CARRIER_PRESENT;
                    low_energy_count = 0;
                }
                break;
                
            case CARRIER_PRESENT:
                if (carrier_detected()) {
                    digital_samples[i] = (decision > 0) ? 1 : 0;
                    low_energy_count = 0;
                } else {
                    low_energy_count++;
                    if (low_energy_count > 50) {
                        state = NO_CARRIER;
                        digital_samples[i] = 1;
                    } else {
                        digital_samples[i] = (decision > 0) ? 1 : 0;
                    }
                }
                break;
        }
    }
    
    get_digital_samples(digital_samples, n);
}

float V21_RX::process_sample(float sample)
{
    delay_line[delay_index] = sample;
    delay_index = (delay_index + 1) % delay_line.size();
    
    float i_mark = 0.0f, q_mark = 0.0f;
    float i_space = 0.0f, q_space = 0.0f;
    
    for (size_t i = 0; i < delay_line.size(); i++) {
        float angle_mark = MARK_FREQ * (i * SAMPLING_PERIOD);
        float angle_space = SPACE_FREQ * (i * SAMPLING_PERIOD);
        
        i_mark += delay_line[i] * std::cos(angle_mark);
        q_mark += delay_line[i] * std::sin(angle_mark);
        
        i_space += delay_line[i] * std::cos(angle_space);
        q_space += delay_line[i] * std::sin(angle_space);
    }
    
    float energy_mark = i_mark * i_mark + q_mark * q_mark;
    float energy_space = i_space * i_space + q_space * q_space;
    float diff = energy_space - energy_mark;
    
    // Filtragem FIR
    float filtered = 0.0f;
    size_t coeff_index = FIR_TAPS - 1;
    
    for (size_t i = 0; i < FIR_TAPS; i++) {
        filtered += diff * fir_coeffs[coeff_index];
        coeff_index = (coeff_index > 0) ? coeff_index - 1 : FIR_TAPS - 1;
    }
    
    return filtered;
}

void V21_RX::update_energy(float energy)
{
    energy_history[energy_index] = energy;
    energy_index = (energy_index + 1) % 5;
}

bool V21_RX::carrier_detected()
{
    float avg_energy = 0.0f;
    for (float e : energy_history) {
        avg_energy += e;
    }
    avg_energy /= 5.0f;
    
    return avg_energy > 80.0f;
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
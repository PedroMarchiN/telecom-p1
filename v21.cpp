#include "v21.hpp"
#include <algorithm>
#include <numbers>

void V21_RX::init_filters() {
    // Filtros para demodulação coerente (I e Q para cada frequência)
    const float bw = 70.0f; // Largura de banda mais estreita
    
    for (int i = 0; i < 32; i++) {
        float n = i - 16;
        float t = n / sampling_rate;
        
        // Filtros para marca (1180 Hz)
        mark_i_coeffs[i] = 2 * bw/sampling_rate * std::cos(2 * M_PI * mark_freq * t) * 
                          (0.54f - 0.46f * std::cos(2 * M_PI * i / 31));
        mark_q_coeffs[i] = 2 * bw/sampling_rate * std::sin(2 * M_PI * mark_freq * t) * 
                          (0.54f - 0.46f * std::cos(2 * M_PI * i / 31));
        
        // Filtros para espaço (980 Hz)
        space_i_coeffs[i] = 2 * bw/sampling_rate * std::cos(2 * M_PI * space_freq * t) * 
                           (0.54f - 0.46f * std::cos(2 * M_PI * i / 31));
        space_q_coeffs[i] = 2 * bw/sampling_rate * std::sin(2 * M_PI * space_freq * t) * 
                           (0.54f - 0.46f * std::cos(2 * M_PI * i / 31));
        
        // Filtro passa-baixa para o envelope
        if (n == 0) {
            lowpass_coeffs[i] = 2 * 30.0f / sampling_rate;
        } else {
            lowpass_coeffs[i] = std::sin(2 * M_PI * 30.0f * n / sampling_rate) / (M_PI * n) * 
                               (0.54f - 0.46f * std::cos(2 * M_PI * i / 31));
        }
    }
}

float V21_RX::fir_filter(float sample, const std::array<float, 32>& coeffs, std::array<float, 32>& state) {
    std::rotate(state.begin(), state.begin() + 1, state.end());
    state.back() = sample;
    
    float output = 0.0f;
    for (int i = 0; i < 32; i++) {
        output += coeffs[i] * state[i];
    }
    return output;
}

bool V21_RX::detect_carrier() {
    bool carrier_present = (mark_energy > carrier_threshold) || 
                         (space_energy > carrier_threshold);
    
    if (carrier_present) {
        no_carrier_time = 0.0f;
        return true;
    } else {
        no_carrier_time += 1.0f/sampling_rate;
        return (no_carrier_time < NO_CARRIER_TIMEOUT);
    }
}

int V21_RX::decide_bit() {
    // Suaviza a decisão com histerese
    const float hysteresis = 0.1f;
    
    if (mark_energy > space_energy * (1.0f + hysteresis)) {
        return 1;
    } else if (space_energy > mark_energy * (1.0f + hysteresis)) {
        return 0;
    } else {
        return last_bit; // Mantém o último bit se não for clara a decisão
    }
}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n) {
    unsigned int digital_samples[n];
    
    for (unsigned int i = 0; i < n; i++) {
        float sample = in_analog_samples[i];
        
        // Demodulação coerente (I e Q para cada frequência)
        float mark_i = fir_filter(sample, mark_i_coeffs, mark_i_state);
        float mark_q = fir_filter(sample, mark_q_coeffs, mark_q_state);
        float space_i = fir_filter(sample, space_i_coeffs, space_i_state);
        float space_q = fir_filter(sample, space_q_coeffs, space_q_state);
        
        // Calcula as energias
        mark_energy = fir_filter(mark_i*mark_i + mark_q*mark_q, lowpass_coeffs, lowpass_state);
        space_energy = fir_filter(space_i*space_i + space_q*space_q, lowpass_coeffs, lowpass_state);
        
        if (!detect_carrier()) {
            digital_samples[i] = 1; // Linha ociosa
            last_bit = 1;
            continue;
        }
        
        last_bit = decide_bit();
        digital_samples[i] = last_bit;
    }
    
    get_digital_samples(digital_samples, n);
}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n) {
    while (n--) {
        *out_analog_samples++ = std::sin(phase);
        phase += (*in_digital_samples++ ? omega_mark : omega_space) * SAMPLING_PERIOD;
        phase = std::remainder(phase, 2*std::numbers::pi_v<float>);
    }
}
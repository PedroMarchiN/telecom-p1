#include "v21.hpp"
#include <cmath>
#include <algorithm>
#include <numbers>

void V21_RX::init_filters() {
    const float bw = 100.0f; // Largura de banda dos filtros
    const float mark_freq = omega_mark / (2 * std::numbers::pi_v<float>);
    const float space_freq = omega_space / (2 * std::numbers::pi_v<float>);
    const float sampling_rate = 1.0f / sampling_period;
    
    // Filtro passa-banda para marca (FIR com janela de Hamming)
    for (int i = 0; i < 32; i++) {
        float n = i - 16;
        if (n == 0) {
            mark_filter_coeffs[i] = 2 * bw / sampling_rate;
        } else {
            mark_filter_coeffs[i] = std::sin(2 * std::numbers::pi_v<float> * bw * n / sampling_rate) / 
                                  (std::numbers::pi_v<float> * n) *
                                  std::cos(2 * std::numbers::pi_v<float> * mark_freq * n / sampling_rate);
        }
        // Aplica janela de Hamming
        mark_filter_coeffs[i] *= 0.54f - 0.46f * std::cos(2 * std::numbers::pi_v<float> * i / 31);
    }
    
    // Filtro passa-banda para espaço
    for (int i = 0; i < 32; i++) {
        float n = i - 16;
        if (n == 0) {
            space_filter_coeffs[i] = 2 * bw / sampling_rate;
        } else {
            space_filter_coeffs[i] = std::sin(2 * std::numbers::pi_v<float> * bw * n / sampling_rate) / 
                                     (std::numbers::pi_v<float> * n) *
                                     std::cos(2 * std::numbers::pi_v<float> * space_freq * n / sampling_rate);
        }
        // Aplica janela de Hamming
        space_filter_coeffs[i] *= 0.54f - 0.46f * std::cos(2 * std::numbers::pi_v<float> * i / 31);
    }
    
    // Filtro passa-baixa para o detector de envelope
    const float cutoff = 50.0f; // Frequência de corte do passa-baixa
    for (int i = 0; i < 32; i++) {
        float n = i - 16;
        if (n == 0) {
            lowpass_filter_coeffs[i] = 2 * cutoff / sampling_rate;
        } else {
            lowpass_filter_coeffs[i] = std::sin(2 * std::numbers::pi_v<float> * cutoff * n / sampling_rate) / 
                                     (std::numbers::pi_v<float> * n);
        }
        // Aplica janela de Hamming
        lowpass_filter_coeffs[i] *= 0.54f - 0.46f * std::cos(2 * std::numbers::pi_v<float> * i / 31);
    }
}

float V21_RX::fir_filter(float sample, const std::array<float, 32>& coeffs, std::array<float, 32>& state) {
    // Desloca o estado do filtro
    std::rotate(state.begin(), state.begin() + 1, state.end());
    state.back() = sample;
    
    // Calcula a saída do filtro
    float output = 0.0f;
    for (int i = 0; i < 32; i++) {
        output += coeffs[i] * state[i];
    }
    
    return output;
}

bool V21_RX::detect_carrier(float mark_energy, float space_energy) {
    bool carrier_present = (mark_energy > carrier_threshold) || 
                         (space_energy > carrier_threshold);
    
    if (carrier_present) {
        no_carrier_time = 0.0f;
        return true;
    } else {
        no_carrier_time += sampling_period;
        return (no_carrier_time < NO_CARRIER_TIMEOUT);
    }
}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n) {
    unsigned int digital_samples[n];
    
    for (unsigned int i = 0; i < n; i++) {
        float sample = in_analog_samples[i];
        
        // Filtra o sinal nas frequências de marca e espaço
        float mark_filtered = fir_filter(sample, mark_filter_coeffs, mark_filter_state);
        float space_filtered = fir_filter(sample, space_filter_coeffs, space_filter_state);
        
        // Calcula a energia em cada frequência
        float mark_energy = mark_filtered * mark_filtered;
        float space_energy = space_filtered * space_filtered;
        
        // Detecta se há portadora presente
        bool carrier_present = detect_carrier(mark_energy, space_energy);
        
        if (!carrier_present) {
            digital_samples[i] = 1; // Linha ociosa
            continue;
        }
        
        // Calcula a diferença de energia e filtra
        float energy_diff = mark_energy - space_energy;
        float filtered_diff = fir_filter(energy_diff, lowpass_filter_coeffs, lowpass_filter_state);
        
        // Decisão do bit
        digital_samples[i] = (filtered_diff > 0) ? 1 : 0;
    }
    
    get_digital_samples(digital_samples, n);
}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n)
{
    while (n--) {
        *out_analog_samples++ = sin(phase);
        phase += (*in_digital_samples++ ? omega_mark : omega_space) * SAMPLING_PERIOD;

        // evita que phase cresça indefinidamente, o que causaria perda de precisão
        phase = remainder(phase, 2*std::numbers::pi);
    }
}

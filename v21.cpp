#include "v21.hpp"
#include <deque>
#include <cmath>
#include <numbers>
#include "config.hpp"

// Implementação do receptor
V21_RX::V21_RX(float omega_mark, float omega_space, 
               std::function<void(const unsigned int *, unsigned int)> get_digital_samples) :
    omega_mark(omega_mark),
    omega_space(omega_space),
    get_digital_samples(get_digital_samples)
{
    // Preenchimento inicial do buffer
    sample_buffer = std::deque<float>(SAMPLES_PER_SYMBOL, 0.0f);
    
    // Coeficientes do filtro passa-baixas (Butterworth)
    lp_numerator[0] = 0.00037506961629696616f;
    lp_numerator[1] = 0.0007501392325939323f;
    lp_numerator[2] = 0.00037506961629696616f;
    lp_denominator[0] = 1.0f;
    lp_denominator[1] = -1.9444776577670935f;
    lp_denominator[2] = 0.9459779362322813f;

    // Pré-cálculo das constantes dos filtros ressonantes
    const float BANDPASS_SMOOTHING = 0.99f;
    const int L = SAMPLES_PER_SYMBOL;
    
    rl_cos_space = std::pow(BANDPASS_SMOOTHING, L) * 
                   std::cos(omega_space * L * SAMPLING_PERIOD);
    rl_sin_space = std::pow(BANDPASS_SMOOTHING, L) * 
                   std::sin(omega_space * L * SAMPLING_PERIOD);
    r_cos_space = BANDPASS_SMOOTHING * std::cos(omega_space * SAMPLING_PERIOD);
    r_sin_space = BANDPASS_SMOOTHING * std::sin(omega_space * SAMPLING_PERIOD);
    
    rl_cos_mark = std::pow(BANDPASS_SMOOTHING, L) * 
                  std::cos(omega_mark * L * SAMPLING_PERIOD);
    rl_sin_mark = std::pow(BANDPASS_SMOOTHING, L) * 
                  std::sin(omega_mark * L * SAMPLING_PERIOD);
    r_cos_mark = BANDPASS_SMOOTHING * std::cos(omega_mark * SAMPLING_PERIOD);
    r_sin_mark = BANDPASS_SMOOTHING * std::sin(omega_mark * SAMPLING_PERIOD);
}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];
    const int L = SAMPLES_PER_SYMBOL;

    for (unsigned int i = 0; i < n; ++i) {
        // Atualiza buffer de amostras
        sample_buffer.push_front(in_analog_samples[i]);
        sample_buffer.pop_back();
        
        // Filtro ressonante para espaço (1650 Hz)
        float vspace_r = sample_buffer[0] - rl_cos_space * sample_buffer[L] +
                         r_cos_space * vspace_r_buffer - r_sin_space * vspace_i_buffer;
        float vspace_i = -rl_sin_space * sample_buffer[L] + 
                         r_cos_space * vspace_i_buffer + r_sin_space * vspace_r_buffer;
        
        // Filtro ressonante para marca (1850 Hz)
        float vmark_r = sample_buffer[0] - rl_cos_mark * sample_buffer[L] +
                        r_cos_mark * vmark_r_buffer - r_sin_mark * vmark_i_buffer;
        float vmark_i = -rl_sin_mark * sample_buffer[L] + 
                        r_cos_mark * vmark_i_buffer + r_sin_mark * vmark_r_buffer;
        
        // Atualiza estados dos filtros
        vspace_r_buffer = vspace_r;
        vspace_i_buffer = vspace_i;
        vmark_r_buffer = vmark_r;
        vmark_i_buffer = vmark_i;
        
        // Calcula diferença de energia
        float raw_decision = (vmark_r * vmark_r + vmark_i * vmark_i) -
                             (vspace_r * vspace_r + vspace_i * vspace_i);
        
        // Filtragem passa-baixas
        float filtered_decision = 
            lp_numerator[0] * raw_decision +
            lp_numerator[1] * raw_decision_buffer[0] +
            lp_numerator[2] * raw_decision_buffer[1] -
            lp_denominator[1] * filtered_decision_buffer[0] -
            lp_denominator[2] * filtered_decision_buffer[1];
        
        filtered_decision /= lp_denominator[0];
        
        // Atualiza buffers de decisão
        raw_decision_buffer[1] = raw_decision_buffer[0];
        raw_decision_buffer[0] = raw_decision;
        filtered_decision_buffer[1] = filtered_decision_buffer[0];
        filtered_decision_buffer[0] = filtered_decision;
        
        // Máquina de estados para detecção de portadora
        switch (state) {
            case IDLE:
                if (std::abs(filtered_decision) > 120.0f) {
                    digital_samples[i] = (filtered_decision > 0) ? 1 : 0;
                    low_difference_counter = 0;
                    state = CARRIER_DETECTED;
                } else {
                    digital_samples[i] = 1; // Linha ociosa
                }
                break;
                
            case CARRIER_DETECTED:
                if (std::abs(filtered_decision) < 60.0f) {
                    low_difference_counter++;
                } else {
                    low_difference_counter = 0;
                }
                
                if (low_difference_counter >= 50) {
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

// Implementação do transmissor
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
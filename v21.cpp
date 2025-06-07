#include "v21.hpp"
#include <algorithm>
#include <numbers>

float V21_RX::lowpass_filter(float input) {
    // Implementação do filtro IIR passa-baixa
    float output = b_coeffs[0] * input + filter_state[0];
    filter_state[0] = b_coeffs[1] * input - a_coeffs[1] * output + filter_state[1];
    filter_state[1] = b_coeffs[2] * input - a_coeffs[2] * output;
    return output;
}

void V21_RX::update_carrier_state(float decision) {
    float abs_decision = std::abs(decision);
    
    if (!carrier_state) {
        // Estado sem portadora - verifica se deve entrar no estado com portadora
        if (abs_decision > CARRIER_THRESHOLD_HIGH) {
            carrier_state = true;
            carrier_hold_counter = 0;
        }
    } else {
        // Estado com portadora - verifica se deve sair
        if (abs_decision < CARRIER_THRESHOLD_LOW) {
            carrier_hold_counter++;
            if (carrier_hold_counter >= CARRIER_HOLD_COUNT) {
                carrier_state = false;
            }
        } else {
            carrier_hold_counter = 0;
        }
    }
}

int V21_RX::decide_bit(float decision) {
    // Histerese na decisão
    const float HYSTERESIS = 20.0f;
    static int last_bit = 1;
    
    if (decision > HYSTERESIS) {
        last_bit = 1;
    } else if (decision < -HYSTERESIS) {
        last_bit = 0;
    }
    
    return last_bit;
}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n) {
    unsigned int digital_samples[n];
    
    for (unsigned int i = 0; i < n; i++) {
        float sample = in_analog_samples[i];
        
        // Atualiza buffer de atraso
        float delayed_sample = delay_buffer[delay_index];
        delay_buffer[delay_index] = sample;
        delay_index = (delay_index + 1) % L;
        
        // Filtros ressonantes para marca (1180Hz)
        float new_mark_r = sample - r_pow_L * cos_mark_L * delayed_sample + 
                         r * (cos_mark * mark_r - sin_mark * mark_i);
        float new_mark_i = -r_pow_L * sin_mark_L * delayed_sample + 
                         r * (sin_mark * mark_r + cos_mark * mark_i);
        
        // Filtros ressonantes para espaço (980Hz)
        float new_space_r = sample - r_pow_L * cos_space_L * delayed_sample + 
                          r * (cos_space * space_r - sin_space * space_i);
        float new_space_i = -r_pow_L * sin_space_L * delayed_sample + 
                          r * (sin_space * space_r + cos_space * space_i);
        
        // Atualiza estados dos filtros
        mark_r = new_mark_r;
        mark_i = new_mark_i;
        space_r = new_space_r;
        space_i = new_space_i;
        
        // Calcula a diferença de energia
        float decision = (space_r*space_r + space_i*space_i) - 
                       (mark_r*mark_r + mark_i*mark_i);
        
        // Filtra a decisão
        float filtered_decision = lowpass_filter(decision);
        
        // Atualiza estado da portadora
        update_carrier_state(filtered_decision);
        
        // Decisão do bit
        if (!carrier_state) {
            digital_samples[i] = 1; // Linha ociosa
        } else {
            digital_samples[i] = decide_bit(filtered_decision);
        }
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
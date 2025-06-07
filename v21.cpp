#include <cstdio>
#include <cmath>
#include <numbers>
#include "v21.hpp"

constexpr float BANDPASS_SMOOTHING = 0.99f;

V21_RX::V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples) :
    omega_mark(omega_mark),
    omega_space(omega_space),
    get_digital_samples(get_digital_samples) 
{
    // Filling buffers with "empty" values
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++)
        sample_buffer.push_front(0.f);
        
    vspace_r_buffer = 0.f;
    vspace_i_buffer = 0.f;
    vmark_r_buffer = 0.f;
    vmark_i_buffer = 0.f;
    raw_decision_buffer[0] = 0.f;
    raw_decision_buffer[1] = 0.f;
    filtered_decision_buffer[0] = 0.f;
    filtered_decision_buffer[1] = 0.f;
    low_difference_counter = 0;

    // Low-pass Butterworth filter coefficients
    lp_numerator[0] = 0.00037506961629696616f;
    lp_numerator[1] = 0.0007501392325939323f;
    lp_numerator[2] = 0.00037506961629696616f;
    lp_denominator[0] = 1.f;
    lp_denominator[1] = -1.9444776577670935f;
    lp_denominator[2] = 0.9459779362322813f;

    // Precomputing sines and cosines
    rl_cos_space = powf(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL) * 
                  cosf(omega_space * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD);
    rl_sin_space = powf(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL) * 
                  sinf(omega_space * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD);
    r_cos_space = BANDPASS_SMOOTHING * cosf(omega_space * SAMPLING_PERIOD);
    r_sin_space = BANDPASS_SMOOTHING * sinf(omega_space * SAMPLING_PERIOD);

    rl_cos_mark = powf(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL) * 
                 cosf(omega_mark * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD);
    rl_sin_mark = powf(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL) * 
                 sinf(omega_mark * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD);
    r_cos_mark = BANDPASS_SMOOTHING * cosf(omega_mark * SAMPLING_PERIOD);
    r_sin_mark = BANDPASS_SMOOTHING * sinf(omega_mark * SAMPLING_PERIOD);

    state = IDLE; // Inicialização crítica do estado
}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];
    const int L = SAMPLES_PER_SYMBOL;

    for (unsigned int i = 0; i < n; i++) {
        sample_buffer.push_front(in_analog_samples[i]);

        float vspace_r = sample_buffer[0] - rl_cos_space * sample_buffer[L] +
                         r_cos_space * vspace_r_buffer - r_sin_space * vspace_i_buffer;
                         
        float vspace_i = -rl_sin_space * sample_buffer[L] +
                         r_cos_space * vspace_i_buffer + r_sin_space * vspace_r_buffer;
                         
        float vmark_r = sample_buffer[0] - rl_cos_mark * sample_buffer[L] +
                        r_cos_mark * vmark_r_buffer - r_sin_mark * vmark_i_buffer;
                        
        float vmark_i = -rl_sin_mark * sample_buffer[L] +
                        r_cos_mark * vmark_i_buffer + r_sin_mark * vmark_r_buffer;

        // Calcula a diferença de energia entre os tons
        float raw_decision = (vmark_r * vmark_r + vmark_i * vmark_i) -
                             (vspace_r * vspace_r + vspace_i * vspace_i);

        // Aplica filtro passa-baixas
        float filtered_decision = 
            lp_numerator[0] * raw_decision +
            lp_numerator[1] * raw_decision_buffer[0] +
            lp_numerator[2] * raw_decision_buffer[1] -
            lp_denominator[1] * filtered_decision_buffer[0] -
            lp_denominator[2] * filtered_decision_buffer[1];
            
        filtered_decision /= lp_denominator[0];

        // Máquina de estados para detecção de portadora e decodificação
        switch (state) {
            case IDLE:
                if (std::abs(filtered_decision) > 120) {
                    // Tom detectado: espaço (0) se positivo, marca (1) se negativo
                    digital_samples[i] = (filtered_decision > 0) ? 0 : 1;
                    low_difference_counter = 0;
                    state = CARRIER_DETECTED;
                } else {
                    // Sem portadora - mantém linha ociosa (1)
                    digital_samples[i] = 1;
                }
                break;

            case CARRIER_DETECTED:
                if (std::abs(filtered_decision) < 60) {
                    low_difference_counter++;
                } else {
                    low_difference_counter = 0;
                }

                if (low_difference_counter >= 50) {
                    // Perda de portadora detectada
                    digital_samples[i] = 1;
                    state = IDLE;
                } else {
                    // Decodificação normal
                    digital_samples[i] = (filtered_decision > 0) ? 0 : 1;
                }
                break;
        }

        // Atualiza buffers
        sample_buffer.pop_back();
        vspace_r_buffer = vspace_r;
        vspace_i_buffer = vspace_i;
        vmark_r_buffer = vmark_r;
        vmark_i_buffer = vmark_i;
        raw_decision_buffer[1] = raw_decision_buffer[0];
        raw_decision_buffer[0] = raw_decision;
        filtered_decision_buffer[1] = filtered_decision_buffer[0];
        filtered_decision_buffer[0] = filtered_decision;
    }

    get_digital_samples(digital_samples, n);
}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n)
{
    while (n--) {
        *out_analog_samples++ = sin(phase);
        phase += (*in_digital_samples++ ? omega_mark : omega_space) * SAMPLING_PERIOD;
        phase = fmod(phase, 2*std::numbers::pi_v<float>);
    }
}
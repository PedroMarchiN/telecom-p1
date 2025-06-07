#include <cstdio>
#include <math.h>
#include <numbers>
#include "v21.hpp"

constexpr float BANDPASS_SMOOTHING = 0.99;

V21_RX::V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples) :
    omega_mark(omega_mark),
    omega_space(omega_space),
    get_digital_samples(get_digital_samples) 
{
    // Filling buffers with "empty" values
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++)
        this->sample_buffer.push_front(0.f);
    this->vspace_r_buffer = 0.f;
    this->vspace_i_buffer = 0.f;
    this->vmark_r_buffer = 0.f;
    this->vmark_i_buffer = 0.f;
    this->raw_decision_buffer[0] = 0.f;
    this->raw_decision_buffer[1] = 0.f;
    this->filtered_decision_buffer[0] = 0.f;
    this->filtered_decision_buffer[1] = 0.f;
    this->low_difference_counter = 0;

    // Low-pass Butterworth filter generated with scipy.signal
    this->lp_numerator[0] = 0.00037506961629696616f;
    this->lp_numerator[1] = 0.0007501392325939323f;
    this->lp_numerator[2] = 0.00037506961629696616f;
    this->lp_denominator[0] = 1.f;
    this->lp_denominator[1] = -1.9444776577670935f;
    this->lp_denominator[2] = 0.9459779362322813f;

    // Precomputing sines and cosines
    this->rl_cos_space =
        pow(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL)
        * cos(omega_space * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD); 
    this->rl_sin_space =
        pow(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL)
        * sin(omega_space * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD); 
    this->r_cos_space = BANDPASS_SMOOTHING * cos(omega_space * SAMPLING_PERIOD); 
    this->r_sin_space = BANDPASS_SMOOTHING * sin(omega_space * SAMPLING_PERIOD); 

    this->rl_cos_mark =
        pow(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL)
        * cos(omega_mark * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD); 
    this->rl_sin_mark =
        pow(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL)
        * sin(omega_mark * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD); 
    this->r_cos_mark = BANDPASS_SMOOTHING * cos(omega_mark * SAMPLING_PERIOD); 
    this->r_sin_mark = BANDPASS_SMOOTHING * sin(omega_mark * SAMPLING_PERIOD); 
};

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];
    const int L = SAMPLES_PER_SYMBOL;

    for (int i = 0; i < n; i++) {
        this->sample_buffer.push_front(in_analog_samples[i]);

        float vspace_r = this->sample_buffer[0] - this->rl_cos_space * this->sample_buffer[L]
                            + this->r_cos_space * this->vspace_r_buffer - this->r_sin_space * vspace_i_buffer;
        float vspace_i = -this->rl_sin_space * this->sample_buffer[L] + this->r_cos_space * vspace_i_buffer
                            + this->r_sin_space * vspace_r_buffer;
        float vmark_r = this->sample_buffer[0] - this->rl_cos_mark * this->sample_buffer[L]
                            + this->r_cos_mark * this->vmark_r_buffer - this->r_sin_mark * vmark_i_buffer;
        float vmark_i = -this->rl_sin_mark * this->sample_buffer[L] + this->r_cos_mark * vmark_i_buffer
                            + this->r_sin_mark * vmark_r_buffer;

        float raw_decision = vmark_r * vmark_r + vmark_i * vmark_i -
                                vspace_r * vspace_r - vspace_i * vspace_i; 

        // a[0]*y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[M]*x[n-M]
        //               - a[1]*y[n-1] - ... - a[N]*y[n-N]
        float filtered_decision = 
            this->lp_numerator[0] * raw_decision
            + this->lp_numerator[1] * this->raw_decision_buffer[0]
            + this->lp_numerator[2] * this->raw_decision_buffer[1]
            - this->lp_denominator[1] * this->filtered_decision_buffer[0]
            - this->lp_denominator[2] * this->filtered_decision_buffer[1];
        filtered_decision /= this->lp_denominator[0];

        switch (this->state) {
            case IDLE:
                if (abs(filtered_decision) > 120) {
                    digital_samples[i] = filtered_decision > 0 ? 1 : 0;
                    this->low_difference_counter = 0;
                    this->state = CARRIER_DETECTED;
                } else
                    digital_samples[i] = 1;

                break;

            case CARRIER_DETECTED:
                if (abs(filtered_decision) < 60)
                    this->low_difference_counter++;
                else
                    this->low_difference_counter = 0;

                if (this->low_difference_counter >= 50) {
                    digital_samples[i] = 1;
                    this->state = IDLE;
                } else
                    digital_samples[i] = filtered_decision > 0 ? 1 : 0;

                break;

            default: break;
        }

        // Updating buffers 
        this->sample_buffer.pop_back();
        this->vspace_r_buffer = vspace_r;
        this->vspace_i_buffer = vspace_i;
        this->vmark_r_buffer = vmark_r;
        this->vmark_i_buffer = vmark_i;
        this->raw_decision_buffer[1] = this->raw_decision_buffer[0];
        this->raw_decision_buffer[0] = raw_decision;
        this->filtered_decision_buffer[1] = this->filtered_decision_buffer[0];
        this->filtered_decision_buffer[0] = filtered_decision;
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
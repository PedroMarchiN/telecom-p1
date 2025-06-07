#include <cmath>
#include <numbers>
#include "v21.hpp"

V21_RX::V21_RX(float omega_mark, float omega_space,
               std::function<void(const unsigned int *, unsigned int)> cb)
    : omega_mark(omega_mark), omega_space(omega_space),
      get_digital_samples(cb),
      bp_mark(omega_mark, SAMPLING_RATE),
      bp_space(omega_space, SAMPLING_RATE),
      lp_diff(0.99f)
{}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    digital_samples.clear();
    digital_samples.reserve(n);

    for (unsigned int i = 0; i < n; ++i) {
        float x = in_analog_samples[i];

        float power_mark  = bp_mark.process(x);
        float power_space = bp_space.process(x);
        float y_diff      = lp_diff.process(power_space - power_mark);

        bool no_carrier = (std::fabs(power_mark) + std::fabs(power_space) < noise_floor);
        unsigned int bit = no_carrier ? 1u : (y_diff > threshold ? 0u : 1u);

        digital_samples.push_back(bit);
    }

    get_digital_samples(digital_samples.data(), digital_samples.size());
}

V21_TX::V21_TX(float omega_mark, float omega_space)
    : omega_mark(omega_mark), omega_space(omega_space), phase(0.0f)
{}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n)
{
    for (unsigned int i = 0; i < n; ++i) {
        float omega = in_digital_samples[i] ? omega_mark : omega_space;
        out_analog_samples[i] = std::sin(phase);
        phase += omega * SAMPLING_PERIOD;

        // mantém fase dentro do intervalo para evitar overflow
        if (phase > std::numbers::pi)
            phase -= 2 * std::numbers::pi;
        else if (phase < -std::numbers::pi)
            phase += 2 * std::numbers::pi;
    }
}

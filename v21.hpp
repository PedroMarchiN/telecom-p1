#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <array>
#include <cmath>
#include "config.hpp"

class V21_RX {
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples)
        : omega_mark(omega_mark), omega_space(omega_space),
          get_digital_samples(get_digital_samples),
          mark_freq(omega_mark / (2 * M_PI)),
          space_freq(omega_space / (2 * M_PI)),
          sampling_rate(SAMPLING_RATE),
          last_bit(1),
          mark_energy(0.0f),
          space_energy(0.0f),
          carrier_threshold(0.001f),
          no_carrier_time(0.0f)
    {
        init_filters();
    };

    void demodulate(const float *in_analog_samples, unsigned int n);

private:
    float omega_mark, omega_space;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
    
    // Parâmetros do sinal
    float mark_freq, space_freq;
    float sampling_rate;
    
    // Estado do demodulador
    int last_bit;
    float mark_energy, space_energy;
    float carrier_threshold;
    float no_carrier_time;
    const float NO_CARRIER_TIMEOUT = 0.1f; // 100ms sem portadora
    
    // Filtros
    std::array<float, 32> mark_i_coeffs, mark_q_coeffs;
    std::array<float, 32> space_i_coeffs, space_q_coeffs;
    std::array<float, 32> mark_i_state, mark_q_state;
    std::array<float, 32> space_i_state, space_q_state;
    std::array<float, 32> lowpass_coeffs;
    std::array<float, 32> lowpass_state;
    
    void init_filters();
    float fir_filter(float sample, const std::array<float, 32>& coeffs, std::array<float, 32>& state);
    bool detect_carrier();
    int decide_bit();
};

class V21_TX {
public:
    V21_TX(float omega_mark, float omega_space) : omega_mark(omega_mark), omega_space(omega_space), phase(0.f) {};
    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float phase;
};

#endif
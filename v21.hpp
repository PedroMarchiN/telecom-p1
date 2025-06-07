#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <array>
#include <vector>
#include "config.hpp"

class V21_RX
{
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples)
        : omega_mark(omega_mark), omega_space(omega_space), 
          get_digital_samples(get_digital_samples),
          sampling_period(1.0f/SAMPLING_RATE),
          carrier_threshold(0.01f),
          no_carrier_time(0.0f)
    {
        // Inicializa os estados dos filtros
        mark_filter_state.fill(0.0f);
        space_filter_state.fill(0.0f);
        lowpass_filter_state.fill(0.0f);
        
        // Calcula os coeficientes dos filtros
        init_filters();
    };
    
    void demodulate(const float *in_analog_samples, unsigned int n);
    
private:
    float omega_mark, omega_space;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
    
    // Membros adicionados para a demodulação
    float sampling_period;
    float carrier_threshold;
    float no_carrier_time;
    const float NO_CARRIER_TIMEOUT = 0.1f; // 100ms sem portadora
    
    // Coeficientes dos filtros
    std::array<float, 32> mark_filter_coeffs;
    std::array<float, 32> space_filter_coeffs;
    std::array<float, 32> lowpass_filter_coeffs;
    
    // Estados dos filtros
    std::array<float, 32> mark_filter_state;
    std::array<float, 32> space_filter_state;
    std::array<float, 32> lowpass_filter_state;
    
    // Métodos auxiliares
    void init_filters();
    float fir_filter(float sample, const std::array<float, 32>& coeffs, std::array<float, 32>& state);
    bool detect_carrier(float mark_energy, float space_energy);
};

class V21_TX
{
public:
    V21_TX(float omega_mark, float omega_space) : omega_mark(omega_mark), omega_space(omega_space), phase(0.f) {};
    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float phase;
};

#endif
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
          sampling_period(1.0f/SAMPLING_RATE),
          r(0.99f),  // Fator de amortecimento
          carrier_state(false),
          carrier_hold_counter(0)
    {
        // Inicializa os estados dos filtros
        mark_i = mark_r = space_i = space_r = 0.0f;
        
        // Calcula constantes dos filtros ressonantes
        L = static_cast<int>(SAMPLING_RATE / 300);  // Taxa de símbolos (300 baud)
        cos_mark = std::cos(omega_mark * sampling_period);
        sin_mark = std::sin(omega_mark * sampling_period);
        cos_space = std::cos(omega_space * sampling_period);
        sin_space = std::sin(omega_space * sampling_period);
        
        // Constantes para os filtros ressonantes
        r_pow_L = std::pow(r, L);
        cos_mark_L = std::cos(omega_mark * L * sampling_period);
        sin_mark_L = std::sin(omega_mark * L * sampling_period);
        cos_space_L = std::cos(omega_space * L * sampling_period);
        sin_space_L = std::sin(omega_space * L * sampling_period);
        
        // Inicializa buffer de atraso
        delay_buffer.resize(L, 0.0f);
        delay_index = 0;
    };

    void demodulate(const float *in_analog_samples, unsigned int n);

private:
    float omega_mark, omega_space;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
    
    // Parâmetros dos filtros
    float sampling_period;
    float r;  // Fator de amortecimento
    int L;    // Atraso em amostras
    
    // Estados dos filtros ressonantes
    float mark_r, mark_i;  // Componentes real e imaginário para marca
    float space_r, space_i; // Componentes real e imaginário para espaço
    
    // Constantes pré-calculadas
    float cos_mark, sin_mark, cos_space, sin_space;
    float r_pow_L, cos_mark_L, sin_mark_L, cos_space_L, sin_space_L;
    
    // Buffer de atraso
    std::vector<float> delay_buffer;
    size_t delay_index;
    
    // Detecção de portadora
    bool carrier_state;
    int carrier_hold_counter;
    const float CARRIER_THRESHOLD_HIGH = 120.0f;
    const float CARRIER_THRESHOLD_LOW = 60.0f;
    const int CARRIER_HOLD_COUNT = 50;
    
    // Filtro passa-baixa
    std::array<float, 3> b_coeffs = {0.0004166f, 0.0008332f, 0.0004166f};
    std::array<float, 2> a_coeffs = {1.0f, -1.99111f, 0.99116f};
    std::array<float, 2> filter_state = {0.0f, 0.0f};
    
    float lowpass_filter(float input);
    void update_carrier_state(float decision);
    int decide_bit(float decision);
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
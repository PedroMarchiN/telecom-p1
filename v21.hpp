#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <deque>
#include <cmath>
#include <numbers>
#include "config.hpp"

class V21_RX
{
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples);
    void demodulate(const float *in_analog_samples, unsigned int n);
    ~V21_RX() = default;
private:
    float omega_mark, omega_space;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
    
    // Constantes pré-calculadas para os filtros ressonantes
    float rl_cos_space, rl_sin_space, r_cos_space, r_sin_space;
    float rl_cos_mark, rl_sin_mark, r_cos_mark, r_sin_mark;
    
    // Coeficientes do filtro passa-baixas
    float lp_numerator[3];
    float lp_denominator[3];
    
    // Buffers de estado
    std::deque<float> sample_buffer;
    float vspace_r_buffer = 0.0f, vspace_i_buffer = 0.0f;
    float vmark_r_buffer = 0.0f, vmark_i_buffer = 0.0f;
    float raw_decision_buffer[2] = {0.0f, 0.0f};
    float filtered_decision_buffer[2] = {0.0f, 0.0f};
    
    // Máquina de estados para detecção de portadora
    enum State { IDLE, CARRIER_DETECTED } state = IDLE;
    unsigned int low_difference_counter = 0;
};

class V21_TX
{
public:
    V21_TX(float omega_mark, float omega_space) : 
        omega_mark(omega_mark), 
        omega_space(omega_space), 
        phase(0.f) {};
    
    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float phase;
};

#endif
#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <cmath>
#include <numbers>
#include "config.hpp"
#include <cstring>

// Constantes globais (definidas no .cpp correspondente)
extern int R;
extern int fs;
extern double T;
extern int L;
extern float r;
extern float rL;
extern float cos_omega0_L;
extern float sin_omega0_L;
extern float cos_omega0;
extern float sin_omega0;
extern float cos_omega1_L;
extern float sin_omega1_L;
extern float cos_omega1;
extern float sin_omega1;

class V21_RX
{
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples)
        : omega_mark(omega_mark), omega_space(omega_space), get_digital_samples(get_digital_samples),
          input_buffer(new float[L]()), input_buffer_index(0),
          last_v0r(0.0f), last_v0i(0.0f), last_v1r(0.0f), last_v1i(0.0f),
          x1(0.0f), x2(0.0f), y1(0.0f), y2(0.0f), threshold(1e-5f)
    {
        std::memset(input_buffer, 0, L * sizeof(float));
    }

    ~V21_RX() {
        delete[] input_buffer;
    }

    void demodulate(const float *in_analog_samples, unsigned int n);

private:
    float omega_mark, omega_space;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
    
    // Buffer circular para amostras de entrada
    float* input_buffer;
    int input_buffer_index;
    
    // Estados dos filtros ressonantes
    float last_v0r, last_v0i;
    float last_v1r, last_v1i;
    
    // Estados do filtro IIR passa-baixas
    float x1, x2, y1, y2;
    
    // Limiar para detecção de portadora
    const float threshold;
};

class V21_TX
{
public:
    V21_TX(float omega_mark, float omega_space) :omega_mark(omega_mark),omega_space(omega_space),phase(0.f) {};
    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float phase;
};

#endif
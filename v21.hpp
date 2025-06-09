#ifndef V21_HPP
#define V21_HPP

#include <vector>
#include <functional>
#include <deque>
#include <cmath>
#include <numbers>
#include "config.hpp"


// Filtro passa-banda simples (IIR ressonante)
class BandPassFilter {
public:
    BandPassFilter() : omega(0.f), r(0.99f), y1(0.f), y2(0.f) {}
    BandPassFilter(float omega, float fs) {
        set_params(omega, fs);
    }

    void set_params(float omega, float fs) {
        float bw = 50.0f; // largura de banda (ajustável)
        r = 1.0f - 2.0f * bw / fs;
        this->omega = omega;
        y1 = y2 = 0.0f;
    }

    float process(float x) {
        float y = x - 2 * r * std::cos(omega) * y1 + r * r * y2;
        y2 = y1;
        y1 = y;
        return y * y; // retorna potência (módulo ao quadrado)
    }

private:
    float omega, r;
    float y1, y2;
};

// Filtro passa-baixa (IIR exponencial simples)
class LowPassFilter {
public:
    LowPassFilter(float r = 0.99f) : r(r), v1(0.f), v2(0.f) {}

    float process(float x) {
        float v = (1 - r) * std::fabs(x) + 2 * r * v1 - r * r * v2;
        v2 = v1;
        v1 = v;
        return v;
    }

private:
    float r;
    float v1, v2;
};

class V21_RX {
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples);
    void demodulate(const float *in_analog_samples, unsigned int n);
    ~V21_RX() = default;
private:
    float omega_mark, omega_space;
    float noise_floor = 0.01f;
    float threshold = 0.0f;

    BandPassFilter bp_mark;
    BandPassFilter bp_space;
    LowPassFilter lp_diff;

    std::vector<unsigned int> digital_samples;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;

    float rl_cos0, rl_sin0, r_cos0, r_sin0;  
    float rl_cos1, rl_sin1, r_cos1, r_sin1;  
    
    float b[3];
    float a[3];
    
    std::deque<float> sample_buffer;
    float v0r_buffer = 0, v0i_buffer = 0;  
    float v1r_buffer = 0, v1i_buffer = 0;  
    float decisionBuffer[2] = {0, 0};
    float filtered_decisionBuffer[2] = {0, 0};
    
    enum State { IDLE, STARTED } state = IDLE;
    unsigned int counter = 0;
};

class V21_TX {
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

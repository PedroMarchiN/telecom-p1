#ifndef V21_HPP
#define V21_HPP

#include <vector>
#include <functional>
#include <cmath>
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
    V21_RX(float omega_mark, float omega_space,
           std::function<void(const unsigned int *, unsigned int)> get_digital_samples);

    void demodulate(const float *in_analog_samples, unsigned int n);

private:
    float omega_mark, omega_space;
    float noise_floor = 0.01f;
    float threshold = 0.0f;

    BandPassFilter bp_mark;
    BandPassFilter bp_space;
    LowPassFilter lp_diff;

    std::vector<unsigned int> digital_samples;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
};

class V21_TX {
public:
    V21_TX(float omega_mark, float omega_space);

    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);

private:
    float omega_mark, omega_space;
    float phase;
};

#endif // V21_HPP

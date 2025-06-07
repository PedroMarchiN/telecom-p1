#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <vector>
#include <cmath>
#include "config.hpp"

// Filtros simples IIR que você vai implementar no .cpp
struct BandPassFilter {
    float r = 0.99f;
    float omega = 0.f;
    float x_prev = 0.f, x_prev_L = 0.f;
    float vi = 0.f, vr = 0.f;

    BandPassFilter() = default;
    BandPassFilter(float omega) : omega(omega) {}

    float process(float x) {
        // Exemplo simplificado: você pode colocar sua lógica real aqui.
        float out = vr; // ou qualquer outra lógica real
        // Atualize vi, vr como no código Python
        return out;
    }
};

struct LowPassFilter {
    float r = 0.999f;
    float v1 = 0.f, v2 = 0.f;

    float process(float x) {
        float v = (1 - r) * std::fabs(x) + 2 * r * v1 - r * r * v2;
        float y = v - v2;
        v2 = v1;
        v1 = v;
        return y;
    }
};

class V21_RX {
public:
    V21_RX(float omega_mark, float omega_space,
           std::function<void(const unsigned int *, unsigned int)> get_digital_samples)
        : omega_mark(omega_mark),
          omega_space(omega_space),
          get_digital_samples(get_digital_samples),
          bp_mark(omega_mark),
          bp_space(omega_space) {}

    void demodulate(const float *in_analog_samples, unsigned int n);

private:
    float omega_mark, omega_space;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;

    // Filtros e buffers
    BandPassFilter bp_mark;
    BandPassFilter bp_space;
    LowPassFilter lp_diff;
    std::vector<unsigned int> digital_samples;

    // Parâmetros de decisão
    float threshold = 0.0f;
    float noise_floor = 1e-4f;
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

#include <cstring>
#include <cmath>
#include <numbers>
#include "v21.hpp"

// Constantes globais
int R = 300;
int fs = 48000;
double T = 1.0 / fs;
int L = fs / R;
float r = 0.99f;

double omega0 = 2 * std::numbers::pi * 1850;
double omega1 = 2 * std::numbers::pi * 1650;

// Pré-calcular constantes
float rL = powf(r, (float)L);
float cos_omega0_L = cosf(omega0 * L * T);
float sin_omega0_L = sinf(omega0 * L * T);
float cos_omega0 = cosf(omega0 * T);
float sin_omega0 = sinf(omega0 * T);

float cos_omega1_L = cosf(omega1 * L * T);
float sin_omega1_L = sinf(omega1 * L * T);
float cos_omega1 = cosf(omega1 * T);
float sin_omega1 = sinf(omega1 * T);

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];
    
    // Coeficientes do filtro IIR passa-baixas
    const float b[] = {0.00094469f, 0.00188938f, 0.00094469f};
    const float a[] = {1.0f, -1.91119707f, 0.91497583f};

    for (unsigned int i = 0; i < n; i++) {
        float s_n = in_analog_samples[i];
        float s_n_L = input_buffer[input_buffer_index];
        input_buffer[input_buffer_index] = s_n;
        input_buffer_index = (input_buffer_index + 1) % L;

        // Filtros ressonantes
        float v0r = s_n - rL * cos_omega0_L * s_n_L + r * cos_omega0 * last_v0r - r * sin_omega0 * last_v0i;
        float v0i =      - rL * sin_omega0_L * s_n_L + r * cos_omega0 * last_v0i + r * sin_omega0 * last_v0r;
        float v1r = s_n - rL * cos_omega1_L * s_n_L + r * cos_omega1 * last_v1r - r * sin_omega1 * last_v1i;
        float v1i =      - rL * sin_omega1_L * s_n_L + r * cos_omega1 * last_v1i + r * sin_omega1 * last_v1r;

        last_v0r = v0r;
        last_v0i = v0i;
        last_v1r = v1r;
        last_v1i = v1i;

        float energy_mark = v0r*v0r + v0i*v0i;
        float energy_space = v1r*v1r + v1i*v1i;
        float energy_diff = energy_space - energy_mark;

        // Filtro IIR passa-baixas
        float x0 = energy_diff;
        float y0 = b[0]*x0 + b[1]*x1 + b[2]*x2 - a[1]*y1 - a[2]*y2;

        // Atualizar estados do filtro
        x2 = x1;
        x1 = x0;
        y2 = y1;
        y1 = y0;

        // Correção crucial: inverter a decisão dos bits!
        if ((energy_mark + energy_space) < threshold) {
            digital_samples[i] = 1;  // Sem portadora - linha ociosa
        } else {
            // Tom de espaço (0) deve ter energia MAIOR que tom de marca (1)
            digital_samples[i] = (y0 > 0) ? 0 : 1;  // 0 = espaço, 1 = marca
        }
    }

    get_digital_samples(digital_samples, n);
}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n)
{
    while (n--) {
        *out_analog_samples++ = sin(phase);
        phase += (*in_digital_samples++ ? omega_mark : omega_space) * T;
        phase = fmod(phase, 2*std::numbers::pi);
    }
}
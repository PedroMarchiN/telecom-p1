#include <cmath>
#include <vector>
#include "v21.hpp"

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    // Limpa e reserva espaço para os bits demodulados
    digital_samples.clear();
    digital_samples.reserve(n);

    for (unsigned int i = 0; i < n; ++i) {
        float x = in_analog_samples[i];

        // Aplica os dois filtros passa-banda
        float y_mark  = bp_mark.process(x);   // Filtro para 1850 Hz
        float y_space = bp_space.process(x);  // Filtro para 1650 Hz

        // Calcula a diferença entre as energias e passa por filtro passa-baixa
        float y_diff = lp_diff.process(y_space - y_mark);

        // Detecta ausência de portadora (sinal fraco em ambas bandas)
        bool no_carrier = (std::fabs(y_mark) + std::fabs(y_space) < noise_floor);

        // Decide o bit com base na energia relativa
        unsigned int bit = no_carrier ? 1u : (y_diff > threshold ? 0u : 1u);

        // Armazena bit
        digital_samples.push_back(bit);
    }

    // Entrega os bits para a UART (ou camada superior)
    get_digital_samples(digital_samples.data(), digital_samples.size());
}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n)
{
    while (n--) {
        *out_analog_samples++ = sin(phase);
        phase += (*in_digital_samples++ ? omega_mark : omega_space) * SAMPLING_PERIOD;

        // evita que phase cresça indefinidamente, o que causaria perda de precisão
        phase = remainder(phase, 2*std::numbers::pi);
    }
}

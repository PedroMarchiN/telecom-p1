#include "uart.hpp"

#include "uart.hpp"
#include <deque>

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
        unsigned int sample = buffer[i];
        
        // Atualiza a janela deslizante
        samples.push_front(sample);
        
        // Atualiza contador de bits baixos
        if (sample == 0) low_bit_counter++;
        if (samples.size() > 93) {
            if (samples.back() == 0) low_bit_counter--;
            samples.pop_back();
        }

        switch (state) {
            case IDLE:
                if (low_bit_counter >= 25 && samples[46] == 0) {
                    // Detectado start bit válido
                    cycles_counter = 15;  // Ponto de amostragem do 1º bit
                    byte = 0;
                    bits_read = 0;
                    state = DATA_BIT;
                }
                break;
                
            case DATA_BIT:
                cycles_counter++;
                if (cycles_counter == 160) {
                    // Amostra o bit no momento correto
                    byte |= (samples[0] & 1) << bits_read;
                    bits_read++;
                    cycles_counter = 0;
                    
                    if (bits_read == 8) {
                        state = STOP_BIT;
                    }
                }
                break;
                
            case STOP_BIT:
                cycles_counter++;
                if (cycles_counter == 160) {
                    // Final do stop bit
                    get_byte(byte);
                    state = IDLE;
                }
                break;
        }
    }
}



void UART_TX::put_byte(uint8_t byte)
{
    samples_mutex.lock();
    put_bit(0);  // start bit
    for (int i = 0; i < 8; i++) {
        put_bit(byte & 1);
        byte >>= 1;
    }
    put_bit(1);  // stop bit
    samples_mutex.unlock();
}

void UART_TX::get_samples(unsigned int *buffer, unsigned int n)
{
    samples_mutex.lock();
    std::vector<unsigned int>::size_type i = 0;
    while (!samples.empty() && i < n) {
        buffer[i++] = samples.front();
        samples.pop_front();
    }
    samples_mutex.unlock();

    while (i < n) {
        // idle
        buffer[i++] = 1;
    }
}

void UART_TX::put_bit(unsigned int bit)
{
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++) {
        samples.push_back(bit);
    }
}

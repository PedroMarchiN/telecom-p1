#include "uart.hpp"

#include "uart.hpp"
#include <deque>

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n)
{
    enum State { IDLE, RECEIVING };
    static State state = IDLE;

    static std::deque<unsigned int> window;
    static int sample_index = 0;
    static int bit_index = 0;
    static uint8_t current_byte = 0;
    static int wait_for = 0;

    for (unsigned int i = 0; i < n; ++i) {
        unsigned int sample = buffer[i];

        if (state == IDLE) {
            // Preencher a janela com as últimas 30 amostras
            window.push_back(sample);
            if (window.size() > 30)
                window.pop_front();

            if (sample == 0 && window.size() == 30) {
                int low_count = 0;
                for (auto s : window)
                    if (s == 0) low_count++;

                if (low_count >= 25) {
                    state = RECEIVING;
                    sample_index = 0;
                    bit_index = 0;
                    current_byte = 0;
                    wait_for = 50+160;
                    window.clear();
                }
            }
        }
        else if (state == RECEIVING) {
            sample_index++;

            if (sample_index == wait_for) {
                if (bit_index < 8) {
                    current_byte |= (sample & 1) << bit_index;
                    bit_index++;
                    wait_for += 160;
                }
                else {
                    get_byte(current_byte);
                    state = IDLE;
                    window.clear();
                }
            }
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

#include "uart.hpp"
#include <deque>

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n)
{
    
    static int clockCounter = 0;
    static int lowCounter = 0;
    static int bitsCounter = 0;

    for (int i = 0; i < n; i++) {
        this->samples.push_front(buffer[i]);
        if (this->samples[0] == 0)
            lowCounter++;
        if (this->samples[30] == 0)
            lowCounter--;  

        switch (state) {
            case IDLE:
                if (lowCounter >= 25 && this->samples[96] == 0) {
                    clockCounter = 15; // after midbit (79 out of 160)
                    this->byte = 0;
                    bitsCounter = 0;
                    state = DATA_BIT;
                }
                break;

            case DATA_BIT:
                if (clockCounter == 159) {
                    this->byte += this->samples[0] << bitsCounter;
                    bitsCounter++;
                    clockCounter = 0;
                    if (bitsCounter == 8) 
                        state = STOP_BIT;
                } else
                    clockCounter++; 
                break;

            case STOP_BIT:
                if (clockCounter == 159) {
                    this->get_byte(this->byte);
                    state = IDLE;
                } else
                    clockCounter++;
                break;

            default: break;
        }

        this->samples.pop_back();
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

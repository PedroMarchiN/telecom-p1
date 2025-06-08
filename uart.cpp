#include "uart.hpp"

#include "uart.hpp"
#include <deque>

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n)
{
    int lowCounter = 0;

    for (int i = 0; i < n; i++) {
        this->samples.push_front(buffer[i]);
        if (this->samples[0] == 0)
            lowCounter++;
        if (this->samples[30] == 0)
            lowCounter--;  

        switch (state) {
            case IDLE:
                if (lowCounter >= 25 && this->samples[96] == 0) {
                    this->clockCounter = 15; // after midbit (79 out of 160)
                    this->byte = 0;
                    this->bitsCounter = 0;
                    this->state = DATA_BIT;
                }

                break;

            case DATA_BIT:
                if (this->clockCounter == 159) {
                    this->byte += this->samples[0] << this->bitsCounter;
                    this->bitsCounter++;
                    this->clockCounter = 0;
                    if (this->bitsCounter == 8) 
                        this->state = STOP_BIT;
                } else
                    this->clockCounter++; 

                break;

            case STOP_BIT:
                if (this->clockCounter == 159) {
                    this->get_byte(this->byte);
                    lowCounter = 0;
                    this->state = IDLE;
                } else
                    this->clockCounter++;
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

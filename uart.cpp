#include "uart.hpp"

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n)
{
    for (unsigned int i = 0; i < n; i++) {
        this->samples.push_front(buffer[i]);
        if (this->samples[0] == 0)
            this->low_bit_counter++;
        if (this->samples[30] == 0)
            this->low_bit_counter--; // low bit leaving the window 

        switch (state) {
            case IDLE:
                if (low_bit_counter >= 25 && this->samples[93] == 0) {
                    // This is a start bit!
                    this->cycles_counter = 15; // after midbit (79 out of 160)
                    this->byte = 0;
                    this->bits_read = 0;
                    this->state = DATA_BIT;
                }

                break;

            case DATA_BIT:
                if (this->cycles_counter == 159) {
                    this->byte += this->samples[0] << this->bits_read;
                    this->bits_read++;
                    this->cycles_counter = 0;
                    if (this->bits_read == 8) 
                        this->state = STOP_BIT;
                } else
                    this->cycles_counter++; 

                break;

            case STOP_BIT:
                if (this->cycles_counter == 159) {
                    this->get_byte(this->byte);
                    this->state = IDLE;
                } else
                    this->cycles_counter++;
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
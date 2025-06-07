#ifndef UART_HPP
#define UART_HPP

#include <functional>
#include <deque>
#include <mutex>
#include <stdint.h>
#include "config.hpp"

class UART_RX {
public:
    UART_RX(std::function<void(uint8_t)> get_byte) : 
        get_byte(get_byte), 
        byte(0),
        cycles_counter(0),
        low_bit_counter(0),
        bits_read(0),
        state(IDLE) {
            // Inicializa com 93 amostras de valor 1 (estado idle)
            for (int i = 0; i < 93; i++)
                samples.push_back(1);
        }
    
    void put_samples(const unsigned int *buffer, unsigned int n);
    
private:
    std::function<void(uint8_t)> get_byte;
    uint8_t byte;
    unsigned int cycles_counter;
    unsigned int low_bit_counter;
    unsigned int bits_read;
    std::deque<unsigned int> samples;
    
    enum State {
        IDLE,
        DATA_BIT,
        STOP_BIT
    } state;
};

class UART_TX
{
public:
    void put_byte(uint8_t byte);
    void get_samples(unsigned int *buffer, unsigned int n);
private:
    std::deque<unsigned int> samples;
    std::mutex samples_mutex;
    void put_bit(unsigned int bit);
};

#endif

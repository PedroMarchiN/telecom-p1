#ifndef UART_HPP
#define UART_HPP

#include <functional>
#include <deque>
#include <mutex>
#include <stdint.h>
#include "config.hpp"

class UART_RX {
public:
    UART_RX(std::function<void(uint8_t)> get_byte);
    void put_samples(const unsigned int *buffer, unsigned int n);

private:
    std::function<void(uint8_t)> get_byte;
    
    // Estado da máquina de estados
    enum State { IDLE, RECEIVING } state;
    
    // Contadores e buffers por instância
    std::deque<unsigned int> window;
    int sample_index;
    int bit_index;
    uint8_t current_byte;
    int wait_for;
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
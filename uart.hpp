#ifndef UART_HPP
#define UART_HPP

#include <functional>
#include <deque>
#include <mutex>
#include <stdint.h>
#include "config.hpp"

class UART_RX
{
public:
    //fuction to get bytes changed
    UART_RX(std::function<void(uint8_t)> get_byte) :
        // initializing values as 0
        get_byte(get_byte), 
        byte(0),
        lowCounter(0),
        clockCounter(0),
        bitsCounter(0),
        
        //puts in indle state
        state(IDLE) {
            for (int i = 0; i < 96; i++)
                this->samples.push_front(1);
        }
    void put_samples(const unsigned int *buffer, unsigned int n);

private:
    std::function<void(uint8_t)> get_byte;

    uint8_t byte; //information received

    //counters
    int clockCounter; 
    int lowCounter;
    int bitsCounter;

    //double ended queue
    std::deque<unsigned int> samples;
    enum {
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
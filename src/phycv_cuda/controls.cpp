// controls.cpp

#include "controls.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

#define CHIP_I2C_ADDR 0x0C
#define BUS "/dev/i2c-1"

using namespace std; 

const std::unordered_map<uint8_t, OptionInfo> Controller::m_opts = {
    {OPT_FOCUS, {0x01, 0, 20000, 0x0B}},
    {OPT_ZOOM, {0x00, 3000, 20000, 0x0A}},
    {OPT_MOTOR_X, {0x05, 0, 180, 0}},
    {OPT_MOTOR_Y, {0x06, 0, 180, 0}},
    {OPT_IRCUT, {0x0C, 0x00, 0x01, 0}},
};

Controller::Controller() 
: m_bus(0)
{
    m_bus = open(BUS, O_RDWR); 
    if (m_bus < 0) {
        cerr << "Error opening the bus." << endl; 
        _exit(1); 
    }
    if (ioctl(m_bus, I2C_SLAVE, CHIP_I2C_ADDR) < 0) {
        cerr << "Error setting I2C address." << endl; 
        _exit(1); 
    }
}

Controller::~Controller()
{
    close(m_bus); 
}

uint16_t Controller::read_word(uint8_t chip_addr, uint8_t reg_addr)
{
    uint16_t value; 

    // Write the register address to the I2C bus
    if (write(m_bus, &reg_addr, sizeof(reg_addr)) != sizeof(reg_addr)) {
        cerr << "Error writing register address to I2C." << endl; 
        _exit(1); 
    }

    // Read the 16-bit value from the specified register
    if (read(m_bus, &value, sizeof(value)) != sizeof(value)) {
        cerr << "Error reading from I2C." << endl; 
        _exit(1); 
    }

    return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8);
}

void Controller::write_word(uint8_t chip_addr, uint8_t reg_addr, uint16_t value)
{
    // Convert the 16-bit value to little-endian format
    uint16_t data = ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8); 

    // Prepare the data buffer with the correct byte order
    uint8_t buffer[3] = {reg_addr, static_cast<uint8_t>(data & 0x00FF), static_cast<uint8_t>((data & 0xFF00) >> 8)};

    // Write the register address and data to the I2C bus
    if (write(m_bus, buffer, sizeof(buffer)) != sizeof(buffer)) {
        cerr << "Error writing to I2C." << endl;
        _exit(1);
    }
}

bool Controller::is_busy() {
    return read_word(CHIP_I2C_ADDR, 0x04) != 0;
}

void Controller::waiting_for_free() {
    int count = 0;
    time_t begin = time(nullptr);
    while (is_busy() && count < (5 / 0.01)) {
        count++;
        usleep(10000);  // 0.01 seconds
    }
}

void Controller::reset(uint8_t opt, bool flag) {
    waiting_for_free();
    auto info = m_opts.find(opt);
    if (info == m_opts.end() || info->second.reset_addr == 0) {
        return;
    }
    write_word(CHIP_I2C_ADDR, info->second.reset_addr, 0x0000);
    set(opt, info->second.min_value);
    if (flag) {
        waiting_for_free();
    }
}

uint16_t Controller::get(uint8_t opt, bool flag) {
    //waiting_for_free();
    auto info = m_opts.find(opt);
    if (info != m_opts.end()) {
        return read_word(CHIP_I2C_ADDR, info->second.reg_addr);
    }
    return 0; // or some default value
}

void Controller::set(uint8_t opt, uint16_t value, bool flag) {
    //waiting_for_free();
    auto info = m_opts.find(opt);
    if (info != m_opts.end()) {
        if (value > info->second.max_value) {
            value = info->second.max_value;
        } else if (value < info->second.min_value) {
            value = info->second.min_value;
        }
        write_word(CHIP_I2C_ADDR, info->second.reg_addr, value);
        if (flag) {
            waiting_for_free();
        }
    }
}
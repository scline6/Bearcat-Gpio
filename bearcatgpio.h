#ifndef BEARCATGPIO_H
#define BEARCATGPIO_H

#include <cstdint>
#include <functional>
#include <ctime>
#include <chrono>

class BearcatGpio
{
public:
    static const int MAX_HEADER_PINS = 40;
    enum Board
    {
        BOARD_CUSTOM = 0,
        BOARD_RPI4B = 1,
        BOARD_RPI5 = 2,
    };
    enum Pull
    {
        PULL_NONE = 0,
        PULL_UP   = 1,
        PULL_DOWN = 2,
    };
    enum Edge
    {
        EDGE_RISING  = 0,
        EDGE_FALLING = 1,
        EDGE_BOTH    = 2,
    };
    enum SpiChipSelect
    {
        SPI_CE0 = MAX_HEADER_PINS + 1,
        SPI_CE1 = MAX_HEADER_PINS + 2,
        SPI_CE2 = MAX_HEADER_PINS + 3,
    };
    enum CommunicationProtocol
    {
        UNDEFINED_PROTOCOL = -1,
        UART_PROTOCOL = 1,
        I2C_PROTOCOL = 2,
        SPI_PROTOCOL = 3,
        GPIO_STEP_DIR_PROTOCOL = 9,
    };
    class I2cHandle
    {
    public:
        int fd;
        int bus;
        int address;
    public:
        static I2cHandle create() {return I2cHandle{-1, -1, -1,};}
    };
    class SpiHandle
    {
    public:
        int fd;
        int bus;
        int channel;
        int mosiPin;
        int misoPin;
        int sclkPin;
        int baud;
    public:
        static SpiHandle create() {return SpiHandle{-1, -1, -1, -1, -1, -1, -1,};}
    };
    class InterfaceHandle
    {
    public:
        CommunicationProtocol protocol = UNDEFINED_PROTOCOL;
        union { I2cHandle i2c; SpiHandle spi; } handle;
    };
    class PinInfo
    {
    public:
        int gpiochip;
        int line;
        const char *name;
    };
public:
    inline static void nsleep(int nsec)
    {
        auto start = std::chrono::steady_clock::now();
        std::int64_t targetNs = nsec;
        while (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count() < targetNs) {}
    }
    inline static void usleep(int usec)
    {
        auto start = std::chrono::steady_clock::now();
        std::int64_t targetNs = usec * 1000;
        while (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count() < targetNs) {}
    }
    static void msleep(int msec);
    static void sleep(int sec);
    static int initialize(Board board = BOARD_RPI4B);
    static int terminate();
    static int setupOutput(int pin);
    static int write(int pin, int level);
    static int setupInput(int pin, int pull);
    static int read(int pin);
    static int writeFast(int pPin, int level); // MMIO, no locking, no auto-setup, for stepper pulsing
    static int setupInputSignaler(int pin, int edge, int timeout_msec, bool debounce, std::function<void(int pin, int level, int tick_usec)> &callback);
    static int stopInputSignaler(int pin);
    static int startSoftPwm(int pin, int frequencyHz, float dutyCycle = 0.5f, int range = 255);
    static int stopSoftPwm(int pin);
    static int i2cOpen(int bus, int address);
    static int i2cClose(int fd);
    static int i2cWriteReg8(int fd, int i2cRegAddress, std::uint8_t value);
    static int i2cReadReg8(int fd, int i2cRegAddress);
    static int i2cWriteReg16(int fd, int i2cRegAddress, std::uint16_t value);
    static int i2cReadReg16(int fd, int i2cRegAddress);
    static int i2cWriteBlock(int fd, int i2cRegAddress, std::uint8_t *byteArray, int byteCount);
    static int i2cReadBlock(int fd, int i2cRegAddress, std::uint8_t *byteArray, int byteCount);
    static int spiOpen(int bus, int chan, int baud, int mode = 0, bool csActiveHigh = false, int bitsPerWord = 8, bool lsbFirst = false);
    static int spiClose(int fd);
    static int spiRead(int fd, std::uint8_t *byteArray, int byteCount);
    static int spiWrite(int fd, std::uint8_t *byteArray, int byteCount);
    static int setCustomPinMap(const PinInfo *entries);
    static bool fastMmioAvailable(); // return true is SoC with MMIO was found
public:
    BearcatGpio() = delete; // static-only utility class
    BearcatGpio(const BearcatGpio &) = delete;
    BearcatGpio &operator=(const BearcatGpio &) = delete;
};

#endif // BEARCATGPIO_H

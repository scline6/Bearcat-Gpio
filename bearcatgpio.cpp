#include "bearcatgpio.h"
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <iostream>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>

namespace {

constexpr std::uint32_t DEFAULT_DEBOUNCE_USEC = 10000; // 10ms

const std::array<BearcatGpio::PinInfo, BearcatGpio::MAX_HEADER_PINS + 1> PIN_TO_LINE_MAP_UNFILLED = {};

const std::array<BearcatGpio::PinInfo, BearcatGpio::MAX_HEADER_PINS + 1> PIN_TO_LINE_MAP_RPI4B =
    {{
      {-2,-2,nullptr}, {-1, -1, "3V3"}, {-1, -1, "5V"}, {0, 2,  "GPIO2/SDA1"},  {-1, -1, "5V"},
      {0, 3,  "GPIO3/SCL1"}, {-1, -1, "GND"}, {0, 4,  "GPIO4"}, {0, 14, "GPIO14/TXD0"}, {-1, -1, "GND"},
      {0, 15, "GPIO15/RXD0"}, {0, 17, "GPIO17"}, {0, 18, "GPIO18"}, {0, 27, "GPIO27"}, {-1, -1, "GND"},
      {0, 22, "GPIO22"}, {0, 23, "GPIO23"}, {-1, -1, "3V3"}, {0, 24, "GPIO24"}, {0, 10, "GPIO10/MOSI"},
      {-1, -1, "GND"}, {0, 9,  "GPIO9/MISO"}, {0, 25, "GPIO25"}, {0, 11, "GPIO11/SCLK"}, {0, 8,  "GPIO8/CE0"},
      {-1, -1, "GND"}, {0, 7,  "GPIO7/CE1"}, {0, 0, "GPIO0/ID_SD"}, {0, 1, "GPIO1/ID_SC"}, {0, 5, "GPIO5"},
      {-1, -1, "GND"}, {0, 6,  "GPIO6"}, {0, 12, "GPIO12"}, {0, 13, "GPIO13"}, {-1, -1, "GND"}, {0, 19, "GPIO19"},
      {0, 16, "GPIO16"}, {0, 26, "GPIO26"}, {0, 20, "GPIO20"}, {-1, -1, "GND"}, {0, 21, "GPIO21"},
      }};

constexpr int RPI5_HEADER_GPIOCHIP = 0; // fixme: confirm this, it could be 4
const std::array<BearcatGpio::PinInfo, BearcatGpio::MAX_HEADER_PINS + 1> PIN_TO_LINE_MAP_RPI5 =
    {{
      {-2,-2,nullptr}, {-1, -1, "3V3"}, {-1, -1, "5V"}, {RPI5_HEADER_GPIOCHIP, 2,  "GPIO2/SDA1"},  {-1, -1, "5V"},
      {RPI5_HEADER_GPIOCHIP, 3,  "GPIO3/SCL1"}, {-1, -1, "GND"}, {RPI5_HEADER_GPIOCHIP, 4,  "GPIO4"}, {RPI5_HEADER_GPIOCHIP, 14, "GPIO14/TXD0"}, {-1, -1, "GND"},
      {RPI5_HEADER_GPIOCHIP, 15, "GPIO15/RXD0"}, {RPI5_HEADER_GPIOCHIP, 17, "GPIO17"}, {RPI5_HEADER_GPIOCHIP, 18, "GPIO18"}, {RPI5_HEADER_GPIOCHIP, 27, "GPIO27"}, {-1, -1, "GND"},
      {RPI5_HEADER_GPIOCHIP, 22, "GPIO22"}, {RPI5_HEADER_GPIOCHIP, 23, "GPIO23"}, {-1, -1, "3V3"}, {RPI5_HEADER_GPIOCHIP, 24, "GPIO24"}, {RPI5_HEADER_GPIOCHIP, 10, "GPIO10/MOSI"},
      {-1, -1, "GND"}, {RPI5_HEADER_GPIOCHIP, 9,  "GPIO9/MISO"}, {RPI5_HEADER_GPIOCHIP, 25, "GPIO25"}, {RPI5_HEADER_GPIOCHIP, 11, "GPIO11/SCLK"}, {RPI5_HEADER_GPIOCHIP, 8,  "GPIO8/CE0"},
      {-1, -1, "GND"}, {RPI5_HEADER_GPIOCHIP, 7,  "GPIO7/CE1"}, {RPI5_HEADER_GPIOCHIP, 0, "GPIO0/ID_SD"}, {RPI5_HEADER_GPIOCHIP, 1, "GPIO1/ID_SC"}, {RPI5_HEADER_GPIOCHIP, 5, "GPIO5"},
      {-1, -1, "GND"}, {RPI5_HEADER_GPIOCHIP, 6,  "GPIO6"}, {RPI5_HEADER_GPIOCHIP, 12, "GPIO12"}, {RPI5_HEADER_GPIOCHIP, 13, "GPIO13"}, {-1, -1, "GND"}, {RPI5_HEADER_GPIOCHIP, 19, "GPIO19"},
      {RPI5_HEADER_GPIOCHIP, 16, "GPIO16"}, {RPI5_HEADER_GPIOCHIP, 26, "GPIO26"}, {RPI5_HEADER_GPIOCHIP, 20, "GPIO20"}, {-1, -1, "GND"}, {RPI5_HEADER_GPIOCHIP, 21, "GPIO21"},
      }};

enum class SocKind { NONE, BCM2711, BCM2712RP1 };

class Bcm2711
{
public:
    Bcm2711() = delete; // static-only; never instantiated
    static bool probe()
    {
        FILE *f = fopen("/proc/cpuinfo", "r");
        if (!f) return false;
        char line[256];
        bool match = false;
        while (fgets(line, sizeof(line), f)) {
            if (strstr(line, "BCM2711") || strstr(line, "Raspberry Pi 4")) { match = true; break; }
        }
        fclose(f);
        return match;
    }
    static int mapRegisters(volatile std::uint32_t *&regs, int &memFd)
    {
        memFd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC);
        off_t mapOff = 0;
        if (memFd < 0) {
            memFd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
            if (memFd < 0) return -1;
            mapOff = static_cast<off_t>(GPIO_BASE);
        }
        std::size_t maplen = (GPIO_LEN * 4 > 4096 ? GPIO_LEN * 4 : 4096);
        void *m = mmap(nullptr, maplen, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, mapOff);
        if (m == MAP_FAILED) { close(memFd); memFd = -1; return -2; }
        regs = static_cast<volatile std::uint32_t *>(m);
        return 0;
    }
    static void unmapRegisters(volatile std::uint32_t *&regs, int &memFd) // note: only called a program exit, never terminate
    {
        if (regs != nullptr) { munmap((void*)regs, 4096); regs = nullptr; }
        if (memFd >= 0) { close(memFd); memFd = -1; }
    }
    static int set(volatile std::uint32_t *regs, int gpiochip, int line, int value)
    {
        if (gpiochip != 0) return -1;
        if (regs == nullptr) return -2;
        if ((line < 0) || (line > 57)) return -3;
        int reg = line / 32;
        std::uint32_t bit = (1u << (line % 32));
        regs[(value ? GPSET0 : GPCLR0) + reg] = bit;
        return 0;
    }
    static int get(volatile std::uint32_t *regs, int gpiochip, int line)
    {
        if (gpiochip != 0) return -1;
        if (regs == nullptr) return -2;
        if ((line < 0) || (line > 57)) return -3;
        int reg = line / 32;
        std::uint32_t bit = 1u << (line % 32);
        return ((regs[GPLEV0 + reg] & bit) ? 1 : 0);
    }
    static int setup(volatile std::uint32_t *regs, int gpiochip, int line, bool output)
    {
        if (gpiochip != 0) return -1;
        if (regs == nullptr) return -2;
        if ((line < 0) || (line > 57)) return -3;
        int reg = line / 10;
        int shift = (line % 10) * 3;
        std::uint32_t val = regs[GPFSEL0 + reg];
        val &= ~(0x7u << shift);
        if (output == true) val |= (0x1u << shift);
        regs[GPFSEL0 + reg] = val;
        return 0;
    }
private:
    static constexpr std::uint32_t GPIO_BASE = 0xFE200000u;
    static constexpr std::size_t GPIO_LEN = 0xB4; // 180
    enum { GPFSEL0 = 0, GPSET0 = 7, GPCLR0 = 10, GPLEV0 = 13 };
};

class Bcm2712RP1
{
public:
    Bcm2712RP1() = delete; // static-only; never instantiated
    static bool probe()
    {
        FILE *f = fopen("/proc/cpuinfo", "r");
        if (!f) return false;
        char line[256];
        bool match = false;
        while (fgets(line, sizeof(line), f)) {
            if (strstr(line, "BCM2712") || strstr(line, "Raspberry Pi 5")) { match = true; break; }
        }
        fclose(f);
        return match;
    }
    static int mapRegisters(volatile uint32_t *&regs, int &memFd)
    {
        memFd = open("/dev/gpiomem0", O_RDWR | O_SYNC | O_CLOEXEC);
        if (memFd < 0) return -1; // note: no /dev/mem fallback -- RP1 is PCIe-attached, not a fixed physical address
        void *m = mmap(nullptr, MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, 0);
        if (m == MAP_FAILED) { close(memFd); memFd = -1; return -2; }
        regs = static_cast<volatile uint32_t *>(m);
        return 0;
    }
    static void unmapRegisters(volatile uint32_t *&regs, int &memFd)
    {
        if (regs) { munmap((void *)regs, MEM_SIZE); regs = nullptr; }
        if (memFd >= 0) { close(memFd); memFd = -1; }
    }
    static int set(volatile uint32_t *regs, int gpiochip, int line, int value)
    {
        (void)gpiochip;
        if (!regs) return -2;
        if ((line < 0) || (line > 27)) return -3; // RP1 IO_BANK0 covers GPIO0-27 on the 40-pin header
        size_t offset = (SYS_RIO0_OFFSET + RIO_OUT + (value ? SET_OFFSET : CLR_OFFSET)) / 4;
        regs[offset] = (1u << line);
        return 0;
    }
    static int get(volatile uint32_t *regs, int gpiochip, int line)
    {
        (void)gpiochip;
        if (!regs) return -2;
        if ((line < 0) || (line > 27)) return -3;
        size_t offset = (SYS_RIO0_OFFSET + RIO_IN) / 4;
        return ((regs[offset] >> line) & 1u) ? 1 : 0;
    }
    static int setup(volatile uint32_t *regs, int gpiochip, int line, bool output)
    {
        (void)gpiochip;
        if (!regs) return -2;
        if ((line < 0) || (line > 27)) return -3;
        size_t padSetOffset = (PADS_BANK0_OFFSET + PADS_GPIO + (static_cast<size_t>(line) * PADS_OFFSET) + SET_OFFSET) / 4;
        size_t padClrOffset = (PADS_BANK0_OFFSET + PADS_GPIO + (static_cast<size_t>(line) * PADS_OFFSET) + CLR_OFFSET) / 4;
        regs[padClrOffset] = PADS_OUT_DISABLE_MASK; // clear "output disable" -> output enabled
        regs[padSetOffset] = PADS_IN_ENABLE_MASK;   // set "input enable"
        size_t ctrlOffset = (IO_BANK0_OFFSET + GPIO_CTRL + (static_cast<size_t>(line) * GPIO_OFFSET) + RW_OFFSET) / 4;
        uint32_t ctrl = regs[ctrlOffset];
        ctrl &= ~CTRL_OUTOVER_MASK;
        ctrl &= ~CTRL_OEOVER_MASK;
        ctrl &= ~CTRL_FUNCSEL_MASK;
        ctrl |= (FSEL_GPIO << CTRL_FUNCSEL_LSB);
        regs[ctrlOffset] = ctrl;
        size_t oeOffset = (SYS_RIO0_OFFSET + RIO_OE + (output ? SET_OFFSET : CLR_OFFSET)) / 4;
        regs[oeOffset] = (1u << line);
        return 0;
    }
private:
    static constexpr size_t MEM_SIZE = 0x30000;
    static constexpr size_t IO_BANK0_OFFSET = 0x00000;
    static constexpr size_t SYS_RIO0_OFFSET = 0x10000;
    static constexpr size_t PADS_BANK0_OFFSET = 0x20000;
    static constexpr size_t RW_OFFSET = 0x0000;  // datasheet @ 2.4: atomic register access, 4 "views" per register
    static constexpr size_t SET_OFFSET = 0x2000;
    static constexpr size_t CLR_OFFSET = 0x3000;
    static constexpr size_t GPIO_CTRL = 0x0004;  // datasheet @ 3.1.4
    static constexpr size_t GPIO_OFFSET = 8;     // per-pin stride within IO_BANK0
    static constexpr uint32_t CTRL_FUNCSEL_MASK = 0x001f;
    static constexpr uint32_t CTRL_FUNCSEL_LSB = 0;
    static constexpr uint32_t CTRL_OUTOVER_MASK = 0x3000;
    static constexpr uint32_t CTRL_OEOVER_MASK = 0xc000;
    static constexpr uint32_t FSEL_GPIO = 5; // FSEL_ALT5 -- rppal's own comment marks this "// GPIO"
    static constexpr size_t PADS_GPIO = 0x04;
    static constexpr size_t PADS_OFFSET = 4; // per-pin stride within PADS_BANK0
    static constexpr uint32_t PADS_IN_ENABLE_MASK = 0x40;
    static constexpr uint32_t PADS_OUT_DISABLE_MASK = 0x80;
    static constexpr size_t RIO_OUT = 0x00;
    static constexpr size_t RIO_OE = 0x04;
    static constexpr size_t RIO_IN = 0x08;
};

class PinHandle
{
public:
    int chip = -1;
    int line = -1;
    int fd = -1; // note: linux file descriptor for the currently-claimed request, -1 if none
    std::atomic<bool> mmioConfigured{false};
    std::atomic<bool> isOutput{false};
    int armedEdge = -1; // note: fd's edge flags is actually configured with, -1 = none
    bool armedDebounce = false; // note: fd's debounce attribute
    std::atomic<double> pwmFreq_Hz{60.0};
    std::atomic<double> pwmDutyFrac{0.5}; // note: 0.0-1.0
    std::atomic<bool> pwmRunning{false};
    std::atomic<bool> pwmStopping{false}; // note: claims exclusive right to join this pin's PWM thread
    std::thread pwmThread;
    bool spiCsActiveHigh = false;
    int spiCsFd = -1;
    std::function<void(int pin, int level, int tick_usec)> signalerCallback;
    std::atomic<bool> signalerRunning{false};
    std::atomic<bool> signalerStopping{false}; // note: claims exclusive right to join this pin's signaler thread
    std::thread signalerThread;
};

class BearcatGpioInternals
{
public:
    static BearcatGpioInternals &instance() { static BearcatGpioInternals instance_; return instance_; }
    BearcatGpioInternals(const BearcatGpioInternals &) = delete;
    BearcatGpioInternals &operator=(const BearcatGpioInternals &) = delete;
public:
    int initialize(BearcatGpio::Board board)
    {
        m_board = board;
        m_initialized = false;
        const std::array<BearcatGpio::PinInfo, BearcatGpio::MAX_HEADER_PINS + 1> *srcPinInfo = nullptr;
        switch (board) {
        case BearcatGpio::BOARD_RPI4B: srcPinInfo = &PIN_TO_LINE_MAP_RPI4B; break;
        case BearcatGpio::BOARD_RPI5: srcPinInfo = &PIN_TO_LINE_MAP_RPI5; break;
            fprintf(stderr,
                    "BearcatGpio WARNING: pin map for this board is unfilled placeholders.\n"
                    "                     Call setCustomPinMap() with a  table.\n"
                    "                     Use gpioinfo and board pinout diagram first.\n");
            srcPinInfo = &PIN_TO_LINE_MAP_UNFILLED;
            break;
        case BearcatGpio::BOARD_CUSTOM:
            srcPinInfo = &PIN_TO_LINE_MAP_UNFILLED; // note: must call setCustomPinMap()
            break;
        default:
            errno = EINVAL;
            return -1;
        }
        m_pinInfo = *srcPinInfo; // plain std::array copy-assignment; compiles to the same memcpy either way
        resetAllPins(); // note: closes any leftover fds and stops any leftover PWM threads from a prior init
        if ((m_socKind == SocKind::NONE) && (m_registers == nullptr)) // note: detect and map the mmio backend only once
        {
            SocKind detectedSoc = detectSoc();
            if (detectedSoc != SocKind::NONE && mmioMap(detectedSoc) == 0) m_socKind = detectedSoc;
        }
        if (m_socKind != SocKind::NONE) fprintf(stderr, "BearcatGpio mmio backend: %s\n", socKindName(m_socKind));
        else fprintf(stderr, "BearcatGpio no mmio backend for this SoC; write()/read()/setupOutput()/setupInput() fall back to ioctl.\n");
        m_initialized = true;
        return 0;
    }
    int terminate() // note: idempotent
    {
        resetAllPins();
        m_initialized = false;
        return 0;
    }
    int setupOutput(int pin)
    {
        const BearcatGpio::PinInfo *p = lookupPinInfo(pin);
        if (p == nullptr) return -1;
        std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
        PinHandle &h = m_pins[pin];
        int fd = requestLine(h, p->gpiochip, p->line, GPIO_V2_LINE_FLAG_OUTPUT, 0);
        if (fd < 0) return -2; // note: unlock pin mutex
        if (m_socKind != SocKind::NONE) mmioSetup(p->gpiochip, p->line, true, h); // no lock: m_socKind/m_registers never change after initialize()
        return 0; // note: unlock pin mutex
    }
    int write(int pin, int level)
    {
        const BearcatGpio::PinInfo *p = lookupPinInfo(pin);
        if (p == nullptr) return -1;
        PinHandle &h = m_pins[pin];
        if (m_socKind != SocKind::NONE)
        {
            if ((h.mmioConfigured.load() == false) || (h.isOutput.load() == false)) mmioSetup(p->gpiochip, p->line, true, h);
            int status = mmioSet(p->gpiochip, p->line, level);
            return status;
        }
        std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
        if ((h.fd < 0) || (h.isOutput.load() == false))
        {
            int fd = requestLine(h, p->gpiochip, p->line, GPIO_V2_LINE_FLAG_OUTPUT, level); // note: setup and write in one shot
            return (fd >= 0 ? 0 : -2); // note: unlock pin mutex
        }
        gpio_v2_line_values vals{};
        vals.mask = 1u;
        vals.bits = (level ? 1u : 0u);
        int status = ioctl(h.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
        return status; // note: unlock pin mutex
    }
    int writeFast(int pin, int level)
    {
        const BearcatGpio::PinInfo *p = lookupPinInfo(pin);
        if (p == nullptr) return -1;
        if (m_socKind == SocKind::NONE) { errno = ENOSYS; return -2; } // note: no mmio backend nor ioctl fallback
        int status = mmioSet(p->gpiochip, p->line, level);
        return status;
    }
    int setupInput(int pin, int pull)
    {
        const BearcatGpio::PinInfo *p = lookupPinInfo(pin);
        if (p == nullptr) return -1;
        std::uint64_t flags = GPIO_V2_LINE_FLAG_INPUT;
        switch (pull) {
        case BearcatGpio::PULL_UP: flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP; break;
        case BearcatGpio::PULL_DOWN: flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN; break;
        default: flags |= GPIO_V2_LINE_FLAG_BIAS_DISABLED; break;
        }
        std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
        PinHandle &h = m_pins[pin];
        int fd = requestLine(h, p->gpiochip, p->line, flags, 0);
        if (fd < 0) return fd; // note: unlock pin mutex
        if (m_socKind != SocKind::NONE) mmioSetup(p->gpiochip, p->line, false, h);
        return 0; // note: unlock pin mutex
    }
    int read(int pin)
    {
        const BearcatGpio::PinInfo *p = lookupPinInfo(pin);
        if (p == nullptr) return -1;
        PinHandle &h = m_pins[pin];
        if (m_socKind != SocKind::NONE)
        {
            if ((h.mmioConfigured.load() == false) || (h.isOutput.load() == true)) mmioSetup(p->gpiochip, p->line, false, h);
            int value = mmioGet(p->gpiochip, p->line);
            return value;
        }
        std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
        if ((h.fd < 0) || (h.isOutput.load() == true))
        {
            int fd = requestLine(h, p->gpiochip, p->line, GPIO_V2_LINE_FLAG_INPUT, 0); // note: set to read if not already
            if (fd < 0) return -2; // note: unlock pin mutex
        }
        gpio_v2_line_values vals{};
        vals.mask = 1u;
        int status = ioctl(h.fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals);
        if (status < 0) return -3;
        int value = static_cast<int>(vals.bits & 1u);
        return value; // note: unlock pin mutex
    }
    int setupInputSignaler(int pin, int edge, int timeout_msec, bool debounce, std::function<void(int pin, int level, int tick_usec)> &callback)
    {
        const BearcatGpio::PinInfo *p = lookupPinInfo(pin);
        if (p == nullptr) return -1;
        std::uint64_t flags = GPIO_V2_LINE_FLAG_INPUT;
        if ((edge == BearcatGpio::EDGE_RISING) || (edge == BearcatGpio::EDGE_BOTH)) flags |= GPIO_V2_LINE_FLAG_EDGE_RISING;
        if ((edge == BearcatGpio::EDGE_FALLING) || (edge == BearcatGpio::EDGE_BOTH)) flags |= GPIO_V2_LINE_FLAG_EDGE_FALLING;
        std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
        PinHandle &h = m_pins[pin];
        if (h.signalerThread.joinable()) { errno = EBUSY; return -2; } // one signaler per pin at a time -- unlock pin mutex
        bool needsRearm = ((h.fd < 0) || (h.isOutput.load() == true) || (h.armedEdge != edge) || (h.armedDebounce != debounce));
        int status = requestLine(h, p->gpiochip, p->line, flags, 0, debounce);
        if ((needsRearm == true) && (status < 0)) return -3; // note: unlock pin mutex
        h.signalerCallback = callback; // copy into pin-owned storage -- the thread below must not depend on the caller's own reference outliving this call
        h.signalerRunning = true;
        h.signalerStopping = false; // note: clear any leftover claim from a fully-joined previous run on this pin
        int fd = h.fd; // note: copy while still locked, same as the old design did
        h.signalerThread = std::thread([pin, fd, timeout_msec, &h]()
        {
            while (h.signalerRunning.load())
            {
                struct pollfd pfd{};
                pfd.fd = fd;
                pfd.events = POLLIN;
                int pr = poll(&pfd, 1, timeout_msec);
                if (pr < 0) break; // note: errno set by poll(), fd likely closed/invalidated, stop the thread
                if (pr == 0) continue; // note: timeout this iteration and keep watching
                if (!(pfd.revents & POLLIN)) continue;
                gpio_v2_line_event ev{};
                ssize_t n = ::read(fd, &ev, sizeof(ev));
                if (n != static_cast<ssize_t>(sizeof(ev))) break; // read failed -- fd likely closed, stop the thread
                if (h.signalerCallback != nullptr)
                {
                    int levelOnceTriggered = (ev.id == GPIO_V2_LINE_EVENT_RISING_EDGE) ? 1 : 0;
                    struct timespec now{};
                    clock_gettime(CLOCK_MONOTONIC, &now);
                    std::int64_t nowNs = static_cast<std::int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
                    int tick_usec = static_cast<int>((nowNs - static_cast<std::int64_t>(ev.timestamp_ns)) / 1000);
                    h.signalerCallback(pin, levelOnceTriggered, tick_usec);
                }
            }
        });
        return 0; // note: unlock pin mutex
    }
    int stopInputSignaler(int pin)
    {
        if ((pin < 1) || (pin > BearcatGpio::MAX_HEADER_PINS)) { errno = EINVAL; return -1; }
        switch (claimSignalerStop(pin)) {
        case ClaimResult::NOT_RUNNING: errno = ENOENT; return -2;
        case ClaimResult::ALREADY_CLAIMED: errno = ENOENT; return -3; // note: another stopInputSignaler() call owns this join
        case ClaimResult::CLAIMED: break;
        }
        PinHandle &h = m_pins[pin];
        h.signalerThread.join(); // note: can block up to timeout_msec, or indefinitely if timeout_msec < 0
        h.signalerStopping = false;
        return 0;
    }
    int startSoftPwm(int pin, int frequencyHz, float dutyCycle, int range)
    {
        const BearcatGpio::PinInfo *p = lookupPinInfo(pin);
        if (p == nullptr) return -1;
        if (frequencyHz <= 0) { errno = EINVAL; return -2; }
        int status = setupOutput(pin);
        if (status < 0) return -3;
        if (range < 1) range = 1;
        float steps = dutyCycle * static_cast<float>(range); // note: quantize duty into range discrete steps like wiringPi
        if (steps < 0) steps = 0;
        if (steps > range) steps = static_cast<float>(range);
        double pwmDutyFrac = static_cast<double>(steps) / static_cast<double>(range);
        std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
        PinHandle &h = m_pins[pin];
        if (h.pwmThread.joinable() == true)
        {
            if (h.pwmStopping.load() == true) { errno = EBUSY; return -4; } // note: unlock pin mutex
            h.pwmFreq_Hz = frequencyHz;
            h.pwmDutyFrac = dutyCycle;
            return 0;
        }
        h.pwmFreq_Hz = static_cast<double>(frequencyHz);
        h.pwmDutyFrac = pwmDutyFrac;
        h.pwmRunning = true;
        h.pwmStopping = false; // note: clear any leftover claim from a fully-joined previous run on this pin
        BearcatGpioInternals *self = this;
        h.pwmThread = std::thread([pin, self, &h]() {
            while (h.pwmRunning.load()) {
                double freq = h.pwmFreq_Hz;
                double duty = h.pwmDutyFrac;
                if (duty < 0) duty = 0;
                if (duty > 1) duty = 1;
                double periodNs = 1e9 / freq;
                long highNs = static_cast<long>(periodNs * duty);
                long lowNs = static_cast<long>(periodNs - highNs);
                if (highNs > 0) {
                    self->write(pin, 1);
                    struct timespec ts{ highNs / 1000000000L, highNs % 1000000000L };
                    nanosleep(&ts, nullptr);
                }
                if (h.pwmRunning.load() == false) break;
                if (lowNs > 0) {
                    self->write(pin, 0);
                    struct timespec ts{ lowNs / 1000000000L, lowNs % 1000000000L };
                    nanosleep(&ts, nullptr);
                }
            }
            self->write(pin, 0); // leave pin low on stop
        });
        return 0; // note: unlock pin mutex
    }
    int stopSoftPwm(int pin)
    {
        if ((pin < 1) || (pin > BearcatGpio::MAX_HEADER_PINS)) { errno = EINVAL; return -1; }
        ClaimResult claimResult = claimPwmStop(pin);
        switch (claimResult) {
        case ClaimResult::NOT_RUNNING: errno = ENOENT; return -2;
        case ClaimResult::ALREADY_CLAIMED: errno = ENOENT; return -3; // note: another stopSoftPwm() call owns this join
        case ClaimResult::CLAIMED: break;
        }
        PinHandle &h = m_pins[pin];
        h.pwmThread.join();
        h.pwmStopping = false;
        return 0;
    }
    static int i2cOpen(int bus, int address)
    {
        if (bus < 0) { errno = EINVAL; return -1; }
        if ((address < 0) || (address > 0x7F)) { errno = EINVAL; return -2; } // 7-bit address only
        char devpath[32];
        snprintf(devpath, sizeof(devpath), "/dev/i2c-%d", bus);
        int fd = open(devpath, O_RDWR);
        if (fd < 0) return -3;
        int status = ioctl(fd, I2C_SLAVE, address);
        if (status < 0) { int savedErrno = errno; close(fd); errno = savedErrno; return -4; }
        return fd;
    }
    static int i2cClose(int fd) { return close(fd); }
    static int i2cWriteByte(int fd, std::uint8_t value)
    {
        int status = i2cIoctl(fd, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, nullptr);
        return status;
    }
    static int i2cReadByte(int fd)
    {
        union i2c_smbus_data data{};
        int status = i2cIoctl(fd, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
        if (status < 0) return status;
        int value = static_cast<int>(data.byte);
        return value;
    }
    static int i2cWriteReg8(int fd, int reg, std::uint8_t value)
    {
        union i2c_smbus_data data{};
        data.byte = value;
        int status = i2cIoctl(fd, I2C_SMBUS_WRITE, static_cast<std::uint8_t>(reg), I2C_SMBUS_BYTE_DATA, &data);
        return status;
    }
    static int i2cReadReg8(int fd, int reg)
    {
        union i2c_smbus_data data{};
        int status = i2cIoctl(fd, I2C_SMBUS_READ, static_cast<std::uint8_t>(reg), I2C_SMBUS_BYTE_DATA, &data);
        if (status < 0) return status;
        int value = static_cast<int>(data.byte);
        return value;
    }
    static int i2cWriteReg16(int fd, int reg, std::uint16_t value)
    {
        union i2c_smbus_data data{};
        data.word = value;
        int status = i2cIoctl(fd, I2C_SMBUS_WRITE, static_cast<std::uint8_t>(reg), I2C_SMBUS_WORD_DATA, &data);
        return status;
    }
    static int i2cReadReg16(int fd, int reg)
    {
        union i2c_smbus_data data{};
        int status = i2cIoctl(fd, I2C_SMBUS_READ, static_cast<std::uint8_t>(reg), I2C_SMBUS_WORD_DATA, &data);
        if (status < 0) return status;
        int value = static_cast<int>(data.word);
        return value;
    }
    static int i2cWriteBlock(int fd, int reg, const char *byteArray, int byteCount)
    {
        if ((byteCount < 0) || (byteCount > I2C_SMBUS_BLOCK_MAX)) { errno = EINVAL; return -1; }
        union i2c_smbus_data data{};
        data.block[0] = static_cast<std::uint8_t>(byteCount);
        for (int i = 0; i < byteCount; i++) data.block[i + 1] = static_cast<std::uint8_t>(byteArray[i]);
        int status = i2cIoctl(fd, I2C_SMBUS_WRITE, static_cast<std::uint8_t>(reg), I2C_SMBUS_I2C_BLOCK_DATA, &data);
        return status;
    }
    static int i2cReadBlock(int fd, int reg, char *byteArray)
    {
        union i2c_smbus_data data{};
        data.block[0] = I2C_SMBUS_BLOCK_MAX;
        int status = i2cIoctl(fd, I2C_SMBUS_READ, static_cast<std::uint8_t>(reg), I2C_SMBUS_I2C_BLOCK_BROKEN, &data);
        if (status < 0) return status;
        for (int i = 1; i <= data.block[0]; i++) byteArray[i - 1] = static_cast<char>(data.block[i]);
        int value = static_cast<int>(data.block[0]);
        return value;
    }
    int spiTransfer(int fd, void *rxBuf, void *txBuf, int byteCount)
    {
        int csPin = -1;
        std::unique_lock<std::mutex> lock(m_manualCsLock);
        for (int i = 1; i <= BearcatGpio::MAX_HEADER_PINS; i++) if (m_pins[i].spiCsFd == fd) { csPin = i; break; } // fixme: better to avoid linear search
        if (csPin < 0) lock.unlock(); // note: kernel handles CS itself so no need to lock
        if (csPin >= 0) BearcatGpio::write(csPin, m_pins[csPin].spiCsActiveHigh ? 1 : 0); // note: pull CS down/up, still under lock
        struct spi_ioc_transfer args{};
        args.tx_buf = static_cast<__u64>(reinterpret_cast<std::uintptr_t>(txBuf));
        args.rx_buf = static_cast<__u64>(reinterpret_cast<std::uintptr_t>(rxBuf));
        args.len = static_cast<std::uint32_t>(byteCount);
        int status = ioctl(fd, SPI_IOC_MESSAGE(1), &args);
        if (csPin >= 0) BearcatGpio::write(csPin, m_pins[csPin].spiCsActiveHigh ? 0 : 1); // note: pull CS up/down, still under lock
        return status;
    }
    int spiOpen(int bus, int chan, int baud, int mode, bool csActiveHigh, int bitsPerWord, bool lsbFirst)
    {
        if (bus < 0) { errno = EINVAL; return -1; }
        if ((mode < 0) || (mode > 3)) { errno = EINVAL; return -2; }
        if (baud <= 0) { errno = EINVAL; return -3; }
        int devIndex = -1;
        int manualCsPin = -1;
        if (chan == BearcatGpio::SPI_CE0) devIndex = 0;
        else if (chan == BearcatGpio::SPI_CE1) devIndex = 1;
        else if (chan == BearcatGpio::SPI_CE2) devIndex = 2;
        else if ((chan >= 1) && (chan <= BearcatGpio::MAX_HEADER_PINS)) { devIndex = 0; manualCsPin = chan; } // '.0' reused purely for clock/MOSI/MISO; see header doc comment
        else { errno = EINVAL; return -4; }
        std::unique_lock<std::mutex> claimLock;
        if (manualCsPin >= 0)
        {
            claimLock = std::unique_lock<std::mutex>(m_manualCsLock);
            if (m_pins[manualCsPin].spiCsFd >= 0) { errno = EBUSY; return -5; }
        }
        char devpath[64];
        snprintf(devpath, sizeof(devpath), "/dev/spidev%d.%d", bus, devIndex);
        int fd = open(devpath, O_RDWR);
        if (fd < 0) return -6;
        std::uint8_t spiMode = static_cast<std::uint8_t>(mode == 1 ? SPI_MODE_1 : mode == 2 ? SPI_MODE_2 : mode == 3 ? SPI_MODE_3 : SPI_MODE_0);
        if ((csActiveHigh == true) && (manualCsPin < 0)) spiMode |= SPI_CS_HIGH; // note: csActiveHigh is only meaningful for kernel-managed CS
        if (lsbFirst == true) spiMode |= SPI_LSB_FIRST;
        int modeStatus = ioctl(fd, SPI_IOC_WR_MODE, &spiMode);
        std::uint8_t bpw = static_cast<std::uint8_t>(bitsPerWord);
        int bpwStatus = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw);
        std::uint32_t speed = static_cast<std::uint32_t>(baud);
        int speedStatus = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (modeStatus < 0 || bpwStatus < 0 || speedStatus < 0) {
            int savedErrno = errno;
            close(fd);
            errno = savedErrno;
            return -7;
        }
        if (manualCsPin >= 0)
        {
            if (BearcatGpio::setupOutput(manualCsPin) < 0) { close(fd); errno = ENXIO; return -8; }
            BearcatGpio::write(manualCsPin, csActiveHigh ? 0 : 1); // start deasserted
            m_pins[manualCsPin].spiCsFd = fd; // note: still holding claimLock
            m_pins[manualCsPin].spiCsActiveHigh = csActiveHigh;
        }
        return fd;
    }
    int spiClose(int fd)
    {
        {
            std::lock_guard<std::mutex> lock(m_manualCsLock); // note: lock spi mutex
            for (auto &pin : m_pins) { if (pin.spiCsFd == fd) { pin.spiCsFd = -1; break; } }
        }
        int status = close(fd);
        return status; // note: unlock spi mutex
    }
    int spiWrite(int fd, char *byteArray, int byteCount)
    {
        if (byteCount < 0) { errno = EINVAL; return -1; }
        int status = spiTransfer(fd, nullptr, byteArray, byteCount);
        return status;
    }
    int spiRead(int fd, char *byteArray, int byteCount)
    {
        if (byteCount < 0) { errno = EINVAL; return -1; }
        int status = spiTransfer(fd, byteArray, nullptr, byteCount);
        return status;
    }
    int setCustomPinMap(const BearcatGpio::PinInfo *pinInfo)
    {
        if (pinInfo == nullptr) { errno = EINVAL; return -1; }
        std::memcpy(m_pinInfo.data(), pinInfo, sizeof(m_pinInfo)); // pinInfo is a raw pointer (public API), so this one spot still needs memcpy
        return 0;
    }
    bool fastMmioAvailable() { return (m_socKind != SocKind::NONE); }
private:
    BearcatGpioInternals() = default;
    ~BearcatGpioInternals() { terminate(); mmioUnmap(); } // note: unmapping happens only at true process exit, not terminate
    const BearcatGpio::PinInfo* lookupPinInfo(int pin)
    {
        if (m_initialized == false) { errno = ENODEV; return nullptr; }
        if ((pin < 1) || (pin > BearcatGpio::MAX_HEADER_PINS)) { errno = EINVAL; return nullptr; }
        const BearcatGpio::PinInfo &p = m_pinInfo[pin];
        if (p.gpiochip < 0) { errno = ENXIO; return nullptr; } // power/gnd/unmapped
        return &p;
    }
    static SocKind detectSoc()
    {
        if (Bcm2711::probe() == true) return SocKind::BCM2711;
        if (Bcm2712RP1::probe() == true) return SocKind::BCM2712RP1;
        return SocKind::NONE;
    }
    static const char *socKindName(SocKind socKind)
    {
        switch (socKind) {
        case SocKind::BCM2711: return "BCM2711 (Raspberry Pi 4B)";
        case SocKind::BCM2712RP1: return "BCM2712RP1 (Raspberry Pi 5)";
        case SocKind::NONE: return "none";
        default: return "unknown";
        }
    }
    static int requestLine(PinHandle &pin, int chip, int line, std::uint64_t flags, int initialValue, bool debounce = false)
    {
        if (pin.fd >= 0) { close(pin.fd); pin.fd = -1; }
        char devpath[32];
        snprintf(devpath, sizeof(devpath), "/dev/gpiochip%d", chip);
        int chipFd = open(devpath, O_RDWR | O_CLOEXEC);
        if (chipFd < 0) return -1;
        gpio_v2_line_request request{};
        request.num_lines = 1;
        request.offsets[0] = static_cast<std::uint32_t>(line);
        snprintf(request.consumer, sizeof(request.consumer), "BearcatGpio");
        request.config.flags = flags;
        if (flags & GPIO_V2_LINE_FLAG_OUTPUT) {
            request.config.num_attrs = 1;
            request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
            request.config.attrs[0].attr.values = initialValue ? 1u : 0u;
            request.config.attrs[0].mask = 1u;
        } else if (debounce) {
            request.config.num_attrs = 1;
            request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
            request.config.attrs[0].attr.debounce_period_us = DEFAULT_DEBOUNCE_USEC;
            request.config.attrs[0].mask = 1u;
        }
        int rc = ioctl(chipFd, GPIO_V2_GET_LINE_IOCTL, &request);
        int savedErrno = errno;
        close(chipFd);
        if (rc < 0) { errno = savedErrno; return -2; }
        pin.chip = chip;
        pin.line = line;
        pin.fd = request.fd;
        pin.isOutput = ((flags & GPIO_V2_LINE_FLAG_OUTPUT) != 0);
        bool rising = ((flags & GPIO_V2_LINE_FLAG_EDGE_RISING) != 0);
        bool falling = ((flags & GPIO_V2_LINE_FLAG_EDGE_FALLING) != 0);
        if (rising && falling) pin.armedEdge = BearcatGpio::EDGE_BOTH;
        else if (rising) pin.armedEdge = BearcatGpio::EDGE_RISING;
        else if (falling) pin.armedEdge = BearcatGpio::EDGE_FALLING;
        else pin.armedEdge = -1;
        pin.armedDebounce = debounce;
        return pin.fd;
    }
    int mmioMap(SocKind kind)
    {
        switch (kind) {
        case SocKind::BCM2711: return Bcm2711::mapRegisters(m_registers, m_memFd);
        case SocKind::BCM2712RP1: return Bcm2712RP1::mapRegisters(m_registers, m_memFd);
        default: return -1;
        }
    }
    void mmioUnmap()
    {
        switch (m_socKind) {
        case SocKind::BCM2711: Bcm2711::unmapRegisters(m_registers, m_memFd); break;
        case SocKind::BCM2712RP1: Bcm2712RP1::unmapRegisters(m_registers, m_memFd); break;
        default: break;
        }
    }
    int mmioSet(int gpiochip, int line, int value)
    {
        switch (m_socKind) {
        case SocKind::BCM2711: return Bcm2711::set(m_registers, gpiochip, line, value);
        case SocKind::BCM2712RP1: return Bcm2712RP1::set(m_registers, gpiochip, line, value);
        default: return -1;
        }
    }
    int mmioGet(int gpiochip, int line)
    {
        switch (m_socKind) {
        case SocKind::BCM2711: return Bcm2711::get(m_registers, gpiochip, line);
        case SocKind::BCM2712RP1: return Bcm2712RP1::get(m_registers, gpiochip, line);
        default: return -1;
        }
    }
    int mmioSetup(int gpiochip, int line, bool output, PinHandle &h)
    {
        switch (m_socKind) {
        case SocKind::BCM2711:
        {
            int status = Bcm2711::setup(m_registers, gpiochip, line, output);
            h.mmioConfigured = true;
            h.isOutput = output;
            return status;
        }
        case SocKind::BCM2712RP1:
        {
            int status = Bcm2712RP1::setup(m_registers, gpiochip, line, output);
            h.mmioConfigured = true;
            h.isOutput = output;
            return status;
        }
        default: return -1;
        }
    }
    enum class ClaimResult { NOT_RUNNING, CLAIMED, ALREADY_CLAIMED }; // note: atomically claim right to stop+join iPin PWM thread
    ClaimResult claimPwmStop(int pin)
    {
        PinHandle &h = m_pins[pin];
        std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
        if (h.pwmThread.joinable() == false) return ClaimResult::NOT_RUNNING; // note: unlock pin mutex
        if (h.pwmStopping.exchange(true)) return ClaimResult::ALREADY_CLAIMED; // note: unlock pin mutex
        h.pwmRunning = false;
        return ClaimResult::CLAIMED; // note: unlock pin mutex
    }
    int stopAndJoinPwmIfRunning(int pin)
    {
        PinHandle &h = m_pins[pin];
        switch (claimPwmStop(pin)) {
        case ClaimResult::NOT_RUNNING:
            return 0;
        case ClaimResult::CLAIMED:
            h.pwmThread.join();
            h.pwmStopping = false;
            return 0;
        case ClaimResult::ALREADY_CLAIMED:
            while (h.pwmThread.joinable()) std::this_thread::yield();
            return 0;
        default:
            return 0;
        }
    }
    ClaimResult claimSignalerStop(int pin)
    {
        PinHandle &h = m_pins[pin];
        std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
        if (h.signalerThread.joinable() == false) return ClaimResult::NOT_RUNNING; // note: unlock pin mutex
        if (h.signalerStopping.exchange(true)) return ClaimResult::ALREADY_CLAIMED; // note: unlock pin mutex
        h.signalerRunning = false;
        return ClaimResult::CLAIMED; // note: unlock pin mutex
    }
    int stopAndJoinSignalerIfRunning(int pin)
    {
        PinHandle &h = m_pins[pin];
        switch (claimSignalerStop(pin)) {
        case ClaimResult::NOT_RUNNING:
            return 0;
        case ClaimResult::CLAIMED:
            h.signalerThread.join();
            h.signalerStopping = false;
            return 0;
        case ClaimResult::ALREADY_CLAIMED:
            while (h.signalerThread.joinable()) std::this_thread::yield();
            return 0;
        default:
            return 0;
        }
    }
    int resetAllPins()
    {
        for (int pin = 0; pin <= BearcatGpio::MAX_HEADER_PINS; pin++) {
            stopAndJoinPwmIfRunning(pin); // note: stopAndJoinPwmIfRunning must happen before resetting the rest
            stopAndJoinSignalerIfRunning(pin); // note: stopAndJoinSignalerIfRunning must happen before resetting the rest
            // note: pin.pwmRunning / pin.pwmStopping / pin.signalerRunning / pin.signalerStopping are false due to the above
            std::lock_guard<std::mutex> lock(m_pinLocks[pin]); // note: lock pin mutex
            PinHandle &h = m_pins[pin];
            if (h.fd >= 0) close(h.fd);
            h.chip = -1;
            h.line = -1;
            h.fd = -1;
            h.isOutput = false;
            h.armedEdge = -1;
            h.armedDebounce = false;
            h.pwmFreq_Hz = 1000.0;
            h.pwmDutyFrac = 0.5; // note: unlock pin mutex
        }
        return 0;
    }
    static int i2cIoctl(int fd, std::uint8_t readWrite, std::uint8_t command, int size, union i2c_smbus_data *data)
    {
        struct i2c_smbus_ioctl_data args{};
        args.read_write = readWrite;
        args.command = command;
        args.size = static_cast<std::uint32_t>(size);
        args.data = data;
        int status = ioctl(fd, I2C_SMBUS, &args);
        return status;
    }
private:
    BearcatGpio::Board m_board = BearcatGpio::BOARD_RPI4B;
    std::array<BearcatGpio::PinInfo, BearcatGpio::MAX_HEADER_PINS + 1> m_pinInfo{};
    bool m_initialized = false;
    SocKind m_socKind = SocKind::NONE;
    volatile std::uint32_t *m_registers = nullptr;
    int m_memFd = -1;
    std::array<std::mutex, BearcatGpio::MAX_HEADER_PINS + 1> m_pinLocks;
    std::array<PinHandle, BearcatGpio::MAX_HEADER_PINS + 1> m_pins{}; // note: pin 0 is unused
    std::mutex m_manualCsLock;
};

} // anonymous namespace

/////////////////////////////////////// START: MAIN API FUNCTIONS ////////////////////////////////////
void BearcatGpio::msleep(int msec)
{
    struct timespec ts;
    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (long)(msec % 1000) * 1000000L;
    nanosleep(&ts, nullptr);
}
void BearcatGpio::sleep(int sec) { ::sleep(static_cast<unsigned int>(sec)); }
int BearcatGpio::initialize(Board board) { return BearcatGpioInternals::instance().initialize(board); }
int BearcatGpio::terminate() { return BearcatGpioInternals::instance().terminate(); }
int BearcatGpio::setupOutput(int pin) { return BearcatGpioInternals::instance().setupOutput(pin); }
int BearcatGpio::write(int pin, int level) { return BearcatGpioInternals::instance().write(pin, level); }
int BearcatGpio::writeFast(int pin, int level) { return BearcatGpioInternals::instance().writeFast(pin, level); }
int BearcatGpio::setupInput(int pin, int pull) { return BearcatGpioInternals::instance().setupInput(pin, pull); }
int BearcatGpio::read(int pin) { return BearcatGpioInternals::instance().read(pin); }
int BearcatGpio::setupInputSignaler(int pin, int edge, int timeout_msec, bool debounce, std::function<void(int pin, int level, int tick_usec)> &callback)
{
    return BearcatGpioInternals::instance().setupInputSignaler(pin, edge, timeout_msec, debounce, callback);
}
int BearcatGpio::stopInputSignaler(int pin) { return BearcatGpioInternals::instance().stopInputSignaler(pin); }
int BearcatGpio::startSoftPwm(int pin, int frequencyHz, float dutyCycle, int range)
{
    return BearcatGpioInternals::instance().startSoftPwm(pin, frequencyHz, dutyCycle, range);
}
int BearcatGpio::stopSoftPwm(int pin) { return BearcatGpioInternals::instance().stopSoftPwm(pin); }
int BearcatGpio::i2cOpen(int bus, int address) { return BearcatGpioInternals::i2cOpen(bus, address); }
int BearcatGpio::i2cClose(int fd) { return BearcatGpioInternals::i2cClose(fd); }
int BearcatGpio::i2cWriteByte(int fd, std::uint8_t value) { return BearcatGpioInternals::i2cWriteByte(fd, value); }
int BearcatGpio::i2cReadByte(int fd) { return BearcatGpioInternals::i2cReadByte(fd); }
int BearcatGpio::i2cWriteReg8(int fd, int reg, std::uint8_t value) { return BearcatGpioInternals::i2cWriteReg8(fd, reg, value); }
int BearcatGpio::i2cReadReg8(int fd, int reg) { return BearcatGpioInternals::i2cReadReg8(fd, reg); }
int BearcatGpio::i2cWriteReg16(int fd, int reg, std::uint16_t value) { return BearcatGpioInternals::i2cWriteReg16(fd, reg, value); }
int BearcatGpio::i2cReadReg16(int fd, int reg) { return BearcatGpioInternals::i2cReadReg16(fd, reg); }
int BearcatGpio::i2cWriteBlock(int fd, int reg, char *byteArray, int byteCount) { return BearcatGpioInternals::i2cWriteBlock(fd, reg, byteArray, byteCount); }
int BearcatGpio::i2cReadBlock(int fd, int reg, char *byteArray) { return BearcatGpioInternals::i2cReadBlock(fd, reg, byteArray); }
int BearcatGpio::spiOpen(int bus, int chan, int baud, int mode, bool csActiveHigh, int bitsPerWord, bool lsbFirst)
{
    return BearcatGpioInternals::instance().spiOpen(bus, chan, baud, mode, csActiveHigh, bitsPerWord, lsbFirst);
}
int BearcatGpio::spiClose(int fd) { return BearcatGpioInternals::instance().spiClose(fd); }
int BearcatGpio::spiRead(int fd, char *byteArray, int byteCount) { return BearcatGpioInternals::instance().spiRead(fd, byteArray, byteCount); }
int BearcatGpio::spiWrite(int fd, char *byteArray, int byteCount) { return BearcatGpioInternals::instance().spiWrite(fd, byteArray, byteCount); }
int BearcatGpio::setCustomPinMap(const PinInfo *entries) { return BearcatGpioInternals::instance().setCustomPinMap(entries); }
bool BearcatGpio::fastMmioAvailable() { return BearcatGpioInternals::instance().fastMmioAvailable(); }
/////////////////////////////////////// END: MAIN API FUNCTIONS ////////////////////////////////////

/* =====================================================================
 * OPTIONAL DEMO / SMOKE TEST  (build with -DBEARCATGPIO_TEST_MAIN)
 * ===================================================================== */
#ifdef BEARCATGPIO_TEST_MAIN
#include <cstdio>

int main()
{
    if (BearcatGpio::initialize() != 0) { perror("initialize"); return 1; }
    printf("fast path available: %s\n", BearcatGpio::fastMmioAvailable() ? "yes" : "no");

    // write() now auto-selects mmio if available, else ioctl -- no
    // mode argument needed either way.
    BearcatGpio::setupOutput(12);
    for (int i = 0; i < 5; i++) {
        BearcatGpio::write(12, 1);
        BearcatGpio::msleep(200);
        BearcatGpio::write(12, 0);
        BearcatGpio::msleep(200);
    }

    // writeFast() remains a distinct, explicit opt-in: mmio-only, no
    // fallback, no locking. setupOutput() must still be called first.
    for (int i = 0; i < 5; i++) {
        BearcatGpio::writeFast(12, 1);
        BearcatGpio::msleep(200);
        BearcatGpio::writeFast(12, 0);
        BearcatGpio::msleep(200);
    }

    // Read pin 11 (GPIO17) with pull-up
    BearcatGpio::setupInput(11, BearcatGpio::PULL_UP);
    printf("pin 11 = %d\n", BearcatGpio::read(11));

    // Wait up to 5s for either edge on pin 7 (GPIO4), debounced --
    // the callback now fires on every edge, continuously, until
    // stopInputSignaler(7) is called.
    std::function<void(int,int,int,bool)> onPin7Edge = [](int pin, int edge, int timeout, bool debounce) {
        printf("edge callback: pin=%d edge=%d timeout=%d debounce=%d, level now=%d\n",
               pin, edge, timeout, debounce, BearcatGpio::read(pin));
    };
    int rc = BearcatGpio::setupInputSignaler(7, BearcatGpio::EDGE_BOTH, 5000, true, onPin7Edge);
    printf("signaler start rc=%d (0=started, negative=error)\n", rc);
    BearcatGpio::sleep(5); // let a few edges fire, if any occur
    BearcatGpio::stopInputSignaler(7);

    // Two "motors" (soft PWM channels) on independent pins, running
    // concurrently, to exercise the per-pin locking this change is for.
    BearcatGpio::startSoftPwm(33, 1000, 0.25f, 100);  // pin 33 (GPIO13)
    BearcatGpio::startSoftPwm(35, 500, 0.5f, 100);    // pin 35 (GPIO19)
    BearcatGpio::sleep(2);
    BearcatGpio::stopSoftPwm(33);
    BearcatGpio::stopSoftPwm(35);

    // Call terminate() and confirm the mmio backend is STILL available
    // afterward -- proof it wasn't torn down, only the pin/PWM state was.
    BearcatGpio::terminate();
    printf("after terminate(): fast path still available: %s\n", BearcatGpio::fastMmioAvailable() ? "yes" : "no");

    return 0; // real unmap happens here, in BearcatGpioInternals' destructor
}

#endif // BEARCATGPIO_TEST_MAIN

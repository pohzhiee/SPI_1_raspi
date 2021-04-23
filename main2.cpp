//
// Access the Raspberry Pi System Timer registers directly.
//
#include <cstdlib>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstdint>
#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

struct GPIO {
    volatile uint32_t GPFSEL0;
    volatile uint32_t GPFSEL1;
    uint32_t GPFSEL2;
    uint32_t GPFSEL3;
    uint32_t GPFSEL4;
    uint32_t GPFSEL5;
    uint32_t unused1_;
    volatile uint32_t GPSET0;
    uint32_t GPSET1;
    uint32_t unused2_;
    volatile uint32_t GPCLR0;
    uint32_t GPCLR1;
    uint32_t unused3_;
    uint32_t GPLEV0;
    uint32_t GPLEV1;
    uint32_t unused4_;
    uint32_t GPEDS0;
    uint32_t GPEDS1;
    uint32_t unused5_;
    uint32_t GPREN0;
    uint32_t GPREN1;
    uint32_t unused6_;
    uint32_t GPFEN0;
    uint32_t GPFEN1;
    uint32_t unused7_;
    uint32_t GPHEN0;
    uint32_t GPHEN1;
    uint32_t unused8_;
    uint32_t GPLEN0;
    uint32_t GPLEN1;
    uint32_t unused9_;
    uint32_t GPAREN0;
    uint32_t GPAREN1;
    uint32_t unused10_;
    uint32_t GPAFEN0;
    uint32_t GPAFEN1;
};
struct SPI {
    volatile uint32_t CS;
    uint32_t FIFO;
    uint32_t CLK;
    uint32_t DLEN;
    uint32_t LTOH;
    uint32_t DC;
};

static_assert(sizeof(GPIO)==144);

// #define PERIPHERAL_BASE 0x20000000   // For Pi 1 and 2
//#define PERIPHERAL_BASE 0x3F000000      // For Pi 3
#define PERIPHERAL_BASE 0xFE000000      // For Pi 4
#define GPIO_OFFSET 0x200000
#define SPI0_OFFSET 0x204000
#define SPI3_OFFSET 0x204600
#define GPIO_BASE (PERIPHERAL_BASE + GPIO_OFFSET)
#define SPI0_BASE (PERIPHERAL_BASE + SPI0_OFFSET)
#define SPI3_BASE (PERIPHERAL_BASE + SPI3_OFFSET)
//#define ST_BASE (PERIPHERAL_BASE + SYSTEM_TIMER_OFFSET)

using namespace std::chrono_literals;
using namespace std::chrono;

std::condition_variable cv;
std::mutex cv_m;
std::atomic<bool> isDone = false;

struct SPITransferInput {
    SPI* spiPtr;
    uint8_t* bufferPtr;
    uint32_t len; // when we have C++20 with std::span can consider removing this len and ptr thing
    uint32_t CSRegisterDefault;
};

void transmitSPI2(std::array<SPITransferInput, 2>& spiInputs)
{

    std::atomic<uint32_t> txCount1 = 0;
    std::atomic<uint32_t> txCount2 = 0;
    // Set the transfer active bit to initiate transfer
    bool isDone1;
    bool isDone2;
    SPI* spi1Ptr = spiInputs.at(0).spiPtr;
    SPI* spi2Ptr = spiInputs.at(1).spiPtr;
    uint8_t* buf1Ptr = spiInputs.at(0).bufferPtr;
    uint8_t* buf2Ptr = spiInputs.at(1).bufferPtr;
    // Set the transfer active bit to initiate transfer
    spi1Ptr->CS |= 0b1 << 7;
    spi2Ptr->CS |= 0b1 << 7;
    do {
        isDone1 = txCount1==spiInputs.at(0).len;
        isDone2 = txCount2==spiInputs.at(1).len;

        if (!isDone1) {
            uint32_t status = spi1Ptr->CS;
            if (status & (0b1 << 18)) {
                uint32_t data = *reinterpret_cast<char*>(&buf1Ptr[txCount1]);
                spi1Ptr->FIFO = data;
                txCount1++;
            }
        }
        if (!isDone2) {
            uint32_t status = spi2Ptr->CS;
            if (status & (0b1 << 18)) {
                uint32_t data = *reinterpret_cast<char*>(&buf2Ptr[txCount2]);
                spi2Ptr->FIFO = data;
                txCount2++;
            }
        }
    }
    while (!isDone1 || !isDone2);

    bool transferDone1 = false;
    bool transferDone2 = false;
    while (!transferDone1 || !transferDone2) {
        transferDone1 = spi1Ptr->CS & (0b1 << 16);
        transferDone2 = spi2Ptr->CS & (0b1 << 16);
    }

    spi1Ptr->CS = spiInputs.at(0).CSRegisterDefault;
    spi2Ptr->CS = spiInputs.at(1).CSRegisterDefault;
}

void transmitSPI(SPI* spiPtr, uint8_t* bufPtr, uint32_t len, uint32_t spiDefaults)
{
    while (true) {
        {
            std::unique_lock<std::mutex> lk(cv_m);
            cv.wait(lk);
        }
        std::atomic<uint32_t> txCount = 0;
        // Set the transfer active bit to initiate transfer
        spiPtr->CS |= 0b1 << 7;
        do {
            uint32_t status = spiPtr->CS;

            if (txCount<len) {
                if (status & (0b1 << 18)) {
                    uint32_t data = *reinterpret_cast<char*>(&bufPtr[txCount]);
                    spiPtr->FIFO = data;
                    txCount++;
                }
            }
        }
        while (txCount!=len);
        while (!(spiPtr->CS & (0b1 << 16)));
        spiPtr->CS = spiDefaults;
        if (isDone)
            break;
    }
}

int main(int argc, char** argv)
{
    int fd;
    if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC))<0) {
        printf("can't open /dev/mem \n");
        exit(-1);
    }
    void* gpio_mmap_ptr = mmap(
            0,
            244,
            PROT_READ | PROT_WRITE,
            MAP_SHARED | MAP_LOCKED,
            fd,
            GPIO_BASE
    );
    void* spi0_mmap_ptr = mmap(
            0,
            2048,
            PROT_READ | PROT_WRITE,
            MAP_SHARED | MAP_LOCKED,
            fd,
            SPI0_BASE
    );

    close(fd);
    if (gpio_mmap_ptr==MAP_FAILED) {
        printf("mmap error %d\n", gpio_mmap_ptr);  // errno also set!
        exit(-1);
    }
    if (spi0_mmap_ptr==MAP_FAILED) {
        printf("spi0 mmap error %d\n", spi0_mmap_ptr);  // errno also set!
        exit(-1);
    }
    void* spi3memPtr = reinterpret_cast<void*>(reinterpret_cast<char*>(spi0_mmap_ptr)+0x600);

    GPIO* gpio_ptr = reinterpret_cast<GPIO*>(gpio_mmap_ptr);
    // Setup the GPIO AF for SPI0
    // Set GPIO14, 15 as output
    {
        auto temp = gpio_ptr->GPFSEL1;
        gpio_ptr->GPFSEL1 = (temp & (~(0b111 << 12 | 0b111 << 15))) | (0b001 << 12 | 0b001 << 15);
    }

    //8, 9, 10, 11 to AF0
    {
        // Set AF0 for GPIO 8,9
        auto temp = gpio_ptr->GPFSEL0;
        // clear and then set the registers
        gpio_ptr->GPFSEL0 = (temp & (~(0b111 << 24 | 0b111 << 27))) | (0b100 << 24 | 0b100 << 27);
        // Set AF0 for GPIO 10,11
        temp = gpio_ptr->GPFSEL1;
        // clear and then set the registers
        gpio_ptr->GPFSEL1 = (temp & (~(0b111 << 0 | 0b111 << 3))) | (0b100 << 0 | 0b100 << 3);
    }
    // Setup GPIO AF for SPI3
    // 0,1,2,3 to AF3
    {
        // Set AF3 for GPIO 0, 1, 2, 3
        // Setting AF3 does not require clearing the register beforehand because setting 0b111 is very straightforward
        auto temp = gpio_ptr->GPFSEL0;
        gpio_ptr->GPFSEL0 = temp | 0b111 | 0b111 << 3 | 0b111 << 6 | 0b111 << 9;
    }

    SPI* spi0Ptr = reinterpret_cast<SPI*>(spi0_mmap_ptr);
    SPI* spi3Ptr = reinterpret_cast<SPI*>(spi3memPtr);
    // set 0b11 on FIFO clear, rest of the settings use default
    // CS0, CPOL=low, CPHA=middle of bit, CS_POLARITY=active low,
    uint32_t spiDefaults = 0b11 << 4;

    // Set SPI0
    {
        spi0Ptr->DLEN = 2;/* undocumented, stops inter-byte gap */
        spi0Ptr->CS = spiDefaults;
        spi0Ptr->CLK = 100;
    }
    // Set SPI3
    {
        spi3Ptr->DLEN = 2;/* undocumented, stops inter-byte gap */
        spi3Ptr->CS = spiDefaults;
        spi3Ptr->CLK = 100;
    }
    char data1[] = "ABCDEFGHIJKLMNOPQRSTU";
    char data2[] = "0123456789abcdefghijkl";
    std::array<SPITransferInput, 2> xferInput = {
            SPITransferInput{spi0Ptr, (uint8_t*)data1, 20, spiDefaults},
            SPITransferInput{spi3Ptr, (uint8_t*)data2, 20, spiDefaults}
    };
//    transmitSPI2(xferInput);
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i<1000; i++) {
        std::this_thread::sleep_until(next);
        if(i%2 == 0){
            gpio_ptr->GPSET0 |= (0b1 << 14 | 0b1 << 15);
        }
        else{
            gpio_ptr->GPCLR0 |= (0b1 << 14 | 0b1 << 15);
        }
        transmitSPI2(xferInput);
        next = next+duration<long, std::ratio<1, 1000>>{1};
    }

    return 0;
}
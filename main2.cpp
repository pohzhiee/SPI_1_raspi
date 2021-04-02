//
// Access the Raspberry Pi System Timer registers directly.
//
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstdint>
#include <iostream>
#include <atomic>

struct GPIO{
    uint32_t GPFSEL0;
    uint32_t GPFSEL1;
    uint32_t GPFSEL2;
    uint32_t GPFSEL3;
    uint32_t GPFSEL4;
    uint32_t GPFSEL5;
    uint32_t unused1_;
    uint32_t GPSET0;
    uint32_t GPSET1;
    uint32_t unused2_;
    uint32_t GPCLR0;
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
struct SPI{
    uint32_t CS;
    uint32_t FIFO;
    uint32_t CLK;
    uint32_t DLEN;
    uint32_t LTOH;
    uint32_t DC;
};

static_assert(sizeof(GPIO) == 144);

// #define PERIPHERAL_BASE 0x20000000   // For Pi 1 and 2
//#define PERIPHERAL_BASE 0x3F000000      // For Pi 3
#define PERIPHERAL_BASE 0xFE000000      // For Pi 4
#define GPIO_OFFSET 0x200000
#define SPI0_OFFSET 0x204000
#define GPIO_BASE (PERIPHERAL_BASE + GPIO_OFFSET)
#define SPI0_BASE (PERIPHERAL_BASE + SPI0_OFFSET)
//#define ST_BASE (PERIPHERAL_BASE + SYSTEM_TIMER_OFFSET)

int main(int argc, char **argv) {
    int32_t t0, t1;

    int  fd;
    if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0) {
        printf("can't open /dev/mem \n");
        exit(-1);
    }
    void *somePtr = mmap(
            0,
            244,
            PROT_READ | PROT_WRITE,
            MAP_SHARED | MAP_LOCKED,
            fd,
            GPIO_BASE
    );
    void *spi0memPtr = mmap(
            0,
            sizeof(SPI),
            PROT_READ | PROT_WRITE,
            MAP_SHARED | MAP_LOCKED,
            fd,
            SPI0_BASE
    );

    close(fd);
    if (somePtr == MAP_FAILED) {
        printf("mmap error %d\n", somePtr);  // errno also set!
        exit(-1);
    }
    if (spi0memPtr == MAP_FAILED) {
        printf("spi0 mmap error %d\n", spi0memPtr);  // errno also set!
        exit(-1);
    }

    GPIO *ptr2 = reinterpret_cast<GPIO*>(somePtr);
    // Setup the GPIO AF for SPI0
    //8, 9, 10, 11 to AF0
    {
        auto temp = ptr2->GPFSEL0;
        // Set AF0 for GPIO 8,9
        ptr2->GPFSEL0 = (temp & (~(0b111 << 24 | 0b111 << 27))) | (0b100 << 24 | 0b100 << 27);
        temp = ptr2->GPFSEL1;
        ptr2->GPFSEL1 = (temp & (~(0b111 << 0 | 0b111 << 3))) | (0b100 << 0 | 0b100 << 3);
    }

    /*
    auto asd1 = ptr2->GPFSEL1;
    auto asd1_cleared = asd1 & (~(asd1 & 0b111 << 15));
    auto asd1_cleared2 = asd1_cleared & (~(asd1_cleared & 0b111 << 12));
    auto asd1_cleared3 = asd1_cleared2 & (~(asd1_cleared2 & 0b111 << 24));
    auto asd1_set = asd1_cleared3 | (0b001 << 15 | 0b001 << 12 | 0b001 << 24);
    ptr2->GPFSEL1 = asd1_set;
    auto asd = ptr2->GPFSEL1;
    ptr2->GPSET0 = (0b1 << 15 | 0b1 << 14 | 0b1 << 18); // set
    ptr2->GPCLR0 = 1 << 15; // clear
//    ptr2->GPSET0 |= 1 << 14; // set
    ptr2->GPCLR0 = 1 << 14; // clear
    uint32_t level0 = ptr2->GPLEV0;
    uint32_t level15 = ((level0) & (0b1 << 15)) >> 15;
    uint32_t level14 = ((level0) & (0b1 << 14)) >> 14;
    uint32_t level18 = ((level0) & (0b1 << 18)) >> 18;

    std::cout << "HELLO" << std::endl;
    std::cout << "Lv15: " << level15 << std::endl;
    std::cout << "Lv14: " << level14 << std::endl;
    std::cout << "Lv18: " << level18 << std::endl;*/

    SPI *spi0Ptr = reinterpret_cast<SPI*>(spi0memPtr);
    uint32_t spiDefaults = 3 << 4;

    spi0Ptr->DLEN = 2;/* undocumented, stops inter-byte gap */
    spi0Ptr->CS = spiDefaults;
    spi0Ptr->CLK = 250;
    char asd[] = "ABCDEFGHIJKLMNOPQRSTU";
    std::atomic<int> dataCount = 20;
    std::atomic<int> currentDataIndex = 0;
    spi0Ptr->CS |= 0b1 << 7;
    do{
        uint32_t status = spi0Ptr->CS;
        if(status & (0b1 << 18)){
            uint32_t data = *reinterpret_cast<char*>(&asd[currentDataIndex]);
            spi0Ptr->FIFO = data;
            currentDataIndex+=1;
            dataCount-=1;
        }
    } while(dataCount > 0);
    while(!(spi0Ptr->CS & (0b1 << 16)));
    spi0Ptr->CS = spiDefaults;




    return 0;
}
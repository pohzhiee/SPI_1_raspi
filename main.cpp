#include <iostream>
#include "pigpio.h"

#define LOOPS 4
#define SPEED 1000000
#define BYTES 15
int main(){
   int loops=LOOPS;
   int speed=SPEED;
   int bytes=BYTES;
   int i;
   int h, h2;
   double start, diff;
   char buf[50] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
   char bufRx[50];

   if (gpioInitialise() < 0) return 1;

//   gpioWrite(15, 1);
//
//    gpioWrite(15, 0);
   h = spiOpen(0, speed, 0);
//
   if (h < 0) return 2;
//
   start = time_time();
//
  spiXfer(h, buf, bufRx, bytes);
//      gpioWrite(15, i%2);
//
   diff = time_time() - start;
//
//   printf("sps=%.1f: %d bytes @ %d bps (loops=%d time=%.1f)\n",
//      (double)loops / diff, bytes, speed, loops, diff);
//
   spiClose(h);

   gpioTerminate();

}
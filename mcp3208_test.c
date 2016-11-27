/*
* mcp3208.c
*
* Created on: Oct 07, 2014
* Author: DougM
* Based on work by: AmosSam
*/
#include "simpletools.h" // Include simple tools
// #include <propeller.h>


#define ADC_cs 18
#define ADC_clk 16
#define ADC_do 17
#define ADC_di 17

//#define ADC_CS 23//18
//#define ADC_CLK 21//16
//#define ADC_DATA 22//17
// function protos
void pinPulseHL(int pin, int d, int d1);
void pinPulseLH(int pin, int d, int d1);
int readADC(int channel, int din, int dout, int clk, int cs);
int readADCAverage(int channel, int din, int dout, int clk, int cs, int samples);

// party time
int main()
{
int ADCVal;
ADCVal = 0;

while(1)
{

// readADC(channel, di, do, clk, cs);
// ADCVal = readADC(1, ADC_di, ADC_do, ADC_clk, ADC_cs);
// readADCAverage(int channel, int din, dout, int clk, int cs, int samples)
printf("Reading...\n");
ADCVal = readADCAverage(1, ADC_di, ADC_do, ADC_clk, ADC_cs, 10);

printf("ADCVal = %d\r", ADCVal);
pause(500);
}
}

/**
* @brief sets pin, int pin, state to high, delays specified time, int d,
* and then puts pin low, and delays specified time, int d1
*/
void pinPulseHL(int pin, int d, int d1)
{
high(pin);
if (d > 10)
usleep(d);
low(pin);
if (d1 > 10)
usleep(d1);
}

/**
* @brief sets pin, int pin, state to low, delays specified time, int d,
* and then puts pin high, and delays specified time, int d1
*/
void pinPulseLH(int pin, int d, int d1)
{
high(pin);
if (d > 10)
usleep(d);
low(pin);
if (d1 > 10)
usleep(d1);
}

/**
* @brief Reads value from mcp3208/mcp3204 ADC
* @details This function reads data from mcp3208/mcp3204 ADC
* over SPI.
*
* @param channel What channel to read? 0 - 7
* @param din Pin number on which DIN is connected
* @param dout Pin number on which DOUT is connected
* @param clk Pin number on which CLK is connected
* @param cs Pin number on which CS is connected
* @returns Value read from mcp3208/mcp3204 ADC
*/
int readADC(int channel, int din, int dout, int clk, int cs)
{
int i;
int AdcResult;
int setup;

// In case pin was already been low, we put it high
// so we can initiate communication after setting up pins
high(cs);
low(din);
low(clk);
low(cs); // Active chip select by setting pin low

// Sending configuration to device
setup = channel | 0b11000;
for(i=0; i < 5;i++) {
pinPulseHL(clk, 10, 0);
if ((setup & 0b10000) == 0b10000)
high(din);
else
low(din); // is MSB != 0
setup <<= 1; // shift left
usleep(10);
}

pinPulseHL(clk, 10, 10); //Empty clock, for sampling

pinPulseHL(clk, 10, 10); //Device returns low, NULL bit, we ignore it...

// read ADC result 12 bit
AdcResult=0;
for(i=0;i<12;i++) {
// We are sending pulse, clock signal, to ADC, because on falling edge it will return data...
pinPulseHL(clk, 10, 0);
// Shifting bit to left, to make room for current one...
AdcResult<<=1;
AdcResult=AdcResult | (input(dout) & 0x01);
usleep(10);
}
high(cs);
return(AdcResult);
}


/**
* @brief Reads specified number of values from mcp3208/mcp3204 ADC
* @details This function reads specified number of times
* data from mcp3208/mcp3204 ADC over SPI,
*
* @param channel What channel to read? 0 - 7
* @param din Pin number on which DIN is connected
* @param dout Pin number on which DOUT is connected
* @param clk Pin number on which CLK is connected
* @param cs Pin number on which CS is connected
* @param samples Number of samples that will be gathered, and then taken average before it's returned
* @returns Value read from mcp3208/mcp3204 ADC
*/
int readADCAverage(int channel, int din, int dout, int clk, int cs, int samples)
{
int i, k = 0;

for (i=0;i<samples;i++)
k = k + readADC(channel, din, dout, clk, cs);
return k/samples;
}
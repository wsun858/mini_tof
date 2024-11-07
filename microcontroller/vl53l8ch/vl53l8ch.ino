// #include <SoftwareSerial.h>

/**
******************************************************************************
* @file    VL53L8CX_HelloWorld_I2C.ino
* @author  STMicroelectronics
* @version V1.0.0
* @date    12 June 2023
* @brief   Arduino test application for STMicroelectronics VL53L8CX
*          proximity sensor satellite based on FlightSense.
*          This application makes use of C++ classes obtained from the C
*          components' drivers.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/*
 * To use these examples you need to connect the VL53L8CX satellite sensor directly to the Nucleo board with wires as explained below:
 * pin 1 (SPI_I2C_n) of the VL53L8CX satellite connected to pin GND of the Nucleo board
 * pin 2 (LPn) of the VL53L8CX satellite connected to pin A3 of the Nucleo board
 * pin 3 (NCS) not connected
 * pin 4 (MISO) not connected
 * pin 5 (MOSI_SDA) of the VL53L8CX satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 6 (MCLK_SCL) of the VL53L8CX satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 7 (PWREN) of the VL53L8CX satellite connected to pin D11 of the Nucleo board
 * pin 8 (I0VDD) of the VL53L8CX satellite not connected
 * pin 9 (3V3) of the VL53L8CX satellite connected to 3V3 of the Nucleo board
 * pin 10 (1V8) of the VL53L8CX satellite not connected
 * pin 11 (5V) of the VL53L8CX satellite not connected
 * GPIO1 of VL53L8CX satellite connected to A2 pin of the Nucleo board (not used)
 * GND of the VL53L8CX satellite connected to GND of the Nucleo board
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
// #include <vl53l8cx_class.h>
// #include <vl53lmz_api.h>
#include "vl53lmz_api.h"
// #include <stdarg.h>
#include <LibPrintf.h>
// #include <HardwareSerial.h>
// #include "HardwareSerial.h"

// char report[256];

/* eanble either spi or i2c (undefined behavour if both enabled)*/
#define SPI_EN
// #define I2C_EN

#ifdef I2C_EN
#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#else
#define DEV_I2C Wire
#endif
#endif

/* define SerialPort Serial */

// nucleo f401re pinout 
#define LPN_PIN A3
#define I2C_RST_PIN -1
#define PWREN_PIN 11

#define SPI_CLK_PIN 3
#define SPI_MISO_PIN 5
#define SPI_MOSI_PIN 4
#define CS_PIN 10


/* blackpill f411 pinout (doesn't work)*/
// #define LPN_PIN A3
// #define LPN_PIN -1
// #define I2C_RST_PIN -1
// #define PWREN_PIN 7

// #define SPI_CLK_PIN 13
// #define SPI_MISO_PIN 12
// #define SPI_MOSI_PIN 11
// #define CS_PIN 6

#ifdef SPI_EN
SPIClass DEV_SPI(SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN);
#endif

VL53LMZ_Configuration the_i2c_p_dev;
VL53LMZ_Configuration the_spy_p_dev;

void init_spy(VL53LMZ_Configuration *p_dev, SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000, int lpn_pin = -1, int i2c_rst_pin = -1);

void init_i2c(VL53LMZ_Configuration *p_dev, TwoWire *i2c, int lpn_pin, int i2c_rst_pin = -1);

int begin(VL53LMZ_Configuration *p_dev);
void xprintf(const char *format, ...); // use if printf library doesn't work (doesn't work with floats tho have fun with that)

/* Setup ---------------------------------------------------------------------*/
extern int example12(VL53LMZ_Configuration *input_Dev);

void setup()
{
#ifdef I2C_EN
  init_i2c(&the_i2c_p_dev, &DEV_I2C, LPN_PIN, I2C_RST_PIN);
#endif

#ifdef SPI_EN
  init_spy(&the_spy_p_dev, &DEV_SPI, CS_PIN, 3000000);
#endif

  // Enable PWREN pin if present
  if (PWREN_PIN >= 0)
  {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize serial for output.

  // max (unstable) is 2mhz so far
  // very stable is 921600hz
  // kind of stable 1843200
  Serial.begin(921600, SERIAL_8O1);
  // Serial.begin(2000000, SERIAL_8O1);

  pinMode(LED_BUILTIN, OUTPUT);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // SerialUSB.begin();

// Initialize I2C bus.
#ifdef I2C_EN
  DEV_I2C.begin();

  /* max is 1 Mhz*/
  uint32_t clock_frequency = 1000000;
  DEV_I2C.setClock(clock_frequency);
  begin(&the_i2c_p_dev);
#endif

// Initalize SPI bus.
#ifdef SPI_EN
  DEV_SPI.begin();
  begin(&the_spy_p_dev);
#endif
}

void loop()
{
  int status = 1;
  bool toggling = false;

  // uint8_t countdown = 5;
  // while (countdown>0)
  // {
  //   delay(1000);
  //   digitalWrite(LED_BUILTIN, true);
  //   delay(1000);
  //   digitalWrite(LED_BUILTIN, false);

  //   Serial.print("helloworld\n");
  //   countdown--;
  // }
  
  uint8_t blink = 0;
  bool blink_tog = false;
  digitalWrite(LED_BUILTIN, HIGH);
  while (status != 0)
  {
    if (blink > 15)
    {
      digitalWrite(LED_BUILTIN, blink_tog);
      blink_tog = !blink_tog;
      blink = 0;
    }
    blink++;
    
    // Serial.print("hello world\n");
// baud rate to read input is 921600
// xprintf("hello world %5.4f \n\r", abc_tester_no_name_issues);
// status = example1();
// status = example2();
// status = example5();
#ifdef I2C_EN
    status = example12(&the_i2c_p_dev);
#endif

#ifdef SPI_EN
    status = example12(&the_spy_p_dev);
#endif

    // delay(1000);
  }
}

int begin(VL53LMZ_Configuration *p_dev)
{
  if (p_dev->platform.lpn_pin >= 0)
  {
    pinMode(p_dev->platform.lpn_pin, OUTPUT);
    digitalWrite(p_dev->platform.lpn_pin, LOW);
  }
  if (p_dev->platform.i2c_rst_pin >= 0)
  {
    pinMode(p_dev->platform.i2c_rst_pin, OUTPUT);
    digitalWrite(p_dev->platform.i2c_rst_pin, LOW);
  }

  if (p_dev->platform.dev_spi)
  {
    // Configure CS pin
    pinMode(p_dev->platform.cs_pin, OUTPUT);
    digitalWrite(p_dev->platform.cs_pin, HIGH);
  }
  return 0;
}

/** Constructorish
 * @param p_dev censor config object
 * @param[in] i2c device I2C to be used for communication
 * @param[in] lpn_pin pin to be used as component LPn
 * @param[in] i2c_rst_pin pin to be used as component I2C_RST
 */
void init_i2c(VL53LMZ_Configuration *p_dev, TwoWire *i2c, int lpn_pin, int i2c_rst_pin)
{
  memset((void *)p_dev, 0x0, sizeof(VL53LMZ_Configuration));
  p_dev->platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;
  p_dev->platform.dev_i2c = i2c;
  p_dev->platform.dev_spi = NULL;
  p_dev->platform.lpn_pin = lpn_pin;
  p_dev->platform.i2c_rst_pin = i2c_rst_pin;
}

/** Constructorish
 * @param p_dev censor config object
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 * @param[in] lpn_pin pin to be used as component LPn
 * @param[in] i2c_rst_pin pin to be used as component I2C_RST
 */
void init_spy(VL53LMZ_Configuration *p_dev, SPIClass *spi, int cs_pin, uint32_t spi_speed, int lpn_pin, int i2c_rst_pin)
{
  memset((void *)p_dev, 0x0, sizeof(VL53LMZ_Configuration));
  p_dev->platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;
  p_dev->platform.dev_i2c = NULL;
  p_dev->platform.dev_spi = spi;
  p_dev->platform.cs_pin = cs_pin;
  p_dev->platform.spi_speed = spi_speed;
  p_dev->platform.lpn_pin = lpn_pin;
  p_dev->platform.i2c_rst_pin = i2c_rst_pin;
}

void vl53lmz_on(VL53LMZ_Configuration *p_dev)
{
  if (p_dev->platform.lpn_pin >= 0)
  {
    digitalWrite(p_dev->platform.lpn_pin, HIGH);
  }
  delay(10);
}

/**
 * @brief       PowerOff the sensor
 * @return      void
 */
void vl53lmz_off(VL53LMZ_Configuration *p_dev)
{
  if (p_dev->platform.lpn_pin >= 0)
  {
    digitalWrite(p_dev->platform.lpn_pin, LOW);
  }
  delay(10);
}

void xprintf(const char *format, ...)
{
  char buffer[16384]; // or smaller or static &c.
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);
  Serial.print(buffer);
}

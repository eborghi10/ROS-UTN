

/* The ESP32 has four SPi buses, however as of right now only two of
 * them are available to use, HSPI and VSPI. Simply using the SPI API 
 * as illustrated in Arduino examples will use HSPI, leaving VSPI unused.
 * 
 * However if we simply intialise two instance of the SPI class for both
 * of these buses both can be used. However when just using these the Arduino
 * way only will actually be outputting at a time.
 * 
 * Logic analyser capture is in the same folder as this example as
 * "multiple_bus_output.png"
 * 
 * created 30/04/2018 by Alistair Symonds
 */
#include <SPI.h>

static const uint32_t spiClk = 1000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = nullptr;

void setup() {
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(HSPI);
  
  //clock miso mosi ss

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(15, OUTPUT); //VSPI SS

}

// the loop function runs over and over again until power down or reset
void loop() {
  Serial.begin(9600);
  //use the SPI buses
  vspiCommand();
  delay(100);
}

uint8_t spiCalcEvenParity(uint16_t value){
  uint8_t cnt = 0;

  for (uint8_t i = 0; i < 16; i++)
  {
    if (value & 0x1)
    {
      cnt++;
    }
    value >>= 1;
  }
  return cnt & 0x1;
}

void vspiCommand() {
  uint16_t command = 0b0100000000000000;
  //Add a parity bit on the the MSB
  command = command | 0x3FFF;
  
  command |= static_cast<uint16_t>(spiCalcEvenParity(command)<<0xF);

  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(15, LOW); //pull SS slow to prep other end for transfer
  vspi->transfer16(command);  
  digitalWrite(15, HIGH); //pull ss high to signify end of data transfer
//  vspi->endTransaction();

  delay(100);

  digitalWrite(15, LOW);
  uint8_t left_byte = SPI.transfer(0x00);
  uint8_t right_byte = SPI.transfer(0x00);
  digitalWrite(15, HIGH);
  vspi->endTransaction();


  uint16_t response = (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
  Serial.println(response);
}


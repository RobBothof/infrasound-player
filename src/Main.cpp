
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <Bounce.h>
#include <TimeLib.h>
#include <Bounce.h>
#include "RoboTimer.h"

const int buttonPin = 1;
const int gainPin = 39;
const int speedPin = 38;

volatile bool bufferLoaded = false;

elapsedMicros readPotsTimer = 0;

Bounce pushbutton = Bounce(buttonPin, 10); // 10 ms debounce

union
{
  float value;
  uint8_t bytes[4];
} sample;

uint8_t sampleBytesLittleEndian[4];

SdFs sd;

const int DAC_ChipSelectPin = 10;
// const word DAC_Config=0b0001000000000000; //gain * 2 (0-4v)
const word DAC_Config = 0b0011000000000000; // gain * 1 (0-2v)
void DacWrite(uint16_t data);

// int value;
// uint64_t pitch;

int numFiles = 0;
int fileIndexer = 0;

const int maxYears = 5;
const int maxMonths = 12;
const int maxDays = 31;
const int maxHours = 24;
const int firstYear = 2023;

volatile uint32_t iSteps;
volatile uint32_t iStep;

volatile int16_t lastSample = 0;
volatile int16_t nextSample = 0;

int _year = 0;
int _month = 0;
int _day = 0;
int _hour = 0;
int _record = -1;
int _sample = -1;

int numRecords = 0;
int numSamples = 0;
int dataOffset = 0;

int sampleGain = 1;
int sampleSpeed = 1;

float lowest = 0.0f;
float highest = 0.0f;

const uint32_t audioBufferSize = 48000;
String mseedname = "";
FsFile mseed;

uint16_t LSBtoMSB(uint16_t value)
{
  return ((value & 0x00FF) << 8) + ((value & 0xFF00) >> 8);
}

volatile int16_t sampleBuffer[audioBufferSize];
volatile uint32_t sampleBufferWriteIndex = 0;
volatile uint32_t sampleBufferReadIndex = 0;

/// Hardware interrupt (PIR) TIMER triggering MachineLoop Interrupt.
RoboTimer IRQTimer;

FASTRUN void AudioLoopInterp()
{
  if (bufferLoaded) {
    if (sampleBufferReadIndex == sampleBufferWriteIndex)
    {
      /// Buffer is empty, wait for additional samples.
      Serial.println("buffer is empty");
      // Serial.println(sampleBufferReadIndex);
    }
    else
    {
      double interp = ((double) iStep) / ((double) iSteps);

      uint16_t data = min(4095, max(0, (uint16_t) (interp * (double) nextSample + ((1.0 - interp) * (double) lastSample) )));

      DacWrite(data);
    
      iStep++;

      if (iStep >= iSteps) {
        // next sample
        iStep=0;
        lastSample = nextSample;
        nextSample = sampleBuffer[sampleBufferReadIndex];
        sampleBufferReadIndex = (sampleBufferReadIndex + 1) % audioBufferSize;
      }
    }
  }
}
/*
FASTRUN void AudioLoop()
{
  if (bufferLoaded) {
    if (sampleBufferReadIndex == sampleBufferWriteIndex)
    {
      /// Buffer is empty, wait for additional samples.
      Serial.println("buffer is empty");
      // Serial.println(sampleBufferReadIndex);
    }
    else
    {
      uint16_t data =  min(4095, max(0, sampleBuffer[sampleBufferReadIndex]));
      DacWrite(data);

      // next sample
      sampleBufferReadIndex = (sampleBufferReadIndex + 1) % audioBufferSize;
    }
  }
}
*/

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(DAC_ChipSelectPin, OUTPUT);
  digitalWrite(DAC_ChipSelectPin, LOW);

  SPI.begin();

  Serial.begin(115200);

  while (!Serial.dtr() && millis() < 15000)
  {
    // wait for user to start the serial monitor
  }

  Serial.println("");
  Serial.println("===================");
  Serial.println("Walrus player v0.1b");
  Serial.println("===================");

  Serial.println("Reading SD CARD");

  if (!sd.begin(SdioConfig(FIFO_SDIO)))
  {
    Serial.println("Error:: Cannot mount Internal SD-Card.");
  }
  else
  {
    Serial.println("SD Card mounted");
  }

  delay(10);

  sampleGain = 1 + analogRead(gainPin);
  sampleSpeed = 1 + (analogRead(speedPin)>>1);

  double micros = 1000000.0 / ((double)(sampleSpeed*sampleSpeed+85));
  iSteps=(uint16_t) micros;
      
  /// Configure interrupt timer
  /// 150Mhz PIT timer clock
  CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;
  IRQTimer.priority(16);
  // IRQTimer.begin(AudioLoop, micros);
  IRQTimer.begin(AudioLoopInterp, 3.0f);
}

void loop()
{
  if (pushbutton.update())
  {
    if (pushbutton.fallingEdge())
    {
      Serial.println("");
      Serial.println("===================");
      Serial.println("  Reset Playback");
      Serial.println("===================");

      _year = 0;
      _month = 0;
      _day = 0;
      _hour = 0;
      _record = -1;
      _sample = -1;

      sampleBufferWriteIndex = 0;
      sampleBufferReadIndex = 0;    
      bufferLoaded=false;  
    }
  }

  if (_record == -1) // no file open, proceed to next file
  {
    if (_hour == maxHours)
    {
      _hour -= maxHours;
      _day++;
    }

    if (_day == maxDays)
    {
      _day -= maxDays;
      _month++;
    }

    if (_month == maxMonths)
    {
      _month -= maxMonths;
      _year++;
    }

    if (_year == maxYears)
    {
      delay(1);
      // Serial.println("No more mseed.");
      return;
    }

    mseedname = String(firstYear + _year) + "/" + String(_month + 1) + "/" + String(_day + 1) + "/" + String(_hour) + ".mseed";
    if (sd.exists(mseedname))
    {
      // We have a file
      // Open it
      if (!mseed.open(mseedname.c_str(), O_READ))
      {
        Serial.println("ERROR:: could not open mseed file.");
      }

      // Start at first record.
      numRecords = mseed.fileSize() / 4096;
      _record = 0;
      _sample = -1;
    }
    else
    {
      _hour++;
    }
  }
  else
  {
    if (_sample == -1)
    {
      mseed.seek(_record * 4096 + 30);
      mseed.read(&numSamples, 2);

      mseed.seek(_record * 4096 + 44);
      mseed.read(&dataOffset, 2);

      numSamples = LSBtoMSB(numSamples);
      _sample = 0;

      dataOffset = LSBtoMSB(dataOffset);

      // Serial.print("now processing: ");
      // Serial.print(mseedname);
      // Serial.print(" - ");
      // Serial.print(_record);
      // Serial.print(" / ");
      // Serial.print(numRecords);
      // Serial.print(" : ");
      // Serial.print(numSamples);
      // Serial.print(" samples.");
      // Serial.print(" data offset:");
      // Serial.println(dataOffset);
    }
    else
    {
      if (_sample == numSamples)
      {
        _record++;
        _sample = -1;

        if (_record > numRecords)
        {
          // Next file!
          mseed.close();
          _sample = -1;
          _record = -1;
          _hour++;
        }
      }
      else
      {
        // Add sample to buffer if there is room.
        if (((sampleBufferWriteIndex + 1) % audioBufferSize) != sampleBufferReadIndex)
        {
          mseed.seek(dataOffset + _record * 4096 + _sample * 4);
          mseed.read(&sampleBytesLittleEndian, 4);

          // Little Endian to Big Endian :'(

          sample.bytes[0] = sampleBytesLittleEndian[3];
          sample.bytes[1] = sampleBytesLittleEndian[2];
          sample.bytes[2] = sampleBytesLittleEndian[1];
          sample.bytes[3] = sampleBytesLittleEndian[0];

          sampleBuffer[sampleBufferWriteIndex] = sample.value * sampleGain + 2048;

          _sample++;

          // Increment the bufferWriteIndex for an upcoming instruction.
          sampleBufferWriteIndex = (sampleBufferWriteIndex + 1) % audioBufferSize;
        }
        else
        {
          // Serial.println("buffer is full.");
          delayNanoseconds(100);
          if (!bufferLoaded)
          {
            bufferLoaded=true;
            Serial.println("buffer filled, staring playback");
          }
        }
      }
    }
  }

  if (readPotsTimer > 100000)
  {
    readPotsTimer = 0;
    int newGain = 1 + analogRead(gainPin);
    int newSpeed = 1 + (analogRead(speedPin)>>1);
    if (sampleGain < newGain - 2 || sampleGain > newGain + 2)
    {
      sampleGain = newGain;
      // Serial.print("Gain set: ");
      // Serial.println(sampleGain);
    }
    if (sampleSpeed != newSpeed)
    {
      sampleSpeed = newSpeed;

      Serial.print("Speed set: ");
      Serial.print(sampleSpeed*sampleSpeed+85);
      Serial.println(" Hz.");

      double micros = max(3.0, 1000000.0 / ((double)(sampleSpeed*sampleSpeed+85)));
      iSteps = (uint32_t) micros / 3.0;
    }
  }
}

void DacWrite(uint16_t data)
{
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DAC_ChipSelectPin, LOW);
  SPI.transfer16(DAC_Config | data);
  digitalWrite(DAC_ChipSelectPin, HIGH);
  SPI.endTransaction();
}

/*
void printMetaData(int year, int month, int day, int hour)
{
  FsFile mseedFile;
  String filename = String(year) + "/" + String(month) + "/" + String(day) + "/" + String(hour) + ".mseed";

  if (!mseedFile.open(filename.c_str(), O_READ))
  {
    Serial.println("ERROR:: could not open mseed file.");
  }
  char headerQuality;
  char stationIDCODE[5];
  char locationID[2];

  char recordIndex[6];

  uint16_t numSamples;
  uint8_t numBlockets;
  uint16_t dataOffset;
  uint16_t firstBlock;

  uint16_t blocketType;
  uint16_t nextBlocket;
  uint8_t encodingFormat;
  uint8_t wordOrder;
  uint8_t dataRecordLength;

  // uint32_t startOffset = 25*4096;
  uint32_t startOffset = 0;

  mseedFile.seek(startOffset);
  mseedFile.read(&recordIndex, 6);

  mseedFile.seek(startOffset + 6);
  mseedFile.read(&headerQuality, 1);

  mseedFile.seek(startOffset + 8);
  mseedFile.read(&stationIDCODE, 5);

  mseedFile.seek(startOffset + 13);
  mseedFile.read(&locationID, 2);

  mseedFile.seek(startOffset + 30);
  mseedFile.read(&numSamples, 2);

  mseedFile.seek(startOffset + 39);
  mseedFile.read(&numBlockets, 1);

  mseedFile.seek(startOffset + 44);
  mseedFile.read(&dataOffset, 2);

  mseedFile.seek(startOffset + 46);
  mseedFile.read(&firstBlock, 2);

  uint16_t blockIndex = startOffset + 48;

  mseedFile.seek(blockIndex);
  mseedFile.read(&blocketType, 2);

  mseedFile.seek(blockIndex + 2);
  mseedFile.read(&nextBlocket, 2);

  mseedFile.seek(blockIndex + 4);
  mseedFile.read(&encodingFormat, 1);

  mseedFile.seek(blockIndex + 5);
  mseedFile.read(&wordOrder, 1);

  mseedFile.seek(blockIndex + 6);
  mseedFile.read(&dataRecordLength, 1);

  Serial.print("Total Bytes:");
  Serial.println(mseedFile.fileSize());

  Serial.print("Number of Records");
  Serial.println(mseedFile.fileSize() / 4096);

  Serial.print("Record index:");
  Serial.println(recordIndex);

  Serial.print("Data Header indicator:");
  Serial.println(headerQuality);
  Serial.print("Station identifier Code:");
  Serial.println(stationIDCODE);
  Serial.print("Location Identifier:");
  Serial.println(locationID);

  Serial.print("Number of samples:");
  Serial.println(LSBtoMSB(numSamples));
  Serial.print("Number of Blockette:");
  Serial.println(numBlockets);

  Serial.print("Beginning of data:");
  Serial.println(LSBtoMSB(dataOffset));
  Serial.print("First Block:");
  Serial.println(LSBtoMSB(firstBlock));

  Serial.print("Blockette Type:");
  Serial.println(LSBtoMSB(blocketType));
  Serial.print("Next Blockette byte number:");
  Serial.println(LSBtoMSB(nextBlocket));

  Serial.print("Encoding format:");
  Serial.println(encodingFormat);
  Serial.print("Word order:");
  Serial.println(wordOrder);
  Serial.print("Data record length:");
  Serial.println(dataRecordLength);

  mseedFile.close();
}
*/
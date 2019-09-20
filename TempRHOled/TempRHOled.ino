/**
 * @file TempRHOled.ino
 * @author Pratap Karonde (pratap.karonde@gmail.com)
 * @brief  Atmega 328P Program to read temperature and humidity data from DHT sensor and display it on an OLED display 
 * @version 0.1
 * @date 2019-08-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#define ARDUINO 10809

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define NUMFLAKES 10  // Number of snowflakes in the animation example
#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

#define DHTPIN 2        // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22 (AM2302)
#define GRAPH_POINTS 30 // Number of data points on gtaph

#define GRAPH_DURTION_MINS 60 // Minutes to plot on the graph
#define GRAPH_SAMPLE_PERIOD ((float)GRAPH_DURTION_MINS) / (float)GRAPH_POINTS
#define FACTOR 2 // Used for calculation of running average
#define REFRESH_DISPLAY_MS 500
// Repaint every x ms
#define ROLL_SCREEN_MS 5000

#define EEPROM_SIGNATURE (uint16_t)0xFAFBFCFD

#define DATA_MULTIPLIER 100.0f
#define DEBUG 1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t measureEveryMS = 1000;
uint16_t temp_array[GRAPH_POINTS]; // 128 temperature Samples for the graph
uint16_t rh_array[GRAPH_POINTS];   // 128 humiduty  Samples for the graph
uint8_t array_head = 0;
uint8_t filledSize = 0;

uint32_t samplesToAverage = 0; // How many times to add up the average before putting it in the array?
uint32_t samplesAveraged = 0;  // How many have we averaged so far?

uint16_t temp_graph_min = 0;
uint16_t temp_graph_max = 0;
uint16_t rh_graph_min = 0;
uint16_t rh_graph_max = 0;

uint8_t displayScreen = -1;

/**
 * @brief Write the sensor information to Serial Port 
 * 
 * @param dhtsensor Structure containing sensor information 
 */
void serialDumpDHTSensor(sensor_t &dhtsensor)
{
#ifdef DEBUG 
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(dhtsensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(dhtsensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(dhtsensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(dhtsensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(dhtsensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(dhtsensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
#endif 
}

void displayArraySlotData (int slot, uint16_t temp, uint16_t rh)
{

#ifdef DEBUG
        display.clearDisplay();

        display.setCursor(0, 0); // Start at top-left corner
        display.print(F("Slot "));  
        display.print(slot);

        display.setCursor(0, 10); // Start at top-left corner
        display.print(F("Temp "));  
        display.print(temp);

        display.setCursor(0, 20); // Start at top-left corner
        display.print(F("RH   "));  
        display.print(rh);

        delay(1000);

        display.display();
#endif 

}

/**
 * @brief Dump the contents of sensor data array to serial port 
 * 
 */
void serialDumpArrays()
{
#ifdef DEBUG 
  static const char SLOT_LABEL[] = "Slot: ";
  static const char TEMP_LABEL[] = ", TEMP: ";
  static const char RH_LABEL[] = ", RH: ";

  if (filledSize == GRAPH_POINTS)
  {
    // Array has rolled over
    for (int slot = array_head; slot < GRAPH_POINTS; slot++)
    {
      Serial.print(SLOT_LABEL);
      Serial.print(slot);
      Serial.print(TEMP_LABEL);
      Serial.print(temp_array[slot] / DATA_MULTIPLIER);
      Serial.print(RH_LABEL);
      Serial.println(rh_array[slot] / DATA_MULTIPLIER);
    }
  }

  // Array yet to roll
  for (int slot = 0; slot < array_head; slot++)
  {
    Serial.print(SLOT_LABEL);
    Serial.print(slot);
    Serial.print(TEMP_LABEL);
    Serial.print(temp_array[slot] / DATA_MULTIPLIER);
    Serial.print(RH_LABEL);
    Serial.println(rh_array[slot] / DATA_MULTIPLIER);
  }
#endif 
}

/**
 * @brief Read Sensor data stored in the EEPROM 
 * 
 * Checks for valid sensor data by looking for a signature flag 
 */
void readFromEEPROM()
{
  unsigned eepromAddress = 0;

#ifdef DEBUG
  Serial.println(F("Reading from EEPROM"));

  display.clearDisplay();
  
  display.setCursor(0, 0); // Start at top-left corner
  display.print(F("Reading EEPROM"));  

  display.display();
  delay (1000);
#endif 

  uint16_t signature = 0;
  EEPROM.get(eepromAddress, signature);

  filledSize = 0;

  if (signature == EEPROM_SIGNATURE)
  {
    eepromAddress += sizeof(EEPROM_SIGNATURE);
    EEPROM.get(eepromAddress, filledSize);
    eepromAddress += sizeof(filledSize);

    if (filledSize >= GRAPH_POINTS)
      filledSize = GRAPH_POINTS;

    Serial.println(filledSize);
  }
#ifdef DEBUG
  else
  {
    display.clearDisplay();
    display.setCursor(0, 0); // Start at top-left corner
    display.print(F("Invalid Signature "));  
    display.print (signature);
    delay(5000);
  }
#endif     

#ifdef DEBUG
  display.clearDisplay();
  display.setCursor(0, 0); // Start at top-left corner
  display.print(F("Stored Slots: "));  
  display.print (filledSize);
  display.display();
  delay (5000);
#endif 

  if (filledSize)
  {
    // Array yet to roll
    for (int slot = 0; slot < filledSize; slot++)
    {
      Serial.println(slot);
      if (eepromAddress < EEPROM.length())
      {
        EEPROM.get(eepromAddress, temp_array[slot]);
        eepromAddress += sizeof(temp_array[slot]);
        Serial.println(temp_array[slot]);

        EEPROM.get(eepromAddress, rh_array[slot]);
        eepromAddress += sizeof(rh_array[slot]);
        Serial.println(rh_array[slot]);

        displayArraySlotData (slot, temp_array[slot], rh_array[slot]);
      }
      else
      {
#ifdef DEBUG
        Serial.print(F("EEPROM overflow: "));
        Serial.println(EEPROM.length());
        display.clearDisplay();
        display.setCursor(0, 0); // Start at top-left corner
        display.print(F("Overflow "));  
        display.print(EEPROM.length());
        display.display();
        delay (5000);
#endif         
        break;
      }
    }
  }
  else
  {
#ifdef DEBUG
        display.clearDisplay();
        display.setCursor(0, 0); // Start at top-left corner
        display.print(F("No data in EEPROM"));  
        display.display();
        delay (5000);
#endif         

    for (int slot = 0; slot < GRAPH_POINTS; slot++)
    {
      temp_array[slot] = 0;
      rh_array[slot] = 0;
    }
  }

  array_head = filledSize;
  getArrayMinMax(temp_array, &temp_graph_min, &temp_graph_max);

#ifdef DEBUG
  display.clearDisplay();

  display.setCursor(0, 0); // Start at top-left corner
  display.print(F("Temp Max  "));  
  display.print(temp_graph_max);

  display.setCursor(0, 10); // Start at top-left corner
  display.print(F("Temp Min  "));  
  display.print(temp_graph_min);
  display.display();
#endif 

  if (temp_graph_max == 0)
    temp_graph_max = 50*DATA_MULTIPLIER;

  if (temp_graph_min == 0)
    temp_graph_min = temp_graph_max;

  int range_adjust = (temp_graph_max - temp_graph_min) / 10.0;
#ifdef DEBUG

        display.setCursor(0, 20); // Start at top-left corner
        display.print(F("Range Adj "));  
        display.print(range_adjust);

        display.display();
        delay (5000);
#endif         

  if (temp_graph_max > range_adjust)
    temp_graph_min -= range_adjust;
  else
      temp_graph_min = 0;
    
  temp_graph_max += range_adjust;

  // Compute RH Min and Max
  getArrayMinMax(rh_array, &rh_graph_min, &rh_graph_max);
#ifdef DEBUG
        display.clearDisplay();

        display.setCursor(0, 0); // Start at top-left corner
        display.print(F("RH Max    "));  
        display.print(rh_graph_max);

        display.setCursor(0, 10); // Start at top-left corner
        display.print(F("RH Min    "));  
        display.print(rh_graph_min);
#endif 

  if (rh_graph_max == 0)
    rh_graph_max = 100*DATA_MULTIPLIER;

  if (rh_graph_min == 0)
    rh_graph_min == rh_graph_max;

  range_adjust = (rh_graph_max - rh_graph_min) / 10.0;


#ifdef DEBUG

        display.setCursor(0, 20); // Start at top-left corner
        display.print(F("Range Adj "));  
        display.print(range_adjust);

        display.display();
        delay (5000);
#endif  

  if (rh_graph_min > range_adjust)
    rh_graph_min -= range_adjust;
  else
    rh_graph_min = 0;

  rh_graph_max += range_adjust;
}

/**
 * @brief Save temperature and humidty data into EEPROM 
 * 
 * @param bCleanup If set to true, it will initialize the EEPROM content. Use this on a new chip
 */
void saveToEEPROM(bool bCleanup = false)
{
  unsigned eepromAddress = 0;

  Serial.println(F("Save to EEPROM"));

  if (bCleanup)
    filledSize = 0;

  EEPROM.put(eepromAddress, EEPROM_SIGNATURE);
  eepromAddress += sizeof(EEPROM_SIGNATURE);

  EEPROM.put(eepromAddress, filledSize);
  eepromAddress += sizeof(filledSize);

  Serial.println(filledSize);

  if (filledSize)
  {
    if (filledSize == GRAPH_POINTS)
    {
      // Array has rolled over
      for (int slot = array_head; slot < GRAPH_POINTS; slot++)
      {
        if (eepromAddress < EEPROM.length())
        {
          EEPROM.put(eepromAddress, temp_array[slot]);
          eepromAddress += sizeof(temp_array[slot]);

          EEPROM.put(eepromAddress, rh_array[slot]);
          eepromAddress += sizeof(rh_array[slot]);
          displayArraySlotData (slot, temp_array[slot], rh_array[slot]);          
        }
        else
        {
          Serial.print(F("EEPROM overflow: "));
          Serial.println(EEPROM.length());
          break;
        }
      }
    }

    // Array yet to roll
    for (int slot = 0; slot < array_head; slot++)
    {
      if (eepromAddress < EEPROM.length())
      {
        EEPROM.put(eepromAddress, temp_array[slot]);
        eepromAddress += sizeof(temp_array[slot]);

        EEPROM.put(eepromAddress, rh_array[slot]);
        eepromAddress += sizeof(rh_array[slot]);
        displayArraySlotData (slot, temp_array[slot], rh_array[slot]);
      }
      else
      {
        Serial.print(F("EEPROM overflow: "));
        Serial.println(EEPROM.length());
        break;
      }
    }
  }
}

/**
 * @brief Get the Array Min Max object
 * 
 * Scan through the array and find min and max values
 * TODO: Is there any other faster way to do this? 
 * 
 * @param array Source Array to scan 
 * @param min   Pointer to the variable that will recieve the min value 
 * @param max   Pointer to the variable that will receive the max value 
 */
void getArrayMinMax(const uint16_t *array, uint16_t *min, uint16_t *max)
{
  static const char comma[] = ", ";

  *max = 0;
  *min = 0;

  if (array)
  {
    if (filledSize == GRAPH_POINTS)
    {
      // Array has rolled over
      for (int slot = array_head; slot < GRAPH_POINTS; slot++)
      {
        if (slot == array_head)
          *min = array[slot];

        if (*max < array[slot])
          *max = array[slot];

        if (*min > array[slot])
          *min = array[slot];

        Serial.print(array[slot]);
        Serial.print(comma);
        Serial.print(*min);
        Serial.print(comma);
        Serial.println(*max);
      }
    }

    // Array yet to roll
    for (int slot = 0; slot < array_head; slot++)
    {
      if (*min == 0 && slot == 0)
        *min = array[slot];

      if (*max < array[slot])
        *max = array[slot];

      if (*min > array[slot])
        *min = array[slot];

      Serial.print(array[slot]);
      Serial.print(comma);
      Serial.print(*min);
      Serial.print(comma);
      Serial.println(*max);
    }
  }
  else
    Serial.println(F("Empty Array Received!"));
}

void drawScreen0(float currrenTemp, float currentRH)
{
  display.setCursor(0, 0); // Start at top-left corner
  display.print(F("Temp: "));
  display.print(currrenTemp);
  display.print(F("C"));

  display.setCursor(0, 10);
  display.print(F("Humidity: "));
  display.print(currentRH);
  display.print(F("%"));

  int barWidth = int((currentRH * 124.0) / DATA_MULTIPLIER);
  display.fillRect(2, 22, barWidth, 8, WHITE);

  // Draw RH Graph
  display.drawRect(0, 20, 127, 12, WHITE);
}

void drawTempGraphScreen()
{
  // Draw Full displayScreen box
  display.drawRect(0, 0, display.width(), display.height(), WHITE);
  display.setTextWrap(false);

  display.setCursor(2, 2);
  display.print(temp_graph_max / DATA_MULTIPLIER);

  display.setCursor(100, 23);
  display.print(temp_graph_min / DATA_MULTIPLIER);

  display.setCursor(120, 2);
  display.print(F("C"));

  int last_x = 0;
  int last_y = display.height() / 2;

  for (int dataPoint = 0; dataPoint < min(filledSize, GRAPH_POINTS); dataPoint++)
  {
    int x = dataPoint * ((float)(display.width() - 2) / (float)GRAPH_POINTS) + 1;
    int y = display.height() - ((float)(temp_array[dataPoint] - temp_graph_min) * (float)32) / ((float)temp_graph_max - temp_graph_min);
    display.drawLine(last_x, last_y, x, y, WHITE);
    last_x = x;
    last_y = y;
  }
}

void drawRHGraphScreen()
{
  // Draw Full displayScreen box
  display.drawRect(0, 0, display.width(), display.height(), WHITE);

  display.setCursor(2, 2);
  display.print(rh_graph_max / DATA_MULTIPLIER);

  display.setCursor(120, 2);
  display.print(F("%"));

  display.setCursor(100, 23);
  display.print(rh_graph_min / DATA_MULTIPLIER);

  display.setTextWrap(false);

  int last_x = 0;
  int last_y = display.height() / 2;

  for (int dataPoint = 0; dataPoint < min(filledSize, GRAPH_POINTS); dataPoint++)
  {
    int x = dataPoint * ((float)(display.width() - 2) / (float)GRAPH_POINTS) + 1;
    int y = display.height() - ((float)(rh_array[dataPoint] - rh_graph_min) * (float)32) / ((float)rh_graph_max - rh_graph_min);
    display.drawLine(last_x, last_y, x, y, WHITE);
    last_x = x;
    last_y = y;
  }
}

/**
 * @brief Read Temperature and humidity data from the sensor 
 * 
 * @param currentTemp Pointer to a variable where the temperature data will be stored 
 * @param currentRH   Pointer to a variable where the relative humidity data will be stored 
 */
void readTempAndHumidity(float *currentTemp, float *currentRH)
{
  static float temp_average = 0;
  static float rh_average = 0;

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    //Serial.println(F("Error reading temperature!"));
  }
  else
    *currentTemp = event.temperature;

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    //Serial.println(F("Error reading humidity"));
  }
  else
    *currentRH = event.relative_humidity;

  if (samplesAveraged == 0)
  {
    samplesAveraged++;
    temp_average = *currentTemp;
    rh_average = *currentRH;
  }
  else
  {
    // Using running average method described here to calcualte running average of temperature and relative humidity
    // https://stackoverflow.com/questions/12636613/how-to-calculate-moving-average-without-keeping-the-count-and-data-total
    samplesAveraged++;
    temp_average = temp_average + (*currentTemp - temp_average) / min(samplesAveraged, FACTOR);
    rh_average = rh_average + (*currentRH - rh_average) / min(samplesAveraged, (uint32_t)FACTOR);

    if (samplesAveraged > samplesToAverage)
    {
      // Averaged enough time, put it in an array
      samplesAveraged = 0;

      temp_array[array_head] = uint16_t(temp_average * 100);
      rh_array[array_head] = uint16_t(rh_average) * 100;

      array_head++;
      if (filledSize < GRAPH_POINTS)
        filledSize++;

      if (array_head >= GRAPH_POINTS)
        array_head = 0;

      serialDumpArrays();

      // Compute Temp Min and Max
      getArrayMinMax(temp_array, &temp_graph_min, &temp_graph_max);
      int range_adjust = (temp_graph_max - temp_graph_min) / 10.0;
      temp_graph_min -= range_adjust;
      temp_graph_max += range_adjust;

      // Compute RH Min and Max
      getArrayMinMax(rh_array, &rh_graph_min, &rh_graph_max);
      range_adjust = (rh_graph_max - rh_graph_min) / 10.0;
      rh_graph_min -= range_adjust;
      rh_graph_max += range_adjust;

      // Save data to EEPROM so that it is not lost with power
      saveToEEPROM();
    }
  }
}

/**
 * @brief Atmega setup code
 * 
 * This code runs first when power is applied to the circuit. It configures the sensor and the OLED display 
 * 
 */
void setup()
{
  Serial.begin(115200);
  Serial.println(F("Starting.."));

  pinMode ( 0, OUTPUT );
  pinMode ( 1, OUTPUT );
  pinMode (13, OUTPUT );

  digitalWrite (0, HIGH);
  digitalWrite (1, HIGH);
  digitalWrite (13, LOW);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  dht.begin();

  Serial.println(F("Initalization done.."));

  // Print temperature sensor details.
  sensor_t dhtsensor;
  dht.temperature().getSensor(&dhtsensor);
  serialDumpDHTSensor(dhtsensor);

  // Print humidity sensor details.
  dht.humidity().getSensor(&dhtsensor);
  serialDumpDHTSensor(dhtsensor);

  // Set delay between sensor readings based on sensor details.
  measureEveryMS = dhtsensor.min_delay / 1000;

  // How many numbers to average before putting it in an array slot
  samplesToAverage = (float)(GRAPH_SAMPLE_PERIOD * 60) / ((float)measureEveryMS / (float)1000);

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner

  //saveToEEPROM(true);
  readFromEEPROM();
  display.display();
  delay(500);
}

unsigned long lastMeasureMS = 0;
unsigned long lastDisplayMS = 0;
unsigned long lastScreenRollMS = 0;

float currrenTemp = 0;
float currentRH = 0;

/**
 * @brief Atmega loop code. This loop runs contineously till the circuit has power 
 * 
 */
void loop()
{
  unsigned long newMS = 0;
  newMS = millis();

  if (!lastMeasureMS || (newMS - lastMeasureMS) > measureEveryMS)
  {
    lastMeasureMS = newMS;
    readTempAndHumidity(&currrenTemp, &currentRH);
  }

  if (lastScreenRollMS == 0 || (newMS - lastScreenRollMS) > ROLL_SCREEN_MS)
  {
    lastScreenRollMS = newMS;
    displayScreen++;

    if (displayScreen > 2)
      displayScreen = 0;
  }

  if (!lastDisplayMS || (newMS - lastDisplayMS) > REFRESH_DISPLAY_MS)
  {
    lastDisplayMS = newMS;
    display.clearDisplay();

    switch (displayScreen)
    {
    case 0:
      drawScreen0(currrenTemp, currentRH);
      break;

    case 1:
      drawTempGraphScreen();
      break;

    case 2:
      drawRHGraphScreen();
      break;

    default:
      break;
    }

    display.display(); // send buffer to displayScreen
  }

  delay(1);
}

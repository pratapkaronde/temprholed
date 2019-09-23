/**
   @file TempRHOled.ino
   @author Pratap Karonde (pratap.karonde@gmail.com)
   @brief  Atmega 328P Program to read temperature and humidity data from DHT sensor and display it on an OLED display
   @version 0.1
   @date 2019-08-13

   @copyright Copyright (c) 2019

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
#include <assert.h>


#define GRAPH_POINTS 100 // Number of data points on gtaph
#define GRAPH_DURTION_MINS 24*60 // Minutes to plot on the graph
#define GRAPH_SAMPLE_PERIOD ((float)GRAPH_DURTION_MINS) / (float)GRAPH_POINTS
#define FACTOR 2 // Used for calculation of running average
#define REFRESH_DISPLAY_MS 500
#define ROLL_SCREEN_MS 5000
#define GRAPH_ADJUST_PERCENT 15 // Keep graph from touching the top and bottom axis by adding some empty space 
#define EEPROM_SIGNATURE (uint16_t)0xFAFBFCFD
#define DATA_MULTIPLIER 100.0f // Data is stored in the EEPROM with this multiplier 
//#define DEBUG 1

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Declaration for Temp / RH Sensor 
#define DHTPIN 2        // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t measureEveryMS = 1000;
int16_t temp_array[GRAPH_POINTS]; // 128 temperature Samples for the graph
int16_t rh_array[GRAPH_POINTS];   // 128 humiduty  Samples for the graph

uint8_t arrayTail = 0;
//#define filledSize (arrayTail+1)

uint32_t samplesToAverage = 0; // How many times to add up the average before putting it in the array?
uint32_t samplesAveraged = 0;  // How many have we averaged so far?

int16_t temp_graph_min = 0;
int16_t temp_graph_max = 0;
int16_t rh_graph_min = 0;
int16_t rh_graph_max = 0;

uint8_t displayScreen = -1;

/**
   @brief Write the sensor information to Serial Port

   @param dhtsensor Structure containing sensor information
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

  display.display();
  delay(1000);
#endif

}

/**
   @brief Dump the contents of sensor data array to serial port

*/
void serialDumpArrays()
{
#ifdef DEBUG
  static const char SLOT_LABEL[] = "Slot: ";
  static const char TEMP_LABEL[] = ", TEMP: ";
  static const char RH_LABEL[] = ", RH: ";

  // Array yet to roll
  for (int slot = 0; slot <= arrayTail; slot++)
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
   @brief Read Sensor data stored in the EEPROM

   Checks for valid sensor data by looking for a signature flag
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

  arrayTail = 0;

  if (signature == EEPROM_SIGNATURE)
  {
    eepromAddress += sizeof(EEPROM_SIGNATURE);
    EEPROM.get(eepromAddress, arrayTail);
    eepromAddress += sizeof(arrayTail);

    if (arrayTail >= GRAPH_POINTS)
      arrayTail = GRAPH_POINTS - 1;

    Serial.println(arrayTail);
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
  display.print(F("Filled Size: "));
  display.print (arrayTail);
  display.display();
  delay (5000);
#endif

  if (arrayTail)
  {
    for (int slot = 0; slot < arrayTail; slot++)
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

  getArrayMinMax(temp_array, arrayTail, GRAPH_ADJUST_PERCENT, 50 * DATA_MULTIPLIER,  &temp_graph_min, &temp_graph_max);

#ifdef DEBUG
  display.clearDisplay();

  display.setCursor(0, 0); // Start at top-left corner
  display.print(F("Temp Max  "));
  display.print(temp_graph_max);

  display.setCursor(0, 10); // Start at top-left corner
  display.print(F("Temp Min  "));
  display.print(temp_graph_min);
  display.display();
  delay(3000);
#endif

  // Compute RH Min and Max
  getArrayMinMax(rh_array, arrayTail, GRAPH_ADJUST_PERCENT, 100 * DATA_MULTIPLIER,  &rh_graph_min, &rh_graph_max);

#ifdef DEBUG
  display.clearDisplay();

  display.setCursor(0, 0); // Start at top-left corner
  display.print(F("RH Max    "));
  display.print(rh_graph_max);

  display.setCursor(0, 10); // Start at top-left corner
  display.print(F("RH Min    "));
  display.print(rh_graph_min);
  delay(3000);
#endif

}

/**
   @brief Save temperature and humidty data into EEPROM

   @param bCleanup If set to true, it will initialize the EEPROM content. Use this on a new chip
*/
void saveToEEPROM(bool bCleanup = false)
{
  unsigned eepromAddress = 0;

  Serial.println(F("Saving to EEPROM"));

  if (bCleanup)
    arrayTail = 0;

  EEPROM.put(eepromAddress, EEPROM_SIGNATURE);
  eepromAddress += sizeof(EEPROM_SIGNATURE);

  EEPROM.put(eepromAddress, arrayTail);
  eepromAddress += sizeof(arrayTail);

  for (int slot = 0; slot < arrayTail; slot++)
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

/**
   @brief Get the Array Min Max object

   Scan through the array and find min and max values
   TODO: Is there any other faster way to do this?

   @param array Source Array to scan
   @param min   Pointer to the variable that will recieve the min value
   @param max   Pointer to the variable that will receive the max value
*/
void getArrayMinMax(const int16_t *array, const uint8_t arraySize, const uint8_t rangeAdjustPercent, const int16_t defaultVal, int16_t *min, int16_t *max)
{
  static const char comma[] = ", ";

  *max = 0;
  *min = 0;

  if (array)
  {
    for (int slot = 0; slot < arraySize; slot++)
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

    if (*min == 0 && *max == 0)
    {
      *min = defaultVal;
      *max = defaultVal;
    }

#ifdef DEBUG
    display.clearDisplay();

    display.setCursor(0, 0); // Start at top-left corner
    display.print(F("Array Min/Max"));

    display.setCursor(0, 10); // Start at top-left corner
    display.print(F("Min  "));
    display.print(*min);

    display.setCursor(0, 20); // Start at top-left corner
    display.print(F("Max  "));
    display.print(*max);

    display.display();
    delay(3000);
#endif

    if (rangeAdjustPercent)
    {
      // Adjust min and max values so that all data shows up nicely in the graphs

      int rangeAdjustValue = (*max - *min) / rangeAdjustPercent;

      *min -= rangeAdjustValue;
      *max += rangeAdjustValue;

#ifdef DEBUG
      display.clearDisplay();

      display.setCursor(0, 0); // Start at top-left corner
      display.print(F("RangAdjVal "));
      display.print(rangeAdjustValue);

      display.setCursor(0, 10); // Start at top-left corner
      display.print(F("Min  "));
      display.print(*min);

      display.setCursor(0, 20); // Start at top-left corner
      display.print(F("Max  "));
      display.print(*max);

      display.display();
      delay(3000);
#endif

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

  for (int dataPoint = 0; dataPoint < min(arrayTail, GRAPH_POINTS); dataPoint++)
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

  for (int dataPoint = 0; dataPoint < min(arrayTail, GRAPH_POINTS); dataPoint++)
  {
    int x = dataPoint * ((float)(display.width() - 2) / (float)GRAPH_POINTS) + 1;
    int y = display.height() - ((float)(rh_array[dataPoint] - rh_graph_min) * (float)32) / ((float)rh_graph_max - rh_graph_min);
    display.drawLine(last_x, last_y, x, y, WHITE);
    last_x = x;
    last_y = y;
  }
}

/**
   @brief Read Temperature and humidity data from the sensor

   @param currentTemp Pointer to a variable where the temperature data will be stored
   @param currentRH   Pointer to a variable where the relative humidity data will be stored
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
      // Averaged enough samples, put them in the array
      samplesAveraged = 0;

      if (arrayTail >= GRAPH_POINTS)
      {
        arrayTail = GRAPH_POINTS - 1;

        // Array is all filled up. Shift contents to make space for the new sample
        for ( int index = 0; index < (GRAPH_POINTS - 1); index++)
        {
          temp_array[index] = temp_array[index + 1];
          rh_array[index] = rh_array[index + 1];
        }
        assert(arrayTail == (GRAPH_POINTS - 1));
      }

      temp_array[arrayTail] = uint16_t(temp_average * 100);
      rh_array[arrayTail] = uint16_t(rh_average) * 100;

      if (arrayTail < (GRAPH_POINTS - 1))
        arrayTail++;

      serialDumpArrays();

      // Compute Temp Min and Max
      getArrayMinMax(temp_array, arrayTail, GRAPH_ADJUST_PERCENT, 0, &temp_graph_min, &temp_graph_max);


      // Compute RH Min and Max
      getArrayMinMax(rh_array, arrayTail, GRAPH_ADJUST_PERCENT, 0, &rh_graph_min, &rh_graph_max);

      // Save data to EEPROM so that it is not lost with power
      saveToEEPROM();
    }
  }
}

/**
   @brief Atmega setup code

   This code runs first when power is applied to the circuit. It configures the sensor and the OLED display

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
   @brief Atmega loop code. This loop runs contineously till the circuit has power

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

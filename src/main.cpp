#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h> // MQTT
#include <U8g2lib.h>      // Display
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include <KeyValueReader.h>
#include "arduinoFFT.h"

arduinoFFT FFT;

// We can globally disable Serial comms by undefining this
#define USE_SERIAL
void print(const char *text)
{
#ifdef USE_SERIAL
  Serial.print(text);
#endif
}

void println(const char *text)
{
#ifdef USE_SERIAL
  Serial.println(text);
#endif
}

// The path of the config file on the SD card
#define CONFIG_FILE "/config.ini"

char hostname[32] = "hostname"; // Override in config
const char *ssid = "Searching"; // displayed as a placeholder while not connected

struct WiFiCredentials
{
  char ssid[32]; // 32 seems long enough for SSID or password
  char password[32];
};

#define MAXIMUM_NUMBER_OF_CREDENTIALS 4

const WiFiCredentials *wifiCredentials[MAXIMUM_NUMBER_OF_CREDENTIALS];
size_t wifiCredentialCount = 0; // Ho wmany have actually been defined?

void addWifiCredentials(const char *ssid, const char *password)
{
  // Make copies of the ssid and password variables.
  if (wifiCredentialCount < MAXIMUM_NUMBER_OF_CREDENTIALS)
  {
    WiFiCredentials *credentials = new WiFiCredentials();

    // Ensure the destination arrays are null-terminated.
    strncpy(credentials->ssid, ssid, sizeof(credentials->ssid) - 1);
    credentials->ssid[sizeof(credentials->ssid) - 1] = '\0';

    strncpy(credentials->password, password, sizeof(credentials->password) - 1);
    credentials->password[sizeof(credentials->password) - 1] = '\0';

    // store the new credentials
    wifiCredentials[wifiCredentialCount] = credentials;
    wifiCredentialCount += 1;
  }
}

// MQTT Broker - the actual settings need to be in the config file
char mqttBroker[32] = "mqtt.local";
char mqttUsername[32] = "user";
char mqttPassword[32] = "password";
int mqttPort = 1883;

// MQTT topics we publish on
const char *MQTT_TOPIC_ALERT = "/intercom/active";
const char *MQTT_TOPIC_AUDIO_BASELINE = "/intercom/audio/baseline";
const char *MQTT_TOPIC_AUDIO_SAMPLE_RATE = "/intercom/audio/samplerate";
const char *MQTT_TOPIC_AUDIO_EVENT = "/intercom/audio/event";
const char *MQTT_TOPIC_INFO = "/intercom/info";
const char *MQTT_TOPIC_TIME = "/intercom/time";    // incoming
const char *MQTT_TOPIC_REBOOT = "intercom/reboot"; // incoming

String mqtt_status = "MQTT inactive";

WiFiClient wifiClient;               // The Wifi connection
PubSubClient mqttClient(wifiClient); // The MQTT connection

// We test Wifi and MQTT connections every so often - this is the interval
const unsigned long CONNECTION_TEST_INTERVAL = 5 * 1000; // milliseconds
unsigned long lastConnectionTest = 0UL;                  // timestamp of the last connection test

// This OLED definition works with the Xiao32C3 extension board
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

const byte BUTTON_OPEN = HIGH;
const byte BUTTON_PRESSED = LOW;

const uint8_t MAIN_BUTTON = D1; // the expansion board button
const uint8_t MICROPHONE_PIN = A0;

enum ROW
{
  HOSTNAME_ROW,
  SSID_ROW,
  IP_ROW,
  MQTT_ROW,
  STATUS_ROW,
  NUMBER_OF_ROWS
};

uint8_t mainButtonState = BUTTON_OPEN; // default is button not pressed
char *statusText = (char *)"";         // displayed in the last line of the display

// OLED utility routines
void gotoRowColumn(byte row, byte column)
{
  if (row != -1)
  {
    oled.setFont(u8g2_font_7x13_tf);
    oled.setCursor(column * 13, row * 13);
  }
  else
  {
    oled.setFont(u8g2_font_profont22_tf);
    oled.setCursor(column * 13, row * 13 + 5);
  }
}

void gotoRow(byte row)
{
  gotoRowColumn(row, 0);
}

void displayString(String text)
{
  oled.print(text);
}

void displayStatus()
{
  // clear the display
  oled.clearBuffer();
  // print hostname in top row
  gotoRow(HOSTNAME_ROW);
  displayString(hostname);
  // Then display the SSID of the Wifi network we're connected to
  gotoRow(SSID_ROW);
  displayString(ssid);
  gotoRow(IP_ROW);
  if (WiFi.status() == WL_CONNECTED)
  {
    // display our IP if connected
    displayString(WiFi.localIP().toString());
  }
  else
  {
    // or a status message if not
    displayString("Connecting WiFi");
  }
  // display the MQTT status
  gotoRow(MQTT_ROW);
  displayString(mqtt_status.c_str());
  gotoRow(STATUS_ROW);
  // And finally a general purpose status message
  displayString(statusText);
  // update the display
  oled.sendBuffer();
}

// Called on setup or if Wifi connection has been lost
void setupWifi()
{
  println("Connecting to WiFi");
  for (size_t index = 0; index < wifiCredentialCount; index += 1)
  {
    char buffer[100];
    sprintf(buffer, "%s@%s", wifiCredentials[index]->ssid, wifiCredentials[index]->password);
    println(buffer);
  }
  // We start by connecting to a WiFi network
  WiFi.disconnect(); // just in case
  WiFi.setHostname(hostname);
  boolean wifiConnected = false;
  // Nothing works without Wifi - loop until we are connected
  while (!wifiConnected)
  {
    int wifiFound = -1; // index of the first known network found
    while (wifiFound < 0)
    {
      ssid = "Scanning";
      displayStatus();
      println("Scanning for networks");
      // How many networks are visible?
      int visibleNetworkCount = WiFi.scanNetworks();
      print("Network count: ");
      println(String(visibleNetworkCount, 10).c_str());
      for (int index = 0; index < visibleNetworkCount; index += 1)
      {
        println(WiFi.SSID(index).c_str());
      }
      // Check all found networks
      for (int index1 = 0; index1 < visibleNetworkCount; index1 += 1)
      {
        print("Checking #");
        print(String(index1).c_str());
        print(" ");
        println(WiFi.SSID(index1).c_str());
        // try all known networks
        for (int index2 = 0; index2 < wifiCredentialCount; index2 += 1)
        {
          // Does the visible network have the SSID of this known network?
          if (strcmp(wifiCredentials[index2]->ssid, WiFi.SSID(index1).c_str()) != 0)
          {
            // not equal
            print("Not matching ");
            println(wifiCredentials[index2]->ssid);
          }
          else
          {
            // it does - stop going through our known networks
            wifiFound = index2;
            break;
          }
        }
        if (wifiFound >= 0)
        {
          // stop going through visible networks
          break;
        }
      }
      if (wifiFound >= 0)
      {
        // We are going to try and connect to this network
        ssid = wifiCredentials[wifiFound]->ssid;
        displayStatus();
        WiFi.begin(ssid, wifiCredentials[wifiFound]->password);
        int attempts = 10; // We attempt only a limited number of times
        // we have a WiFi we can try and connect to
        while (WiFi.status() != WL_CONNECTED && attempts > 0)
        {
          attempts -= 1;
          delay(500);
        }
        wifiConnected = WiFi.status() == WL_CONNECTED;
        // If the Wifi is not connected we start the search all over again
      }
    }
  }

  // Finally, the Wifi is connected
  println("WiFi connected");
  println("IP address: ");
  println(WiFi.localIP().toString().c_str());
  println(WiFi.getHostname());
  // Update the display with SSID and IP address
  displayStatus();
}

// publish an (long) integer to the MQTT broker
void publishInteger(const char *topic, long value, bool retain)
{
  if (mqttClient.connected())
  {
    char message[20];
    sprintf(message, "%d", value);
    boolean result;
    result = mqttClient.publish(topic, (const uint8_t *)message, strlen(message), retain);
    /*
    print("Publish ");
    print(topic);
    print("=");
    print(message);
    print(" : ");
    println(result == true ? "OK" : "FAIL");
    */
  }
  else
  {
    mqtt_status = "MQTT disconnected";
    displayStatus();
  }
}

// publish an (long) integer to the MQTT broker - default is to retain the value
void publishInteger(const char *topic, long value)
{
  publishInteger(topic, value, true);
}

// publish a string to the MQTT broker
void publishString(const char *topic, char *value, bool retain)
{
  if (mqttClient.connected())
  {
    boolean result;
    result = mqttClient.publish(topic, (const uint8_t *)value, strlen(value), retain);
    /*
    print("Publish ");
    print(topic);
    print("=");
    print(value);
    print(" : ");
    println(result == true ? "OK" : "FAIL");
    */
  }
  else
  {
    mqtt_status = "MQTT disconnected";
    displayStatus();
  }
}

// publish a string to the MQTT broker - default is to retain the value
void publishString(const char *topic, char *value)
{
  publishString(topic, value, true);
}

void publishAudioEvent(const char *topic, uint16_t loudness, double peak, double inRangePercent, boolean retain)
{
  // We publish an event containing loudness and peak as a JSON object.
  // Instead of installing yet another library for this one purpose - let's cheat
  char buffer[64];
  sprintf(buffer, "{\"loudness\": %d,\"peak\": %.0f, \"inrange\": %.2f}", loudness, peak, inRangePercent);
  publishString(topic, buffer, retain);
}

void publishAudioEvent(const char *topic, uint16_t loudness, double peak, double inRangePercent)
{
  publishAudioEvent(topic, loudness, peak, inRangePercent, true);
}

// This will reboot the ESP32 after a 5 second countdown
void reboot()
{
  // Maybe Node Red wants to know about it?
  publishString(MQTT_TOPIC_INFO, (char *)"Rebooting...");
  for (uint8_t index = 5; index > 0; index -= 1)
  {
    char buffer[20];
    sprintf(buffer, "Rebooting in %d", index);
    statusText = buffer;
    displayStatus();
    delay(1000);
  }
  ESP.restart();
}

// Configuration of the OLED display
void setupOLED()
{
  println("Start OLED");
  oled.begin();
  println("Setup OLED");
  oled.setFont(u8g2_font_7x13_tf);
  oled.setFontRefHeightExtendedText();
  oled.setDrawColor(1);
  oled.setFontPosTop();
  oled.setFontDirection(0);
  // The display is ready - update it for the first time
  displayStatus();
}

// Setup for the SD card that holds the config file
void setupSDCard()
{
  println("Initializing SD Card");
  pinMode(D2, OUTPUT);
  if (!SD.begin(D2))
  {
    println("Failed");
    statusText = (char *)"SD card not found";
    displayStatus();
    return;
  }
  println("SD Card Ready.");
}

// Variables for dealing with all things audio
// Larger values than 256 lead to crashes. 256 works and we want as many samples as possible.
#define SAMPLES 256
uint16_t detectInterval = 50;     // time in ms between acquiring audio samples
uint16_t publishInterval = 1000;  // Publish once a second to keep graphs moving
uint16_t sampleFrequency = 14000; // in Hertz
uint16_t oneSampleMicros = 0;     // computed later - how long does one sample take at the sample frequency
uint16_t samples[SAMPLES];        // The buffer holding the audio samples
uint16_t baseLine = 0;            // The measured audio samples vary around this value.
uint16_t loudness = 0;            // A measurement of how loud the signal is. Needs looking at.
unsigned long lastCheck;          // timestamp of the last time we acquired audio data
unsigned long lastPublish;        // timestamp when last forced a publish of audio data
unsigned long loudStart = 0;      // timestamp of the first loud event detected after silence
unsigned long minimumRingDuration = 2000;
uint16_t loudThreshold = 0.5;     // When is a signal "loud" - above [0..1] times baseLine - don't even FFT anything quieter
double minimumRingLoudness = 0.6; // Again [0..1] times baseLine
uint16_t minimumRingFrequency = 1800;
double minimumPercentInRanges = 0; // works mostly with this, but pick higher value in config

// We store frequency ranges for detection in data structures like this:
struct Range
{
  double low;
  double high;
};

#define MAXIMUM_NUMBER_OF_RANGES 32
const Range *ranges[MAXIMUM_NUMBER_OF_RANGES]; // 32 should be enough
size_t rangeCount = 0;

// Called when a new range is found in the config file
void addRange(double low, double high)
{
  if (rangeCount < MAXIMUM_NUMBER_OF_RANGES)
  {
    Range *range = new Range();

    range->low = low;
    range->high = high;

    ranges[rangeCount] = range;
    rangeCount += 1;
  }
}
// read audio data from the ADC into the samples buffer at the sampleFrequency
void readSamples()
{
  if (oneSampleMicros == 0)
  {
    oneSampleMicros = 1.0 / sampleFrequency * 1000000.0; // in microseconds
  }
  unsigned long start = micros(); // to measure how long the acquisition took
  unsigned long next = start;     // acquisition is duw now
  // loop until all the samples have been taken
  for (uint16_t index = 0; index < SAMPLES; index += 1)
  {
    uint16_t loopCount = 0;
    while (next > micros())
    {
      loopCount += 1;
      if (loopCount > 100)
      {
        println("Overflow");
        break;
      }
      // loop until next sample is due
    }
    // Read the sample from the ADC
    samples[index] = analogRead(MICROPHONE_PIN);
    // the next sample is due at this time:
    next = (start + (index + 1) * oneSampleMicros);
  }
}

// calculate some measure of loudness. This needs improving - or not
void computeLoudness()
{
  // crude calculation - will hopefully do for our purposes
  double sum = 0;
  for (uint16_t index = 0; index < SAMPLES; index += 1)
  {
    sum += abs(samples[index] - baseLine);
  }
  // and
  sum /= SAMPLES;
  loudness = static_cast<uint16_t>(sum);
}

void setupAudio()
{
  char buffer[32]; // the usual buffer for generating messages
  println("Audio setup");
  sprintf(buffer, "Reading %d samples", SAMPLES);
  println(buffer);
  // We do one reading of samples to calibrate
  unsigned long start = micros();
  readSamples();
  unsigned long end = micros();
  sprintf(buffer, "Took %2f ms", (end - start) / 1000.0);
  println(buffer);
  // Calculate the baseline and max/min
  double sum = 0;
  uint16_t min = 65535; // very big minimum
  uint16_t max = 0;     // very small maximum
                        // // look at all the samples taken
  for (uint16_t index = 0; index < SAMPLES; index += 1)
  {
    uint16_t sample = samples[index];   // one sample
    sum += static_cast<double>(sample); // add all the samples
    if (sample > max)
    {
      max = sample; // new maximum
    }
    if (sample < min)
    {
      min = sample; // new minimum
    }
  }
  // The baseline is the average of all readings
  // The values of our sample readings should be in the range [0...2*baseLine]
  // The baseLine is basically the zero line of the samples
  baseLine = static_cast<uint16_t>(sum / SAMPLES);
  // and gets published on MQTT
  publishInteger(MQTT_TOPIC_AUDIO_BASELINE, baseLine);
  sprintf(buffer, "%d..%d..%d", min, baseLine, max);
  println(buffer);
}

// called when an MQTT topic we subscribe to gets an update
void mqttCallback(char *topic, byte *message, unsigned int length)
{
  // Convert the byte* message to a String
  String messageString;
  for (int index = 0; index < length; index += 1)
  {
    messageString += (char)message[index];
  }
  print("Received ");
  print(topic);
  print("=");
  println(messageString.c_str());
  if (strcmp(topic, MQTT_TOPIC_TIME) == 0)
  {
    statusText = (char *)messageString.c_str();
    displayStatus();
  }
  else if (strcmp(topic, MQTT_TOPIC_REBOOT) == 0)
  {
    reboot();
  }
}

// Setup the MQTT connection to the broker
void setupMQTT()
{
  println("Connection MQTT");
  mqtt_status = String("MQTT connecting");
  displayStatus();
  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  // if the client is already connected - do nothing
  if (!mqttClient.connected())
  {
    // make up a unique client id
    String clientId = String(hostname) + "-" + String(WiFi.macAddress());
    print("Client ");
    println(clientId.c_str());
    // Now try to connect to the MQTT broker
    if (mqttClient.connect(clientId.c_str(), mqttUsername, mqttPassword))
    {
      println("MQTT broker connected");
      mqtt_status = String(mqttBroker) + ":" + String(mqttPort);
      displayStatus();
      // Make sure we publish stuff so they are available in Node Red right away
      publishInteger(MQTT_TOPIC_AUDIO_BASELINE, baseLine);
      publishInteger(MQTT_TOPIC_AUDIO_SAMPLE_RATE, sampleFrequency);
      mqttClient.subscribe(MQTT_TOPIC_TIME);
    }
    else
    {
      // Update the OLED with an error message
      print("failed with state ");
      print(String(mqttClient.state()).c_str());
      mqtt_status = String("MQTT failed: ") + String(mqttClient.state());
      displayStatus();
    }
  }
}

// Called when the intercom has been detected to be active
void handleIntercomOn()
{
  println("DING");
  if (!mqttClient.connected())
  {
    setupMQTT();
  }
  publishInteger(MQTT_TOPIC_ALERT, 1);
}

// Called when the intercom is no longer ringing.
// Needed to reset the value published on MQTT
void handleIntercomOff()
{
  println("DONG");
  if (!mqttClient.connected())
  {
    setupMQTT();
  }
  publishInteger(MQTT_TOPIC_ALERT, 0);
}

// A press on the button simulates the intercom ringing
void onMainButtonPressed()
{
  handleIntercomOn();
  statusText = (char *)"DING DONG!";
  displayStatus();
}

// Releasing the button simulates the intercom no longer ringing
void onMainButtonReleased()
{
  handleIntercomOff();
  statusText = (char *)"";
  displayStatus();
}

// Deal with a button press/release
void handleButton(byte button, uint8_t *state, void (*onPress)(), void (*onRelease)())
{
  if (digitalRead(button) == BUTTON_OPEN)
  {
    // button currently not pressed (open)
    if (*state == BUTTON_PRESSED)
    {
      // last time around it was however pressed - change the state
      *state = BUTTON_OPEN;
      // invoke callback if defined
      if (onRelease != NULL)
      {
        (*onRelease)();
      }
    }
  }
  else
  {
    // button is currently pressed
    if (*state == BUTTON_OPEN)
    {
      // last time around it was however not pressed - change the state
      *state = BUTTON_PRESSED;
      // invoke callback if defined
      if (onPress != NULL)
      {
        (*onPress)();
      }
    }
  }
}

// The callback called every time the KeyValueReader finds a key/value in the config file
void keyValuePair(const char *key, const char *value)
{
  char buffer[64];
  sprintf(buffer, "%s=%s", key, value ? value : "null");
  println(buffer);
  if (value) // ignore lines with an empty value
  {
    // Do the work for every known key
    if (strcmp(key, "hostname") == 0)
    {
      strcpy(hostname, value);
    }
    else if (strcmp(key, "wifi") == 0)
    {
      char ssid[32];
      char password[32];
      char *token = strtok((char *)value, ":");
      if (token != NULL)
      {
        strncpy(ssid, token, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
        token = strtok(NULL, ":");
        if (token != NULL)
        {
          strncpy(password, token, sizeof(password) - 1);
          password[sizeof(password) - 1] = '\0';
        }
        if (ssid[0] != '\0' && password[0] != '\0')
        {
          addWifiCredentials(ssid, password);
        }
      }
    }
    else if (strcmp(key, "range") == 0)
    {
      char low[32];
      char high[32];
      char *token = strtok((char *)value, ":");
      if (token != NULL)
      {
        strncpy(low, token, sizeof(low) - 1);
        low[sizeof(low) - 1] = '\0';
        token = strtok(NULL, ":");
        if (token != NULL)
        {
          strncpy(high, token, sizeof(high) - 1);
          high[sizeof(high) - 1] = '\0';
        }
        if (low[0] != '\0' && high[0] != '\0')
        {
          addRange(atof(low), atof(high));
        }
      }
    }
    else if (strcmp(key, "mqtt_broker") == 0)
    {
      strcpy(mqttBroker, value);
    }
    else if (strcmp(key, "mqtt_username") == 0)
    {
      strcpy(mqttUsername, value);
    }
    else if (strcmp(key, "mqtt_password") == 0)
    {
      strcpy(mqttPassword, value);
    }
    else if (strcmp(key, "mqtt_port") == 0)
    {
      mqttPort = atoi(value);
    }
    else if (strcmp(key, "detect_interval_ms") == 0)
    {
      detectInterval = strtoul(value, 0, 10);
    }
    else if (strcmp(key, "publish_interval_ms") == 0)
    {
      publishInterval = strtoul(value, 0, 10);
    }
    else if (strcmp(key, "sample_frequency_hz") == 0)
    {
      sampleFrequency = strtoul(value, 0, 10);
    }
    else if (strcmp(key, "loud_threshold") == 0)
    {
      loudThreshold = strtod(value, 0);
    }
    else if (strcmp(key, "minimum_ring_duration_ms") == 0)
    {
      minimumRingDuration = strtoul(value, 0, 10);
    }
    else if (strcmp(key, "minimum_ring_loudness") == 0)
    {
      minimumRingLoudness = strtod(value, 0);
    }
    else if (strcmp(key, "minimum_frequency_hz") == 0)
    {
      minimumRingFrequency = strtoul(value, 0, 10);
    }
    else if (strcmp(key, "percent_in_ranges") == 0)
    {
      minimumPercentInRanges = strtod(value, 0);
    }
    else
    {
      // That was a key we don't know
      sprintf(buffer, "UNKNOWN CONFIG: %s=%s", key, value);
      println(buffer);
    }
  }
}

// Read the config file from the SD card
void readConfig()
{
  KeyValueReader reader(CONFIG_FILE); // Create a new reader
  reader.process(keyValuePair);       // invoke the reader with keyValuePair as the callback
  reader.close();                     // Cleanup
}

// The general setup routine called on program startup
void setup()
{
  // Set up the pins
  pinMode(MAIN_BUTTON, INPUT_PULLUP);
  pinMode(MICROPHONE_PIN, INPUT_PULLUP);
#ifdef USE_SERIAL
  // Wait for Serial to be ready
  Serial.begin(115200); // Serial for debugging
  while (!Serial.availableForWrite())
  {
  };
  delay(1000); // Give the IDE a chance to catch up with Serial being ready
#endif
  println("GO!");
  setupOLED();   // OLED first - we want to display stuff
  setupSDCard(); // SD card seconds - we want to be able to read the config file
  readConfig();  // Read the config file. Needed for the subsequent setup routines
  setupAudio();
  setupWifi(); // Wifi before MQTT. MQTT depends on Wifi
  setupMQTT();
  // Setup some timestamps
  lastConnectionTest = millis();
  lastCheck = millis();
  lastPublish = millis();
  // We are ready
  statusText = (char *)"Ready.";
  displayStatus();
  println("Ready.");
}

// small utility function
// retuns the change from reference to value in percent
double relativeChangePercent(double reference, double value)
{
  double change = (value - reference) / reference;
  return change * 100;
}

bool ringing = false;
uint16_t lastTrigger = 0;
uint16_t numberOfLoudIntervals;
uint16_t numberOfInRangesIntervals;
double sumOfLoudness;
double sumOfPeaks;

// Check if the measured peak frequency falls within any of the ranges we consider interesting
bool isInRanges(double frequency)
{
  for (uint8_t index = 0; index < rangeCount; index += 1)
  {
    if (ranges[index]->low <= frequency && ranges[index]->high >= frequency)
    {
      return true;
    }
  }
  return false;
}

// Do the hard work
// Called several times per second to do all the detection work
void doFFT()
{
  char buffer[150];
  double inRange;
  // Read the samples from the ADC
  readSamples();
  // Calculate some measure of how loud the samples are
  computeLoudness();
  if (loudness > static_cast<uint16_t>(baseLine * loudThreshold))
  {
    // The current sample is quite loud. Is it the intercom though?
    // // Some setup for the FFT
    double vReal[SAMPLES];
    double vImag[SAMPLES];
    for (uint16_t i = 0; i < SAMPLES; i += 1)
    {
      vReal[i] = samples[i];
      vImag[i] = 0.0; // Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
    }
    // Create an FFT object for our sample
    FFT = arduinoFFT(vReal, vImag, SAMPLES, sampleFrequency);
    // Measure how long the FFT takes
    unsigned long fftStart = micros();
    // Do the actual FFT magic
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    double peak = FFT.MajorPeak();
    // Take another time measurement
    unsigned long fftEnd = micros();
    if (loudStart == 0)
    {
      // The last samples were not considered loud. These are.
      loudStart = millis();
      numberOfLoudIntervals = 1;
      numberOfInRangesIntervals = isInRanges(peak) ? 1 : 0;
      sumOfLoudness = loudness;
      sumOfPeaks = peak;
    }
    else
    {
      // Was loud and still is
      numberOfLoudIntervals += 1;
      numberOfInRangesIntervals += isInRanges(peak) ? 1 : 0;
      sumOfLoudness += loudness;
      sumOfPeaks += peak;
    }
    double percentInRange = static_cast<double>(numberOfInRangesIntervals) / static_cast<double>(numberOfLoudIntervals) * 100.0;
    inRange = percentInRange;
    publishAudioEvent(MQTT_TOPIC_AUDIO_EVENT, loudness, peak, percentInRange);
    lastPublish = millis();
    // Now we can do the actual detecting of the intercom
    // This whole process might need some refinement
    // The intercom is detected by:
    // Has it been loud for quite a while?
    if (!ringing && millis() - loudStart > minimumRingDuration)
    {
      // Is the signal actuall quite loud?
      if (sumOfLoudness / numberOfLoudIntervals > baseLine * minimumRingLoudness)
      {
        // Is the frequency quite high?
        if (sumOfPeaks / numberOfLoudIntervals > minimumRingFrequency)
        {
          // Finally check if enough samples where within the specified frequency ranges
          if (percentInRange > minimumPercentInRanges)
          {
            // It all checks out - assume it's the intercom
            if (millis() - lastTrigger > 5 * 1000) // not more than every 5 seconds
            {
              lastTrigger = millis();
              ringing = true;
              handleIntercomOn();
            }
          }
        }
      }
    }
  }
  else if (loudStart > 0 || millis() - lastPublish > publishInterval)
  {
    // The current samples are either not loud or it is time to publish a fresh set of values
    if (loudStart > 0)
    {
      // was loud - is no longer
      // Just logging for now. Could be used to trigger stuff
      unsigned long duration = millis() - loudStart;
      sprintf(buffer, "Was loud for %dms %.1f%%", duration, inRange);
      println(buffer);
      if (ringing)
      {
        ringing = false;
        handleIntercomOff();
      }
    }
    loudStart = 0; // No longer loud
    publishAudioEvent(MQTT_TOPIC_AUDIO_EVENT, 0, 0.0, 0.0);
    lastPublish = millis();
  }
}

// Main loop
void loop()
{
  // process any messages received from MQTT
  mqttClient.loop();
  // Every so often we check connectivity
  if (millis() - lastConnectionTest > CONNECTION_TEST_INTERVAL)
  {
    // check connections
    if (WiFi.status() != WL_CONNECTED)
    {
      // Wifi no longer connected - set up again
      setupWifi();
    }
    if (!mqttClient.connected())
    {
      // MQTT no longer connected - set up again
      setupMQTT();
    }
    lastConnectionTest = millis();
  }
  // Check button states for quick response times
  handleButton(MAIN_BUTTON, &mainButtonState, &onMainButtonPressed, &onMainButtonReleased);
  // Check if we need to run another sampling of the audio
  if (millis() - lastCheck > detectInterval)
  {
    lastCheck = millis();
    // Sampling required - do the heavy lifting
    doFFT();
  }
  delay(10); // A short delay? Might not be necessary?
}

// THE END
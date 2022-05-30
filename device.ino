#include <ESP32Time.h>
#include <WiFi.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <analogWrite.h>
#include <arduinoFFT.h>
#include "soc/sens_reg.h"

#define MuteButtonPin 19
#define WiFiStatusPin 18
#define CloudStatusPin 4
#define MicMutedStatusPin 5
#define MicPin 27
#define DeviceName "%DEVICE%"
#define BufferSize 50
#define LAT "%LATITUDE%"
#define LNG "%LONGITUDE%"
#define WiFiSSID "%SSID%"
#define WiFiPass "%PASS%"

uint32_t adc_register;
uint32_t wifi_register;

enum Connection
{
    Connected,
    Disconnected,
    Reconnecting
};

int bands[8] = {0, 0, 0, 0, 0, 0, 0, 0};
ESP32Time rtc;
Connection connection = Disconnected;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

bool micListening = true;

void setup()
{
    Serial.begin(115200);
    pinMode(MicPin, INPUT);
    pinMode(MuteButtonPin, INPUT_PULLDOWN);
    pinMode(WiFiStatusPin, OUTPUT);
    pinMode(MicMutedStatusPin, OUTPUT);
    pinMode(CloudStatusPin, OUTPUT);
    adc_register = READ_PERI_REG(SENS_SAR_READ_CTRL2_REG); // Wifi with ADC2 on ESP32 workaround.
    connectToWiFi();
    wifi_register = READ_PERI_REG(SENS_SAR_READ_CTRL2_REG); // Wifi with ADC2 on ESP32 workaround.
    Serial.println(mqttClient.setBufferSize(3000));
    mqttClient.setServer("mqtt.faurskov.dev", 1883);
}

long lastMsg = 0;
int sampleBuffer[BufferSize][4];
String sampleBufferDates[BufferSize];
int bufferIndex = 0;
bool sentPresence = false;
unsigned long currentMillis = millis();
unsigned long previousMillis = 0;
unsigned long interval = 10000;

void loop()
{
    currentMillis = millis();
    micStatusLoop();
    if (ensureConnected() == false)
        return;
    if (ensureMqttConnected() == false)
        return;

    mqttClient.loop();

    if (sentPresence == false)
    {
        String device = DeviceName;
        String presenceTopic = "devices/" + device + "/presence";
        char presenceTopicBuffer[presenceTopic.length() + 1];
        presenceTopic.toCharArray(presenceTopicBuffer, presenceTopic.length() + 1);

        String location = "{\"lat\":" + (String)LAT + ",\"lng\":" + (String)LNG + "}";
        char locationBuffer[location.length() + 1];
        location.toCharArray(locationBuffer, location.length() + 1);

        mqttClient.publish(presenceTopicBuffer, locationBuffer);
        Serial.println("sent presence message");
        sentPresence = true;
    }

    if (micListening == false)
        return;

    WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, adc_register);          // Wifi with ADC2 on ESP32 workaround.
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV); // Wifi with ADC2 on ESP32 workaround.
    bool successfulRecording = micLoop();
    WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, wifi_register); // Wifi with ADC2 on ESP32 workaround.

    if (successfulRecording == false)
        return;

    String payload = "{\"sent_at\":\"" + (String)rtc.getEpoch() + "\",\"data\":{\"bands\":[" + String(bands[0]) + "," + String(bands[1]) + "," + String(bands[2]) + "," + String(bands[3]) + "," + String(bands[4]) + "," + String(bands[5]) + "," + String(bands[6]) + "," + String(bands[7]) + "]}}";

    char buffer[payload.length() + 1];
    payload.toCharArray(buffer, payload.length() + 1);

    String topic = "devices/" + (String)DeviceName + "/sample";
    char topicBuffer[topic.length() + 1];
    topic.toCharArray(topicBuffer, topic.length() + 1);

    bool sent = mqttClient.publish(topicBuffer, buffer);
}

void connectToWiFi()
{
    WiFi.begin(WiFiSSID, WiFiPass);
    while (WiFi.status() != WL_CONNECTED)
    {
        blinkLed(WiFiStatusPin, 300, millis());
    }
    connection = Connected;
    analogWrite(WiFiStatusPin, 255);
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    while (loadTimeFromNTP() == false)
    {
        Serial.println("Failed to set time, retrying...");
    }
}

bool loadTimeFromNTP()
{
    Serial.println("Setting time");
    timeClient.begin();
    timeClient.forceUpdate();
    rtc.setTime(timeClient.getEpochTime());
    timeClient.end();
    delay(2000);
    if (rtc.getYear() < 2000)
        return false;
    Serial.print("Time set: ");
    Serial.println(rtc.getDateTime());
    return true;
}

bool ensureConnected()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        if (connection == Reconnecting)
        {
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
            connection = Connected;
        }

        analogWrite(WiFiStatusPin, 255);
        return true;
    }
    blinkLed(WiFiStatusPin, 300, currentMillis);
    analogWrite(CloudStatusPin, 0);
    if (currentMillis - previousMillis < interval)
        return false;
    connection = Reconnecting;
    Serial.println("WiFi down, attempting to reconnect!");
    WiFi.disconnect();
    WiFi.reconnect();

    previousMillis = currentMillis;

    return false;
}

unsigned long previousMillisMqtt = 0;
unsigned long intervalMqtt = 5000;
bool ensureMqttConnected()
{
    if (mqttClient.connected())
    {
        analogWrite(CloudStatusPin, 255);
        return true;
    }

    if (WiFi.status() != WL_CONNECTED)
        return false;

    blinkLed(CloudStatusPin, 500, currentMillis);
    if (currentMillis - previousMillisMqtt < intervalMqtt)
        return false;

    Serial.println("Connecting to MQTT Broker");
    mqttClient.connect("mqtt.faursakov.dev", "eas", "Oppose-Shopping7-Retake");
    previousMillisMqtt = currentMillis;

    return false;
}

const int BLOCK_SIZE = 256;
const uint8_t amplitude = 50;
int buff[BLOCK_SIZE];
int byteIndex = 0;
double vReal[BLOCK_SIZE];
double vImag[BLOCK_SIZE];
int32_t samples[BLOCK_SIZE];
arduinoFFT FFT = arduinoFFT();

bool micLoop()
{
    while (byteIndex < BLOCK_SIZE)
    {
        int sample = analogRead(MicPin);
        if (sample > 4000)
            return false;
        buff[byteIndex] = sample;
        byteIndex++;
    }

    byteIndex = 0;

    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
    {
        vReal[i] = buff[i];
        vImag[i] = 0.0; // Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
    }
    FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);
    for (int i = 0; i < 8; i++)
    {
        bands[i] = 0;
    }
    for (int i = 2; i < (BLOCK_SIZE / 2); i++)
    { // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
        if (i <= 2)
            bands[0] = max(bands[0], (int)(vReal[i])); // 125Hz
        if (i > 3 && i <= 5)
            bands[1] = max(bands[1], (int)(vReal[i])); // 250Hz
        if (i > 5 && i <= 7)
            bands[2] = max(bands[2], (int)(vReal[i])); // 500Hz
        if (i > 7 && i <= 15)
            bands[3] = max(bands[3], (int)(vReal[i])); // 1000Hz
        if (i > 15 && i <= 30)
            bands[4] = max(bands[4], (int)(vReal[i])); // 2000Hz
        if (i > 30 && i <= 53)
            bands[5] = max(bands[5], (int)(vReal[i])); // 4000Hz
        if (i > 53 && i <= 200)
            bands[6] = max(bands[6], (int)(vReal[i])); // 8000Hz
        if (i > 200)
            bands[7] = max(bands[7], (int)(vReal[i])); // 16000Hz
    }

    Serial.print("1:");
    Serial.print(bands[0]);
    Serial.print(",2:");
    Serial.print(bands[1]);
    Serial.print(",3:");
    Serial.print(bands[2]);
    Serial.print(",4:");
    Serial.print(bands[3]);
    Serial.print(",5:");
    Serial.print(bands[4]);
    Serial.print(",6:");
    Serial.print(bands[5]);
    Serial.print(",7:");
    Serial.print(bands[6]);
    Serial.print(",8:");
    Serial.println(bands[7]);

    return true;
}

enum MicToggleState
{
    TogglingOn,
    TogglingOff,
    Toggled,
    None
};

MicToggleState micToggleState = None;
long onTimer = 0;
void micButtonToggler(int pin)
{
    bool pressed = digitalRead(pin);

    if (!pressed && micToggleState != None)
    {
        micToggleState = None;
        return;
    }

    if (!pressed)
        return;

    if (micToggleState == None)
    {
        onTimer = currentMillis;
        micToggleState = micListening ? TogglingOff : TogglingOn;
    }

    if (currentMillis > (onTimer + 2000))
    {
        if (micToggleState == TogglingOff)
        {
            micListening = false;
        }
        else if (micToggleState == TogglingOn)
        {
            micListening = true;
        }
        micToggleState = Toggled;
    }
}

void micStatusLoop()
{
    micButtonToggler(MuteButtonPin);
    if (micToggleState == TogglingOn)
    {
        fadeLed(MicMutedStatusPin, 500, currentMillis);
        return;
    }
    if (micToggleState == TogglingOff)
    {
        fadeLed(MicMutedStatusPin, 1000, currentMillis);
        return;
    }
    else
    {
        analogWrite(MicMutedStatusPin, micListening ? 255 : 0);
        return;
    }
}

void blinkLed(int pin, int cycleTime, unsigned long currentTime)
{
    int cycle = currentTime % cycleTime; // 572
    bool turnOn = cycle < (cycleTime / 2);
    analogWrite(pin, turnOn ? 255 : 0);
}

void fadeLed(int pin, int cycleTime, unsigned long currentTime)
{
    float levelPerMilisecond = cycleTime / 2.0 / 256.0;
    int halfCycle = cycleTime / 2;
    int cycle = currentTime % cycleTime;
    bool increasing = cycle < halfCycle;
    int level = (increasing ? cycle : halfCycle - (cycle % halfCycle)) / levelPerMilisecond;
    analogWrite(pin, level);
}

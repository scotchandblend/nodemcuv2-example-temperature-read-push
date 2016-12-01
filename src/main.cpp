#include "Arduino.h"
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "DHT_U.h"
#include "ESP8266WiFi.h"
#include "WiFiUdp.h"
#include "config.h"

#define DHTPIN 5
#define DHTTYPE DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);
unsigned int readDelayMS;

char serverAddress[] = "afternoon-oasis-61290.herokuapp.com";
int port = 80;

void setupDht() {
  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);

  Serial.print("Sensor up: ");
  Serial.println(sensor.name);
  Serial.println();

  readDelayMS = sensor.min_delay / 1000;

  Serial.println("sensor.min_delay");
  Serial.println(sensor.min_delay);
}

void printVersions() {
  Serial.print("Core Version: "); Serial.println(ESP.getCoreVersion());
  Serial.print("SDK Version: "); Serial.println(ESP.getSdkVersion());
  Serial.print("Boot Version: "); Serial.println(ESP.getBootVersion());
}

WiFiClient client;

static const char ntpServerName[] = "us.pool.ntp.org";
const int timeZone = 0;

WiFiUDP Udp;
unsigned int localPort = 8888;

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

void setupTime() {
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
}

void setup() {
  Serial.begin(115200);

  printVersions();

  setupDht();

  Serial.println("connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(SSID, PASSWORD);
    delay(5000);
    Serial.print(".");
  }

  Serial.println("connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  setupTime();
}

void loop() {
  //Serial.print("start free heap: "); Serial.println(ESP.getFreeHeap(),DEC);
  //Serial.print("start free sketch space: "); Serial.println(ESP.getFreeSketchSpace(),DEC);

  delay(readDelayMS);

  sensors_event_t event;
  dht.temperature().getEvent(&event);

  if (isnan(event.temperature)) {
    Serial.println("error reading temperature");
    return;
  }

  String ts(now());
  double temp_in_f = ((event.temperature * 9) / 5) + 32;

  Serial.print("{");
  Serial.print(ts); Serial.print(",");
  Serial.print(temp_in_f); Serial.print(",");
  Serial.print(event.temperature);
  Serial.println("}");

  String f(temp_in_f, 2);
  String postData = String("f=" + f + "&ts=" + ts + "&secret=" + SECRET);
  //String postData("f=45.67&ts=289378935274&secret=secret");

  //Serial.print("end free heap: "); Serial.println(ESP.getFreeHeap(),DEC);
  //Serial.print("end free sketch space: "); Serial.println(ESP.getFreeSketchSpace(),DEC);

  client.stop();
  if (client.connect(serverAddress, port)) {
    client.println("POST /temperature HTTP/1.1");

    client.print("Host: ");
    client.println(serverAddress);

    client.println("User-Agent: ArduinoWiFi");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Connection: close");

    char lengthHeader[40];
    sprintf(lengthHeader, "Content-Length: %d", strlen(postData.c_str()));
    client.println(lengthHeader);

    client.println();
    client.println(postData);
    client.println();

    String line = client.readStringUntil('\r');
    Serial.println(line);
  }
}

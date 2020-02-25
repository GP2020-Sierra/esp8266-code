#include <Arduino.h>

extern "C"
{
#include <user_interface.h>
}

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define CHANNEL_HOP_INTERVAL_MS 1000

#define DHTTYPE    DHT11     // DHT 11
#define DHTPIN 13
DHT_Unified dht(DHTPIN, DHTTYPE);


#define DATA_LENGTH 112

#define TYPE_MANAGEMENT 0x00
#define TYPE_CONTROL 0x01
#define TYPE_DATA 0x02
#define SUBTYPE_PROBE_REQUEST 0x04
#define SUBTYPE_BEACON 0b1000

struct RxControl
{
  signed rssi : 8; // signal intensity of packet
  unsigned rate : 4;
  unsigned is_group : 1;
  unsigned : 1;
  unsigned sig_mode : 2;       // 0:is 11n packet; 1:is not 11n packet;
  unsigned legacy_length : 12; // if not 11n packet, shows length of packet.
  unsigned damatch0 : 1;
  unsigned damatch1 : 1;
  unsigned bssidmatch0 : 1;
  unsigned bssidmatch1 : 1;
  unsigned MCS : 7;        // if is 11n packet, shows the modulation and code used (range from 0 to 76)
  unsigned CWB : 1;        // if is 11n packet, shows if is HT40 packet or not
  unsigned HT_length : 16; // if is 11n packet, shows length of packet.
  unsigned Smoothing : 1;
  unsigned Not_Sounding : 1;
  unsigned : 1;
  unsigned Aggregation : 1;
  unsigned STBC : 2;
  unsigned FEC_CODING : 1; // if is 11n packet, shows if is LDPC packet or not.
  unsigned SGI : 1;
  unsigned rxend_state : 8;
  unsigned ampdu_cnt : 8;
  unsigned channel : 4; //which channel this packet in.
  unsigned : 12;
};

struct SnifferPacket
{
  struct RxControl rx_ctrl;
  uint8_t data[DATA_LENGTH];
  uint16_t cnt;
  uint16_t len;
};

struct KnownMac
{
  uint8_t mac[6];
  int8_t rssi;
  uint8_t chan;
  uint8_t flags;
  struct KnownMac *next;
};

static struct KnownMac* MAC_LIST;

// Declare each custom function (excluding built-in, such as setup and loop) before it will be called.
// https://docs.platformio.org/en/latest/faq.html#convert-arduino-file-to-c-manually
static void showMetadata(SnifferPacket *snifferPacket);
static void ICACHE_FLASH_ATTR sniffer_callback(uint8_t *buffer, uint16_t length);
static void printDataSpan(uint16_t start, uint16_t size, uint8_t *data);
static int hashMAC(uint8_t *data, uint16_t offset);
static void getMAC(char *addr, uint8_t *data, uint16_t offset);
void channelHop();

static void showMetadata(SnifferPacket *snifferPacket)
{

  unsigned int frameControl = ((unsigned int)snifferPacket->data[1] << 8) + snifferPacket->data[0];

  //uint8_t version      = (frameControl & 0b0000000000000011) >> 0;
  uint8_t frameType = (frameControl & 0b0000000000001100) >> 2;
  uint8_t frameSubType = (frameControl & 0b0000000011110000) >> 4;
  //uint8_t toDS         = (frameControl & 0b0000000100000000) >> 8;
  //uint8_t fromDS       = (frameControl & 0b0000001000000000) >> 9;

  if (frameType == TYPE_CONTROL) {
    return; // Don't proc control frames; some miss src address (i.e. ack)
  }

  uint8_t *mac = snifferPacket->data + 10;
  struct KnownMac *macEntry = MAC_LIST;
  bool found = 0;
  if (macEntry != nullptr)
  {
    while (macEntry->next != nullptr)
    {
      if (!memcmp(macEntry->mac, mac, 6))
      {
        found = 1;
        break;
      }
      macEntry = macEntry->next;
    }
    if (!found)
    {
      struct KnownMac *next = new KnownMac();
      memcpy(next->mac, mac, 6);
      next->next = nullptr;
      macEntry->next = next;
      macEntry = next;
    }
  }
  else
  {
    macEntry = MAC_LIST = new KnownMac();
    memcpy(MAC_LIST->mac, mac, 6);
    MAC_LIST->next = nullptr;
  }

  macEntry->chan = wifi_get_channel();
  macEntry->rssi = snifferPacket->rx_ctrl.rssi;

  if (frameType == TYPE_MANAGEMENT ||
      frameSubType == SUBTYPE_BEACON)
  {
    macEntry->flags |= 1;
  }
  
  if (found) { return; }  
}

/**
 * Callback for promiscuous mode
 */
static void ICACHE_FLASH_ATTR sniffer_callback(uint8_t *buffer, uint16_t length)
{
  //wifi_promiscuous_enable(0); // Prevent multiple packets building up
  struct SnifferPacket *snifferPacket = (struct SnifferPacket *)buffer;
  showMetadata(snifferPacket);
  //wifi_promiscuous_enable(1); // Prevent multiple packets building up
}

static void printDataSpan(uint16_t start, uint16_t size, uint8_t *data)
{
  for (uint16_t i = start; i < DATA_LENGTH && i < start + size; i++)
  {
    Serial.write(data[i]);
  }
}

static void getMAC(char *addr, uint8_t *data, uint16_t offset)
{
  sprintf(addr, "%02x:%02x:%02x:%02x:%02x:%02x", data[offset + 0], data[offset + 1], data[offset + 2], data[offset + 3], data[offset + 4], data[offset + 5]);
}

static os_timer_t channelHop_timer;

/**
 * Callback for channel hoping
 */
void channelHop()
{
  // hoping channels 1-13
  uint8 new_channel = wifi_get_channel() + 1;
  if (new_channel > 13)
  {
    // Stop recieving
    wifi_promiscuous_enable(0);
    new_channel = 1;
    struct KnownMac *HEAD = MAC_LIST;
    uint32_t basestation = 0, devices = 0;
    // Free chain
    while (HEAD != nullptr)
    {
      /*
      Serial.print("RSSI: ");
      Serial.print(HEAD->rssi, DEC);

      Serial.print(" Ch: ");
      Serial.print(HEAD->chan);

      char addr[] = "00:00:00:00:00:00";
      getMAC(addr, HEAD->mac, 0);
      Serial.print(" Peer MAC: ");
      Serial.print(addr);

      if (HEAD->flags & 1) {
        Serial.print(" Base Station ");
      }

      Serial.println();
      */

     if (HEAD->flags & 1) {
        basestation++;
      } else {
        devices++;
      }

      struct KnownMac *next = HEAD->next;
      free(HEAD);
      HEAD = next;
    }
    devices = min(devices-1, devices); // Min avoids rollover
    devices /= 2;
    basestation /= 2;
    Serial.printf("Found %d devices and %d basestations\n", devices, basestation);
    Serial.printf("{'sensor': 'esp8266', 'devs': %d, 'bss': %d}\n", devices, basestation);
    MAC_LIST = new KnownMac();
    uint64_t emptymac = 0;
    memcpy(MAC_LIST->mac, &emptymac, 6);
    MAC_LIST->next = nullptr;
    //Start recieving
    wifi_promiscuous_enable(1);
  }
  // Suspend runtime
  wifi_set_channel(new_channel);
}

void setup()
{
  // set the WiFi chip to "promiscuous" mode aka monitor mode
  MAC_LIST = new KnownMac();
  uint64_t emptymac = 0;
  memcpy(MAC_LIST->mac, &emptymac, 6);
  MAC_LIST->next = nullptr;
  Serial.begin(115200);
  delay(10);
  Serial.write("Test\n");
  //Setup DHT11
  dht.begin();
    Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  //Setup wifi
  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(1);
  wifi_promiscuous_enable(0);
  delay(10);
  wifi_set_promiscuous_rx_cb(sniffer_callback);
  delay(10);
  wifi_promiscuous_enable(1);
  // setup the channel hoping callback timer
  os_timer_disarm(&channelHop_timer);
  os_timer_setfn(&channelHop_timer, (os_timer_func_t *)channelHop, NULL);
  os_timer_arm(&channelHop_timer, CHANNEL_HOP_INTERVAL_MS, 1);
}

void loop()
{
  delay(CHANNEL_HOP_INTERVAL_MS*3);
  wifi_promiscuous_enable(0);
  //Dump DHT11 data:
  sensors_event_t temp_event;
  dht.temperature().getEvent(&temp_event);
  sensors_event_t humid_event;
  dht.humidity().getEvent(&humid_event);
  
  if (
    isnan(temp_event.temperature) || 
    isnan(humid_event.relative_humidity)) {
    Serial.printf("DHT11 not found!!!\n");
  } else {
  
    Serial.printf("{'sensor': 'dht11', 'humid': %f, 'temp': %f}\n", humid_event.relative_humidity, temp_event.temperature);
  }
  wifi_promiscuous_enable(1);
}

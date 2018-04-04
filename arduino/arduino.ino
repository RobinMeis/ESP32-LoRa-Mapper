#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "abp_keys.h"
#include "gps_config.h"
#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
#include <HardwareSerial.h>

HardwareSerial gpsSerial(1); //We can use Hardware Serial for our GPS module on ESP32


static const PROGMEM u1_t NWKSKEY[16] = NETWORK_SESSION_KEY; //Set these options in abp_keys.h
static const u1_t PROGMEM APPSKEY[16] = APP_SESSION_KEY;
static const u4_t DEVADDR = DEVICE_ADDRESS;

void os_getArtEui (u1_t* buf) { } //Empty callbacks for ABP
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

const unsigned TX_INTERVAL = 5; // Schedule TX every this many seconds (might become longer due to duty cycle limitations).

const lmic_pinmap lmic_pins = { // Pin mapping for TTGO with 3D antenna
  .mosi = 27,
  .miso = 19,
  .sck = 5,
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 32, 33},
};

TinyGPSPlus gps; //GPS

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200); //Debug Port
    Serial.println(F("Starting"));

    gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX, GPS_TX); //GPS Port, configure in gps_config.h

    os_init(); // LMIC init
    LMIC_reset();

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9; // TTN uses SF9 for its RX2 window.
    LMIC_setDrTxpow(DR_SF7,14); // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)

    do_send(&sendjob); // Start job
}

void loop() {
    os_runloop_once();

    while (gpsSerial.available() > 0)
        if (gps.encode(gpsSerial.read()))
            displayGPS();
}

void displayGPS() { //Shows information about GPS status on serial console and OLED
    if (gps.location.isValid()) {
        Serial.print(F("GPS: lat: "));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(", lng: "));
        Serial.print(gps.location.lng(), 6);
        Serial.print(F(", alt: "));
        Serial.print(gps.altitude.meters(), 6);
        Serial.print(F(", HDOP: "));
        Serial.print(gps.hdop.hdop(), 6);
    } else
        Serial.print(F("INVALID GPS DATA"));
    Serial.println();
}


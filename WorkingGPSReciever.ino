#include <HardwareSerial.h>
#include "LoRaWan_APP.h"

// ── GPS ──────────────────────────────────────────────────────────────────────
HardwareSerial GPSserial(1);
#define GPS_RX 45
#define GPS_TX 46

unsigned long lastDataTime = 0;
const unsigned long timeout = 3000;

// ── LoRa config (match these on your receiver) ────────────────────────────────
#define RF_FREQUENCY          915000000  // Hz — change to 868E6 if you're in EU
#define TX_OUTPUT_POWER       14         // dBm
#define LORA_BANDWIDTH        0          // 0=125kHz, 1=250kHz, 2=500kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       1          // 1=4/5, 2=4/6, 3=4/7, 4=4/8
#define LORA_PREAMBLE_LENGTH  8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON  false
#define BUFFER_SIZE           64

char txPacket[BUFFER_SIZE];
bool loraIdle = true;

// ── LoRa event callbacks ──────────────────────────────────────────────────────
void OnTxDone(void) {
  Serial.println("LoRa TX done");
  loraIdle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("LoRa TX timeout");
  loraIdle = true;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("Starting GPS + LoRa...");

  // Heltec board init (handles OLED, LoRa power pin, etc.)
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(
    MODEM_LORA,
    TX_OUTPUT_POWER,
    0,                        // FSK frequency deviation (unused)
    LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR,
    LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH,
    LORA_FIX_LENGTH_PAYLOAD_ON,
    true,                     // CRC on
    0, 0,                     // freq hopping off
    LORA_IQ_INVERSION_ON,
    3000                      // TX timeout ms
  );
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  bool gotData = false;

  while (GPSserial.available()) {
    String line = GPSserial.readStringUntil('\n');
    line.trim();
    lastDataTime = millis();
    gotData = true;

    if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
      parseGPGGA(line);
    }
  }

  if (!gotData && millis() - lastDataTime > timeout) {
    Serial.println("No GPS connection");
    lastDataTime = millis();
  }

  Radio.IrqProcess(); // must be called regularly for Heltec LoRa
}

// ── Transmit a lat/lon over LoRa ──────────────────────────────────────────────
void transmitGPS(double lat, double lon) {
  if (!loraIdle) return; // previous TX still in progress

  snprintf(txPacket, BUFFER_SIZE, "%.6f,%.6f", lat, lon);
  Serial.printf("Transmitting: %s\n", txPacket);

  loraIdle = false;
  Radio.Send((uint8_t *)txPacket, strlen(txPacket));
}

// ── Parse $GPGGA / $GNGGA ─────────────────────────────────────────────────────
void parseGPGGA(String gga) {
  String fields[15];
  int index = 0, start = 0;

  for (int i = 0; i <= (int)gga.length(); i++) {
    if (i == (int)gga.length() || gga[i] == ',') {
      fields[index++] = gga.substring(start, i);
      start = i + 1;
      if (index >= 15) break;
    }
  }

  if (fields[6].toInt() == 0) {
    Serial.println("No fix");
    return;
  }

  double lat = convertToDecimal(fields[2], fields[3]);
  double lon = convertToDecimal(fields[4], fields[5]);

  Serial.print("Latitude: ");  Serial.print(lat, 6);
  Serial.print("  Longitude: "); Serial.println(lon, 6);

  transmitGPS(lat, lon);
}

// ── NMEA → decimal degrees ────────────────────────────────────────────────────
double convertToDecimal(String raw, String dir) {
  if (raw.length() < 6) return 0.0;

  int dotIndex = raw.indexOf('.');
  if (dotIndex < 2) return 0.0;

  double deg = raw.substring(0, dotIndex - 2).toDouble();
  double min = raw.substring(dotIndex - 2).toDouble();
  double dec = deg + (min / 60.0);

  if (dir == "S" || dir == "W") dec = -dec;
  return dec;
}

  int fix = fields[6].toInt(); // fix quality
  if (fix == 0) {
    Serial.println("No fix");
    return;
  }

  String latRaw = fields[2];
  String latDir = fields[3];
  String lonRaw = fields[4];
  String lonDir = fields[5];

  double lat = convertToDecimal(latRaw, latDir);
  double lon = convertToDecimal(lonRaw, lonDir);

  Serial.print("Latitude: "); Serial.print(lat, 6);
  Serial.print("  Longitude: "); Serial.println(lon, 6);
}

// Convert NMEA raw coordinates to decimal degrees
double convertToDecimal(String raw, String dir) {
  if (raw.length() < 6) return 0.0;

  int dotIndex = raw.indexOf('.');
  if (dotIndex < 2) return 0.0;

  // Minutes always occupy last 2 digits before the decimal point
  double deg = raw.substring(0, dotIndex - 2).toDouble();
  double min = raw.substring(dotIndex - 2).toDouble();
  double dec = deg + (min / 60.0);

  if (dir == "S" || dir == "W") dec = -dec;
  return dec;
}
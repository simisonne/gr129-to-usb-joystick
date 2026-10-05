/*
 * GR129 spectrum monitor for nRF52840 (nrf_to_nrf)
 *
 * Sweeps the full 2.4GHz ISM band (nRF24 channels 0..100 = 2400..2500MHz),
 * records the strongest sample (lowest RSSISAMPLE = strongest, values are
 * -dBm) per channel over a 2s window, then reports the 15 hottest channels.
 *  - M,ch:min/hits,... pkts=N   every 2s
 *  - P,ch,hex16                 any hardware-CRC packet on GR129 address
 *
 * A point-blank GR129 transmitter shows up as a handful of channels with
 * min << noise floor (noise floor is ~95..101).
 */

#include <nrf_to_nrf.h>

nrf_to_nrf radio;

static const uint8_t PKT_LEN = 16;
static const uint8_t GR129_ADDR[5] = {0x6D, 0x6A, 0x73, 0x73, 0x73};

uint8_t minRssi[101];
uint16_t hitCount[101];      // dwell visits with sample < -85dBm
uint8_t ch = 0;
uint32_t tReport = 0;
uint16_t pktCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("S,boot,GR129 spectrum v1"));

  if (!radio.begin()) {
    Serial.println(F("S,err,radio begin failed"));
    while (1) delay(1000);
  }
  radio.setAutoAck(false);
  radio.setPayloadSize(PKT_LEN);
  radio.setCRCLength(NRF_CRC_16);
  radio.setAddressWidth(5);
  radio.setDataRate(NRF_1MBPS);
  radio.openReadingPipe(1, GR129_ADDR);
  radio.flush_rx();
  radio.startListening();
  radio.setChannel(0);

  memset(minRssi, 0xFF, sizeof(minRssi));
  memset(hitCount, 0, sizeof(hitCount));
  tReport = millis();
  Serial.println(F("S,sweep,0..100"));
}

void loop() {
  // dwell on the current channel: 20 instant RSSI samples (~1.5ms)
  uint8_t m = 255;
  for (uint8_t i = 0; i < 20; i++) {
    uint8_t v = radio.getRSSI();
    if (v < m) m = v;
  }
  if (m < minRssi[ch]) minRssi[ch] = m;
  if (m < 85) hitCount[ch]++;

  while (radio.available()) {
    uint8_t p[PKT_LEN];
    radio.read(p, PKT_LEN);
    pktCount++;
    Serial.print(F("P,"));
    Serial.print(ch);
    Serial.print(',');
    for (uint8_t i = 0; i < PKT_LEN; i++) {
      if (p[i] < 0x10) Serial.print('0');
      Serial.print(p[i], HEX);
    }
    Serial.println();
  }

  ch++;
  if (ch > 100) ch = 0;
  radio.setChannel(ch);
  delay(4);

  if (millis() - tReport >= 2000) {
    tReport = millis();

    // insertion-sort channel indices by minRssi (strongest first)
    uint8_t order[101];
    for (uint16_t i = 0; i < 101; i++) order[i] = i;
    for (uint16_t i = 1; i < 101; i++) {
      uint8_t k = order[i];
      int16_t j = i - 1;
      while (j >= 0 && minRssi[order[j]] > minRssi[k]) { order[j + 1] = order[j]; j--; }
      order[j + 1] = k;
    }

    Serial.print(F("M,"));
    for (uint8_t i = 0; i < 15; i++) {
      uint8_t c = order[i];
      Serial.print(c);
      Serial.print(':');
      Serial.print(minRssi[c]);
      if (hitCount[c]) { Serial.print('/'); Serial.print(hitCount[c]); }
      Serial.print(i == 14 ? '\n' : ',');
      delay(2);   // pace the line so the CDC TX ring never overflows
    }
    Serial.print(F("P,"));
    Serial.println(pktCount);
    Serial.flush();   // do not let the CDC TX ring drop the report

    memset(minRssi, 0xFF, sizeof(minRssi));
    memset(hitCount, 0, sizeof(hitCount));
  }
}

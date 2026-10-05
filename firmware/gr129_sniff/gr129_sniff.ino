/*
 * GR129 (MJX / V202-family) 2.4GHz sniffer for nRF52840
 *
 * Protocol facts (reverse-engineered from public sources, see notes below):
 *  - RF chip family: Beken BK2421 = nRF24L01-compatible ESB
 *  - Air format: 1Mbps (fallback 250kbps), 5-byte address, static 16-byte payload,
 *    hardware CRC16 (poly 0x11021, init 0xFFFF)  [nrf_to_nrf defaults match this]
 *  - TX address (sniffed from GR129 SPI: cmd 0x30 TX_ADDR):
 *      6D 6A 73 73 73   (= 'm','j','s' + two more; P1 sniffed as 6A 6D 37 37 37,
 *       consistent with the V202 P1 derivation rule -> confirms P0/TX addr)
 *  - 16-byte frame layout (verified: sum(frame[0..14]) & 0xFF == frame[15]):
 *      [0] throttle     [1] rudder/yaw   [2] elevator/pitch  [3] aileron/roll
 *      [4..6] trims (0x40 center)         [7..9] TX ID (3 bytes)
 *      [10..13] flags/zero                [14] flags (0xC0 = bind)
 *      [15] checksum
 *  - Channel hopping: 4 base rows of 16 channels; row = sum(txid)&3,
 *    increment = (sum(txid)&0x1E)>>2; channel = row[i]+inc, minus 3 if div by 16.
 *    TX dwells 8ms per channel (2 packets x 4ms), full cycle 128ms.
 *  - Bind frames: same table or on 8 fixed "test" channels (uncertain -> we scan wide)
 *
 * This firmware only SNIFFS and prints frames over USB CDC serial:
 *   S,<state>,<msg>        status
 *   T,<txid hex>,<16 hop channels>   hop table once TX ID seen
 *   V,<ms>,<ch>,<hex16>,<thr>,<yaw>,<pit>,<ail>,<trimY>,<trimP>,<trimR>,<flags>
 *   R,<ms>,<ch>,<hex16>    raw frame failing app-checksum (rate limited)
 */

#include <nrf_to_nrf.h>

nrf_to_nrf radio;

static const uint8_t PKT_LEN = 16;
// Sniffed GR129 TX_ADDR (register order)
static const uint8_t GR129_ADDR[5] = {0x6D, 0x6A, 0x73, 0x73, 0x73};

// V202/MJX frequency hopping base rows (deviation + execuc v202-receiver)
static const uint8_t FH[4][16] = {
  {0x27,0x1B,0x39,0x28,0x24,0x22,0x2E,0x36,0x19,0x21,0x29,0x14,0x1E,0x12,0x2D,0x18},
  {0x2E,0x33,0x25,0x38,0x19,0x12,0x18,0x16,0x2A,0x1C,0x1F,0x37,0x2F,0x23,0x34,0x10},
  {0x11,0x1A,0x35,0x24,0x28,0x18,0x25,0x2A,0x32,0x2C,0x14,0x27,0x36,0x34,0x1C,0x17},
  {0x22,0x27,0x17,0x39,0x34,0x28,0x2B,0x1D,0x18,0x2A,0x21,0x38,0x10,0x26,0x20,0x1F}
};
// 8 bind "test" channels: 0x18 + (i<<3), minus 3 if divisible by 16
static const uint8_t FTEST[8] = {0x18,0x1D,0x28,0x2D,0x38,0x3D,0x48,0x4D};

enum Stage { SCAN_ALL, SCAN_KNOWN, LOCKED };
Stage stage = SCAN_ALL;

uint8_t txid[3];
bool haveTxid = false;
uint8_t hop[16];
uint8_t hopIdx = 0;
uint8_t scanCh = 8;
uint32_t tScanCh = 0;
uint32_t tLastValid = 0;   // last app-valid frame (any stage)
uint32_t tLastAny = 0;     // last HW-CRC-valid packet
uint32_t tLastRaw = 0;
uint32_t tRateFlip = 0;
uint8_t errCode = 0;
uint8_t rateIdx = 0;   // 0=1M 1=250k 2=2M
uint16_t rpdHit[101];
uint16_t rpdTotal = 0;
uint32_t tDPrint = 0;
uint8_t lastRmin = 255, lastRmax = 0;
uint32_t tSelfStart = 0;
bool selfDone = false;
uint16_t selfRxTot = 0, selfTxTot = 0;
uint32_t tAddrFlip = 0;
bool addrRev = false;
// second candidate pipe: V202 P1 derivation (sniffed off GR129 SPI per btoschi)
static const uint8_t ADDR_P1[5] = {0x6A, 0x6D, 0x37, 0x37, 0x37};
uint32_t pipeCnt[2] = {0, 0};     // frames per pipe since last Q line
uint32_t tRateCycle = 0;
uint32_t tLinger = 0;   // set on each catch; SCAN_ALL holds the channel 10ms
// channels where we actually caught a frame: revisit them at double rate
uint8_t hot[16];
uint8_t hotN = 0;
static void noteHot(uint8_t c) {
  for (uint8_t i = 0; i < hotN; i++) if (hot[i] == c) return;
  if (hotN < 16) { hot[hotN++] = c; return; }
  memmove(hot, hot + 1, 15);          // keep a rolling window
  hot[15] = c;
}
static const uint8_t ADDR_REV[5] = {0x73, 0x73, 0x73, 0x6A, 0x6D};
uint32_t pktAny = 0, pktValid = 0;

static void setRate(uint8_t idx) {
  rateIdx = idx;
  switch (idx) {
    case 0:  radio.setDataRate(NRF_1MBPS);   break;
    case 1:  radio.setDataRate(NRF_250KBPS); break;
    default: radio.setDataRate(NRF_2MBPS);   break;
  }
  Serial.print(F("S,rate,"));
  Serial.println(idx == 0 ? F("1M") : (idx == 1 ? F("250k") : F("2M")));
}

static void computeHop() {
  uint8_t s = txid[0] + txid[1] + txid[2];
  const uint8_t *row = FH[s & 0x03];
  uint8_t inc = (s & 0x1E) >> 2;
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t v = row[i] + inc;
    hop[i] = (v & 0x0F) ? v : v - 3;   // avoid channels divisible by 16
  }
  Serial.print(F("T,"));
  Serial.print(txid[0], HEX); Serial.print(txid[1], HEX); Serial.println(txid[2], HEX);
  Serial.print(F("H"));
  for (uint8_t i = 0; i < 16; i++) { Serial.print(','); Serial.print(hop[i]); }
  Serial.println();
}

static bool checkCsum(const uint8_t *f) {
  uint16_t s = 0;
  for (uint8_t i = 0; i < 15; i++) s += f[i];
  return (s & 0xFF) == f[15];
}

static void printHex16(const uint8_t *f) {
  for (uint8_t i = 0; i < PKT_LEN; i++) {
    if (f[i] < 0x10) Serial.print('0');
    Serial.print(f[i], HEX);
  }
}

// execuc/v202 decode: sign-magnitude-ish with center at 0x00/0x80
static int16_t decAxis(uint8_t b) {
  return (b < 0x80) ? -(int16_t)b : ((int16_t)b - 0x80);
}

static uint8_t curPipe = 0;
static void printValid(const uint8_t *f, uint8_t ch) {
  Serial.print(F("V,"));
  Serial.print(millis()); Serial.print(',');
  Serial.print(ch); Serial.print(',');
  Serial.print(curPipe); Serial.print(',');
  printHex16(f); Serial.print(',');
  Serial.print(f[0]); Serial.print(',');
  Serial.print(decAxis(f[1])); Serial.print(',');
  Serial.print(decAxis(f[2])); Serial.print(',');
  Serial.print(decAxis(f[3])); Serial.print(',');
  Serial.print((int8_t)(f[4] - 0x40)); Serial.print(',');
  Serial.print((int8_t)(f[5] - 0x40)); Serial.print(',');
  Serial.print((int8_t)(f[6] - 0x40)); Serial.print(',');
  Serial.println(f[14]);
}

static void printRaw(const uint8_t *f, uint8_t ch) {
  if (millis() - tLastRaw < 10) return;
  tLastRaw = millis();
  Serial.print(F("R,"));
  Serial.print(millis()); Serial.print(',');
  Serial.print(ch); Serial.print(',');
  Serial.print(curPipe); Serial.print(',');
  printHex16(f); Serial.println();
}

static void enterLocked(uint8_t ch) {
  int8_t hit = -1;
  for (uint8_t i = 0; i < 16; i++) if (hop[i] == ch) { hit = i; break; }
  if (hit < 0) return;            // channel outside the table: keep hunting
  stage = LOCKED;
  hopIdx = hit;
  tLastValid = millis();
  errCode = 0;
  Serial.print(F("S,locked,at "));
  Serial.println(ch);
}

static void handlePacket(const uint8_t *f, uint8_t ch) {
  pktAny++;
  tLastAny = millis();
  noteHot(ch);
  bool ok = checkCsum(f);
  bool bind = ok && (f[14] == 0xC0);

  if (ok) {
    pktValid++;
    if (!haveTxid) {
      txid[0] = f[7]; txid[1] = f[8]; txid[2] = f[9];
      haveTxid = true;
      computeHop();
      stage = SCAN_KNOWN;
      Serial.println(F("S,scan_known"));
    }
    tLastValid = millis();
    // data frame (not bind): we can follow the hop stream
    if (!bind) {
      if (stage == LOCKED) {
        hopIdx = (hopIdx + 1) & 0x0F;
        radio.setChannel(hop[hopIdx]);
        errCode = 0;
      } else if (stage == SCAN_KNOWN) {
        enterLocked(ch);
      }
      printValid(f, ch);
    } else {
      // bind frame: log it, keep hunting data frames
      printValid(f, ch);
      Serial.println(F("S,bind_frame"));
    }
  } else {
    printRaw(f, ch);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("S,boot,GR129 sniffer v1"));

  if (!radio.begin()) {
    Serial.println(F("S,err,radio begin failed"));
    while (1) delay(1000);
  }
  radio.setAutoAck(false);          // GR129 has no auto-ack
  radio.setPayloadSize(PKT_LEN);    // static 16B -> PCNF0 = no S0/LF/S1 (nRF24 format)
  radio.setCRCLength(NRF_CRC_16);   // nRF24-compatible CRC16
  radio.setAddressWidth(5);
  setRate(0);                       // 1M first, cycles 1M/250k/2M if silent
  radio.openReadingPipe(0, GR129_ADDR);
  radio.openReadingPipe(1, ADDR_P1);
  radio.flush_rx();
  radio.startListening();

  scanCh = 8;
  radio.setChannel(scanCh);
  tScanCh = micros();
  tRateFlip = millis();
  tRateCycle = millis();
  // TXID captured live from this controller (V line at boot):
  txid[0] = 0xF8; txid[1] = 0x7D; txid[2] = 0x01;
  haveTxid = true;
  computeHop();
  stage = SCAN_ALL;          // TX uses the whole band (99-ch histogram), full sweep wins
  Serial.println(F("S,scan,all-fast"));
  tSelfStart = millis();
}

void loop() {
  // ---- 8s antenna/RF loopback self test (before anything else) ----------
  // TX 4 packets on ch64 at the GR129 address, listen back immediately.
  // If these are received, our radio + antenna path works end to end.
  if (false) {
    static uint32_t tSelfLast = 0;
    if (millis() - tSelfLast > 60) {
      tSelfLast = millis();
      uint8_t p[16];
      memset(p, 0xA5, sizeof(p));
      radio.stopListening();
      radio.setChannel(64);
      radio.openWritingPipe(GR129_ADDR);
      uint8_t txok = 0;
      for (uint8_t i = 0; i < 4; i++) if (radio.write(p, 16)) txok++;
      radio.startListening();
      radio.setChannel(64);
      uint32_t t0 = millis();
      uint8_t got = 0;
      while (millis() - t0 < 6) {
        if (radio.available()) { radio.read(p, 16); got++; }
      }
      selfRxTot += got;
      selfTxTot += txok;
      Serial.print(F("L,tx="));
      Serial.print(selfTxTot);
      Serial.print(F(",rx="));
      Serial.println(selfRxTot);
      if (millis() - tSelfStart > 8000) {
        selfDone = true;
        radio.stopListening();
        radio.openReadingPipe(1, GR129_ADDR);
        radio.setChannel(scanCh);
        radio.startListening();
        Serial.print(F("S,selftest done tx="));
        Serial.print(selfTxTot);
        Serial.print(F(",rx="));
        Serial.println(selfRxTot);
      }
    }
    return;
  }
  // -----------------------------------------------------------------------

  uint32_t now = millis();
  uint32_t nowUs = micros();

  // ---- receive ----
  uint8_t pipe;
  while (radio.available(&pipe)) {
    uint8_t buf[PKT_LEN];
    radio.read(buf, PKT_LEN);
    uint8_t ch = radio.getChannel() & 0x7F;
    if (pipe < 2) pipeCnt[pipe]++;
    curPipe = (pipe < 2) ? pipe : 9;
    handlePacket(buf, ch);
    curPipe = 0;
    tLinger = micros();   // hold this channel: burst pairs arrive <10ms apart
  }

  // ---- 10s heartbeat: frame counts (rate stays fixed at 1M) ----
  if (now - tRateCycle > 10000) {
    tRateCycle = now;
    Serial.print(F("Q,1M,A=")); Serial.print(pipeCnt[0]);
    Serial.print(F(",B=")); Serial.println(pipeCnt[1]);
    pipeCnt[0] = pipeCnt[1] = 0;
  }

  // ---- channel scheduling ----
  if (stage == SCAN_ALL) {
    // linger on the current channel for 10ms after a catch: bursts arrive
    // in pairs closer than 10ms (measured), so staying put catches both
    if (nowUs - tLinger < 10000) {
      // hold channel
    } else if (nowUs - tScanCh > 2000) {   // fast 2ms dwell, plain full sweep
      scanCh++;
      if (scanCh > 100) scanCh = 2;
      radio.setChannel(scanCh);
      tScanCh = nowUs;
    }
    // 1M + forward address are proven correct (frames captured), no flipping
  }
  else if (stage == SCAN_KNOWN) {
    // hybrid: park 300ms on the proven energy hotspot, then sweep the
    // V202 hop table + bind test channels (5ms each)
    static bool parked = false;
    static uint8_t k = 0;
    uint32_t ph = now % 420;
    if (ph < 300) {
      if (!parked) {
        radio.setChannel(hotN ? hot[0] : 40);
        parked = true;
      }
    } else {
      parked = false;
      if (nowUs - tScanCh > 5000) {
        k = (k + 1) % 24;
        uint8_t c = (k < 16) ? hop[k] : FTEST[k - 16];
        radio.setChannel(c);
        tScanCh = nowUs;
      }
    }
  }
  else { // LOCKED: follow hop, with execuc-style timeout recovery
    uint32_t el = now - tLastValid;
    if (el > 11) {
      uint8_t jump;
      if (errCode == 0)      { jump = el / 8 + 1; errCode = 1; }
      else if (errCode == 1) { jump = 10;         errCode = 2; }
      else                   { jump = 1 + (now % 15); }
      hopIdx = (hopIdx + jump) & 0x0F;
      radio.setChannel(hop[hopIdx]);
      tLastValid = now;   // reset the error clock (same as execuc)
      Serial.print(F("S,jump,"));
      Serial.println(jump);
      // give up to scanning if the stream is really gone
      if (now - tLastAny > 3000) {
        stage = SCAN_ALL;
        Serial.println(F("S,sig_lost"));
      }
    }
  }
}

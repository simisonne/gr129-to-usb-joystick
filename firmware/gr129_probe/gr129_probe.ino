/*
 * GR129 slot walker v14.
 *
 * Confirmed model: one channel per 8ms slot, 16 slots per 128ms cycle,
 * 2 frames per visit -> ~250 frames/s total; a parked receiver hears only
 * its own channel's turn (15.6/s).
 *
 * Anchor on every parked burst (slot 0), then WALK slots 1..15 with forced
 * retunes at period/16 spacing. Predictions come from a learned
 * (parity,slot)->channel table prefilled with a +2/slot guess chain.
 * Bursts re-anchor + toggle parity; 3 empty slots or 10s silence drops us
 * back to PARK / SCOUT.
 *
 * v14 = timing only, table logic untouched:
 *  1. retune WALK_GUARD us BEFORE the predicted slot start, so the radio is
 *     already listening when the pair lands instead of arriving with it
 *  2. that guard self calibrates: we track the earliest in-slot arrival seen
 *     per 10s window and set guard = earliest + 1.2ms margin
 *  3. all serial output is buffered and written at the END of loop(), after
 *     the walk retune, so a slow CDC write can never delay a channel change
 *  4. V lines carry only the 4 axes the feeder reads (shorter write)
 *
 * v17 fixes silently truncated output (the USB TX ring is 256 bytes and the
 *   ~280 byte Q line did not fit, so its tail was dropped and replaced)
 *
 * v16 makes a mistrusted slot recoverable and measures the cycle:
 *  - a wrong entry can never heal while it is trusted (we listen on the wrong
 *    channel, hear nothing, and so record no mismatch that could correct it).
 *    After MISS_DEMOTE consecutive empty visits a slot goes back into search.
 *  - the guard is capped so the pair's second frame keeps 800us of dwell
 *  - `cycms=` reports the mean anchor to anchor interval, which is the only
 *    trustworthy measurement of the controller's real cycle length
 *
 * v15 added one diagnostic only: a per-slot catch histogram in the Q line
 * (`slots=h0/../h15`), so a log shows whether the same slot indices keep
 * delivering and the rest never do, or whether the yielding slots rotate.
 * With conf=16/16 and emin~0 but only ~5 slots yielding per cycle, that is
 * the question that decides what to fix next.
 */
#include <nrf_to_nrf.h>

nrf_to_nrf radio;
static const uint8_t PKT_LEN = 16;
static const uint8_t ADDR_A[5] = {0x6D, 0x6A, 0x73, 0x73, 0x73};
#define GUESS_STEP 2

// the boot banner is the only place a log states its version, so keep it in
// step with the file name (it said v14 for every build from v14 to v17)
#define FW_VERSION "v18"

enum St { SCOUT, PARK, WALK };
St st = SCOUT;

// ---------------- buffered output ----------------
static const uint16_t OB_SZ = 512;
static char ob[OB_SZ];
static uint16_t obn = 0;

// The USB TX ring in this core is 256 bytes and Serial.write() returns how
// many bytes it actually accepted, silently dropping the rest.  The Q line is
// ~280 bytes, so writing it in one go truncated it mid field and the tail was
// replaced by whatever line came next.  Never write more than the ring has
// room for, and keep the remainder for the next loop() pass: the host drains
// the ring in about a millisecond, so nothing is lost and nothing blocks.
static void outFlush() {
  if (!obn) return;
  int room = Serial.availableForWrite();
  if (room <= 0) return;                 // host is behind: keep buffering
  uint16_t n = obn;
  if (n > (uint16_t)room) n = (uint16_t)room;
  size_t wrote = Serial.write((const uint8_t *)ob, n);
  if (wrote < obn) memmove(ob, ob + wrote, obn - wrote);
  obn -= (uint16_t)wrote;
}
static void outCh(char c) {
  if (obn >= OB_SZ - 1) return;          // host gone for seconds: drop, never block
  ob[obn++] = c;
}
static void outS(const char *s) { while (*s) outCh(*s++); }
static void outU(uint32_t v) {
  char t[11]; uint8_t n = 0;
  if (!v) t[n++] = '0';
  while (v) { t[n++] = (char)('0' + (v % 10)); v /= 10; }
  while (n) outCh(t[--n]);
}
static void outI(int32_t v) {
  if (v < 0) { outCh('-'); outU((uint32_t)(-v)); } else outU((uint32_t)v);
}

uint8_t hotCh = 40;
uint8_t tbl[2][16];
uint8_t tblSrc[2][16];      // 0 = guess, 1 = observed
uint16_t mism[2][16];

bool walking = false;
bool awaitBurst = false;
uint8_t wSlot = 0;
uint8_t wMiss = 0;
uint8_t parity = 0;
uint32_t cyc = 0;
uint32_t burstUs = 0, prevBurstUs = 0, tSlot = 0;
uint32_t periodUs = 128000;
uint32_t burstT0 = 0;
uint32_t tLastFrame = 0;
uint32_t tDwell = 0;
uint8_t scoutCh = 2;
uint16_t walkHits = 0, parkHits = 0, walkAborts = 0;
uint16_t slotHit[16];          // frames caught per slot, this 10s window
uint16_t slotMiss[2][16];      // consecutive empty visits to (parity,slot)
uint16_t demotes = 0;          // slots put back into search
bool slotGot = false;          // did the slot we are dwelling on deliver?
uint32_t anchorGapSum = 0;     // anchor to anchor, to measure the real cycle
uint16_t anchorGapN = 0;
#define MISS_DEMOTE 10

// The guard only has to put us in front of the first frame of the pair.  The
// second frame lands ~4ms later, so cap it: a large guard eats the tail of the
// dwell and would drop the second frame of every pair.
static uint32_t guardFromEmin(uint16_t e) {
  uint32_t g = (uint32_t)e + 1200u;
  uint32_t lim = (e + 800u < 4000u) ? (4000u - e - 800u) : 500u;
  if (g > lim) g = lim;
  if (g < 500u) g = 500u;
  if (g > 3500u) g = 3500u;
  return g;
}
uint32_t tStat = 0;

// v14 timing calibration
uint32_t walkGuardUs = 2000;   // retune this early; adapted from eMinUs
uint16_t eMinUs = 0;           // earliest in-slot arrival seen this window
bool eMinSeen = false;
uint16_t eMinLastUs = 0;

static int16_t decAxis(uint8_t b) {
  return (b < 0x80) ? -(int16_t)b : ((int16_t)b - 0x80);
}

void retune(uint8_t ch) {
  radio.stopListening();
  radio.setChannel(ch);
  radio.startListening();
}

uint8_t slotOfNow(uint32_t nowUs) {
  int32_t d = (int32_t)(nowUs - burstUs);
  if (d < 0) d += (int32_t)periodUs;
  if (d >= (int32_t)periodUs) d -= (int32_t)periodUs;
  uint8_t k = (uint8_t)(d / (periodUs / 16));
  return k > 15 ? 15 : k;
}

uint8_t wrapCh(int16_t c) {
  while (c > 100) c -= 99;
  while (c < 2) c += 99;
  return (uint8_t)c;
}

// V,<ms>,<ch>,0,h,<thr>,<yaw>,<pit>,<ail>\n
// fields 5..8 are the four axes (feeder reads p[5..8])
void printV(uint8_t ch, uint32_t now, uint8_t *buf) {
  outS("V,"); outU(now); outCh(',');
  outU(ch); outS(",0,h,");
  outU(buf[0]); outCh(',');
  outI(decAxis(buf[1])); outCh(',');
  outI(decAxis(buf[2])); outCh(',');
  outI(decAxis(buf[3])); outCh('\n');
}

void setState(St s) {
  st = s;
  outS("S,state,");
  outS(s == SCOUT ? "scout" : (s == PARK ? "park" : "walk"));
  outCh('\n');
}

void resetTbl() {
  for (uint8_t p = 0; p < 2; p++) {
    for (uint8_t k = 0; k < 16; k++) {
      tbl[p][k] = wrapCh((int16_t)hotCh + (int16_t)k * GUESS_STEP);
      tblSrc[p][k] = 0;
      mism[p][k] = 0;
      slotMiss[p][k] = 0;
    }
  }
}

void setTbl(uint8_t p, uint8_t k, uint8_t ch) {
  if (k > 15) return;
  if (tblSrc[p][k]) {
    if (tbl[p][k] == ch) return;
    if (++mism[p][k] < 3) return;   // flicker tolerance on observed values
    mism[p][k] = 0;
  }
  for (uint8_t j = 0; j < 16; j++) {
    if (j != k && tblSrc[p][j] && tbl[p][j] == ch &&
        !(tblSrc[p][k] && tbl[p][k] == ch)) {
      if (mism[p][k] < 6) { mism[p][k]++; return; }  // slot taken: retry later
    }
  }
  tbl[p][k] = ch;
  tblSrc[p][k] = 1;
  mism[p][k] = 0;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  outS("S,boot,GR129 walker " FW_VERSION "\n");
  outFlush();
  if (!radio.begin()) {
    Serial.println(F("S,err,begin"));
    while (1) delay(1000);
  }
  radio.setAutoAck(false);
  radio.setPayloadSize(PKT_LEN);
  radio.setCRCLength(NRF_CRC_16);
  radio.setAddressWidth(5);
  radio.setDataRate(NRF_1MBPS);
  radio.openReadingPipe(0, ADDR_A);
  radio.flush_rx();
  retune(hotCh);
  resetTbl();
  tLastFrame = millis();
  tStat = millis();
  tDwell = micros();
  setState(SCOUT);
}

void startWalk() {
  walking = true;
  awaitBurst = false;
  wSlot = 0;                     // still listening on hotCh (slot 0)
  wMiss = 0;
  slotGot = false;
  // land on slot 1's channel WALK_GUARD before slot 1 actually starts
  tSlot = burstUs + periodUs / 16 - walkGuardUs;
}

void endWalk() {
  walking = false;
  awaitBurst = true;
  retune(hotCh);
}

void loop() {
  uint32_t now = millis();
  uint32_t nowUs = micros();

  // ---------------- receive ----------------
  uint8_t pipe;
  while (radio.available(&pipe)) {
    uint8_t buf[PKT_LEN];
    radio.read(buf, PKT_LEN);
    uint8_t ch = radio.getChannel() & 0x7F;
    uint32_t gap = now - tLastFrame;
    tLastFrame = now;

    if (st == SCOUT) {
      hotCh = ch;
      resetTbl();
      parkHits++;
      printV(ch, now, buf);
      burstT0 = now;
      prevBurstUs = nowUs;
      burstUs = nowUs;
      awaitBurst = true;
      setState(PARK);
      retune(hotCh);
      continue;
    }

    bool newBurst;
    if (ch == hotCh) newBurst = (gap > 30) || awaitBurst;
    else             newBurst = (gap > 4000);   // stream truly lost: re-anchor

    if (newBurst) {
      prevBurstUs = burstUs;
      burstUs = nowUs;
      if (prevBurstUs && burstUs > prevBurstUs) {
        anchorGapSum += burstUs - prevBurstUs;
        anchorGapN++;
      }
      // periodUs stays pinned at the protocol constant 128000 (measured
      // 127-128ms everywhere); a measured value latching onto 3 missed
      // anchors once locked the walk at 1/3 speed permanently
      parity ^= 1;
      cyc++;
      burstT0 = now;
      outS("A,"); outU(now); outCh(',');
      outU(ch); outCh(',');
      outU(gap); outCh(',');
      outU(periodUs); outCh('\n');
      if (ch != hotCh) {
        hotCh = ch;
        resetTbl();
      }
      setTbl(parity, 0, ch);
      if (st != WALK) setState(WALK);
      startWalk();
    } else {
      printV(ch, now, buf);
      uint8_t k = walking ? slotOfNow(nowUs) : 0;
      slotHit[k]++;                // which slots actually deliver
      if (walking) {
        slotGot = true;
        setTbl(parity, k, ch);
        if (k == 0) {
          parkHits++;
        } else {
          walkHits++;
          // how late in the slot did this frame land? the smallest value
          // seen over a window is the first-of-pair arrival + loop latency,
          // which is exactly the guard we need to beat it next time
          int32_t e = (int32_t)(nowUs - burstUs) - (int32_t)k * (int32_t)(periodUs / 16);
          if (e < 0) e += (int32_t)periodUs;
          if (e > (int32_t)(periodUs / 16)) e = (int32_t)(periodUs / 16);
          if (!eMinSeen || (uint16_t)e < eMinUs) { eMinUs = (uint16_t)e; eMinSeen = true; }
        }
        wMiss = 0;
      } else {
        parkHits++;
      }
    }
  }

  // ---------------- SCOUT sweep ----------------
  if (st == SCOUT && nowUs - tDwell > 3000) {
    scoutCh++;
    if (scoutCh > 100) scoutCh = 2;
    retune(scoutCh);
    tDwell = nowUs;
  }

  // ---------------- WALK advance (retunes happen before any output) --------
  if (walking && (int32_t)(nowUs - tSlot) >= 0) {
    if (wSlot > 0) {
      if (slotGot) {
        slotMiss[parity][wSlot] = 0;   // it delivered, keep trusting it
      } else {
        wMiss++;                 // completed slot produced nothing
        if (tblSrc[parity][wSlot] &&
            ++slotMiss[parity][wSlot] >= MISS_DEMOTE) {
          tblSrc[parity][wSlot] = 0;   // put it back into search
          slotMiss[parity][wSlot] = 0;
          mism[parity][wSlot] = 0;
          demotes++;
        }
      }
      if (wMiss > 200) {        // safety only; search misses are normal
        walkAborts++;
        endWalk();
        setState(PARK);
        outFlush();
        return;
      }
    }
    wSlot++;
    if (wSlot > 15) {
      endWalk();                 // awaitBurst restarts the walk on arrival
      outFlush();
      return;
    }
    uint8_t pred;
    if (tblSrc[parity][wSlot]) {
      pred = tbl[parity][wSlot];
    } else {
      // strided search: every cycle each open slot tests another candidate;
      // stride 37 coprime with 99 covers the whole band in 97 cycles
      uint16_t c = ((uint32_t)hotCh + cyc * 37u + (uint32_t)wSlot * 13u) % 99u;
      pred = (uint8_t)(2 + c);
    }
    retune(pred);
    tSlot += periodUs / 16;
    slotGot = false;
  }

  // ---------------- silence ----------------
  if (st != SCOUT && now - tLastFrame > 10000) {
    walking = false;
    awaitBurst = false;
    setState(SCOUT);
    retune(scoutCh);
    outS("S,lost,scout\n");
  }

  // ---------------- stats ----------------
  if (now - tStat >= 10000) {
    eMinLastUs = eMinUs;
    if (eMinSeen) {                       // adapt the walk guard
      walkGuardUs = guardFromEmin(eMinUs);
      eMinSeen = false;
    }
    uint32_t cycms = anchorGapN ? (anchorGapSum / anchorGapN) / 1000u : 0;
    anchorGapSum = 0;
    anchorGapN = 0;
    outS("Q,");
    outS(st == SCOUT ? "scout" : (st == PARK ? "park" : "walk"));
    outS(",ch="); outU(hotCh);
    outS(",cyc="); outU(cyc);
    outS(",period="); outU(periodUs);
    outS(",walkH="); outU(walkHits);
    outS(",parkH="); outU(parkHits);
    outS(",aborts="); outU(walkAborts);
    uint8_t cf0 = 0, cf1 = 0;
    for (uint8_t k = 0; k < 16; k++) { if (tblSrc[0][k]) cf0++; if (tblSrc[1][k]) cf1++; }
    outS(",conf="); outU(cf0); outCh('/'); outU(cf1);
    outS(",guard="); outU(walkGuardUs);
    outS(",emin="); outU(eMinLastUs);
    outS(",cycms="); outU(cycms);
    outS(",demote="); outU(demotes);
    outS(",slots=");
    for (uint8_t k = 0; k < 16; k++) { if (k) outCh('/'); outU(slotHit[k]); }
    outS(",p0=");
    for (uint8_t k = 0; k < 16; k++) { if (k) outCh('/'); outU(tbl[0][k]); }
    outS(",p1=");
    for (uint8_t k = 0; k < 16; k++) { if (k) outCh('/'); outU(tbl[1][k]); }
    outCh('\n');
    walkHits = parkHits = walkAborts = 0;
    for (uint8_t k = 0; k < 16; k++) slotHit[k] = 0;
    tStat = now;
  }

  // ---------------- flush (all I/O after the retunes) ----------------
  outFlush();
}

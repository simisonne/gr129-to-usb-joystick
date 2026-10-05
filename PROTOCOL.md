# GR129 / MJX F645 radio protocol

Reverse engineered by listening to the air with a ProMicro nRF52840 running an RF24 style driver
(`nrf_to_nrf`), so this is the nRF24 compatible view of a BK2421 style link.

Everything below was measured rather than guessed, and the numbers are the ones that decided the
design of the sniffer.

## Radio settings

| Setting | Value |
| --- | --- |
| Address | `6D 6A 73 73 73`, address width 5 |
| Data rate | 1 Mbps |
| CRC | 16 bit, hardware |
| Auto ack | off, none observed |
| Payload | static, 16 bytes |

The address looks like a product line constant, `mj` plus three `s` bytes, so it is probably the
same on every unit of this model. A second candidate, `6A 6D 37 37 37`, never matched over a full
capture run. The address is set at the top of the sketch if a unit ever differs.

Channels 2 to 100 are all in use across the band.

## Cycle structure

- The controller runs a 16 slot cycle of 128ms. Measured 125 to 138ms across runs.
- One channel per slot, 8ms per slot.
- Each slot carries a pair of identical 16 byte frames, about 4ms apart.
- That is roughly 250 frames per second on air.
- A receiver parked on a single channel hears only that channel's turn, two frames per cycle, about
  15.6 per second. Parking is the quickest way to confirm a candidate channel.
- The cycle alternates between two hop sequences, so the firmware keeps two tables, one per parity.

The hop sequence is not a simple ladder. Consecutive slot channels are unrelated, which rules out
the `+2 per slot` guess chain that the firmware starts from and any similar shortcut.

## Frame layout, 16 bytes

| Bytes | Meaning |
| --- | --- |
| 0 | throttle, 0 to 255, idles at 0x80 |
| 1 | yaw, signed around 0x80, `value = -b if b < 0x80 else b - 0x80` |
| 2 | pitch, same encoding |
| 3 | roll, same encoding |
| 4 to 6 | trims, 0x40 is centre |
| 7 to 9 | transmitter id, `F8 7D 01` on the unit tested |
| 10 to 13 | zero in every capture |
| 14 | flags, 0xC0 during bind, 0x00 for stick data |
| 15 | checksum, sum of bytes 0 to 14, low byte |

The firmware does not check the transmitter id or the checksum, the hardware CRC over the whole
frame is enough. The id is useful when you want to confirm you are listening to your own unit.

## Binding

1. Power the model first, on a flat surface, and its LED blinks.
2. On the controller, the two throttle side buttons must be in the same position or it cannot
   activate.
3. Power the controller on, then push the throttle stick slowly to full and back once. A beep means
   it activated, and about ten seconds of transmission follows.
4. A solid TX LED means connected, a slow blink means not connected, a fast blink means it is
   sending bind. Rebinding is needed after every controller power off.

## How the sniffer follows the hop

The board anchors on the first frame it hears, then walks the 16 slots in step with the
transmitter. For each slot it retunes, a slot ahead of schedule by a guard interval, to the channel
learned for that slot, and listens.

Three things turned out to matter more than anything else:

1. Retune ahead of the slot, not on it. Retuning on the slot boundary means the radio is still
   settling when the pair starts. The guard is self calibrating: the firmware tracks the earliest
   in slot arrival it sees and keeps the retune just ahead of that.
2. A wrong table entry cannot heal while it is trusted. You listen on the wrong channel, hear
   nothing, and therefore record no mismatch that could trigger a correction. Slots that come up
   empty ten visits in a row are dropped back into a search, which is what lifted capture from
   about 5 of 16 slots to all 16.
3. Do not let serial output delay a retune. The board buffers output and writes it after the
   retune, never in the middle of it.

## Approaches that did not work

| Approach | Result |
| --- | --- |
| Park on the strongest channel | 15.6 frames/s, far too choppy to fly |
| Sticky park plus a full band scout | about 20 frames/s |
| Assume a `+2` channel step per slot | the real sequence is not linear |
| Reuse the V202 hop table from another MJX writeup | different protocol, never matched |
| Trust the hop table once every slot is learned | capture capped at about 5 of 16 slots, see the healing note above |

## Measurement traps that cost the most time

These are about the tools rather than the radio, but they are the reason several earlier attempts
looked like protocol problems.

1. **`serial.read(n)` waits to accumulate n bytes.** `read(512)` at roughly 2 kB/s of frame lines
   hands over about 20 frames every 300ms and stamps them all with the same instant. That is
   roughly 150ms of input lag, and it reads as a feel problem rather than a bug. The signature is a
   gap p50 of 0 next to a p95 near the read timeout. Read what has arrived: `s.read(s.in_waiting or 1)`.
2. **Time frames from the board's clock, not from delivery.** The firmware stamps every frame with
   its own millisecond counter, which is the true arrival time. Calibrate the offset once per
   delivered chunk from the freshest frame in that chunk, then convert every frame in the chunk
   with the same offset. Converting frame by frame while the minimum is still falling drags each
   older frame onto the delivery instant and reproduces exactly the bug the stamp was meant to fix.
3. **The USB TX ring is 256 bytes.** `Serial.write` returns how many bytes it accepted, so a
   280 byte report line silently loses its tail, and the tail is replaced by whatever line is
   written next. That reads like a PC side parser bug and is not. Check
   `Serial.availableForWrite()` and write only what fits.
4. **Cycle rate is worth measuring, not assuming.** A cycle counter that advances slower than the
   walk implies can mean missed anchors or a wrong cycle length, and those need opposite fixes.
   The firmware reports the mean anchor to anchor interval so the length is measured.

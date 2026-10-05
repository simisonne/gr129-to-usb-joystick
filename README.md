# GR129 to USB joystick

Fly a PC simulator with the 2.4GHz controller that came with your MJX GR129 (also sold as the
F645). A ProMicro nRF52840 listens to the controller's radio link, follows its channel hopping,
decodes the four stick axes and hands them to a small Windows program that drives a virtual Xbox
gamepad through ViGEmBus.

No hardware modification of the controller. Nothing is opened, nothing is soldered.

Status: working and flown. Stick input is live in Liftoff with roughly 130 frames per second of
radio capture and about 1ms of USB delay.

## What you need

- An MJX GR129 or F645 controller (the one bundled with the toy heli or drone).
- A ProMicro nRF52840, the nice!nano style board with a UF2 bootloader and no SoftDevice.
- A known good USB data cable.
- A Windows PC with Python 3.

The firmware learns the controller's hop sequence while you move the sticks, so you do not need to
capture a table for your own unit. It was built and tested on one controller, and the protocol
appears to be fixed across the product line, but only one unit has been confirmed. If yours
behaves differently, open an issue with a log.

## Quick start

1. Flash the board. Double tap the reset button, two clicks inside half a second, so the
   bootloader drive appears. Drag `gr129_v17.uf2` onto that drive. The board reboots on its own.
2. Find the port: `py -m serial.tools.list_ports`
3. Set up and run the PC side:

       powershell -ExecutionPolicy Bypass -File setup_and_run.ps1 --serial COM4

   That installs ViGEmBus, `vgamepad` and `pyserial` if they are missing, adds the firewall rule
   for the optional Pi path, then starts the feeder. On the first run Windows will ask about the
   firewall, allow it on private networks.

4. Move the sticks. In the console you should see `pkt/s` climb and the four axis values follow
   your hands. Open your simulator, it sees a normal Xbox 360 pad.

Axis mapping is Mode 2, so throttle is the left stick vertical, yaw is the left stick horizontal,
pitch is the right stick vertical and roll is the right stick horizontal. Every axis has an invert
flag, `--inv-thr`, `--inv-yaw`, `--inv-pit`, `--inv-ail`, if your unit is wired the other way.

The first 30 seconds after a flash are the learning ramp, `pkt/s` climbs from a trickle to full
rate as the hop table fills in. That is normal.

## Running the feeder by hand

    py -m pip install vgamepad pyserial
    py feeder.py --serial COM4

Useful switches:

| Switch | What it does |
| --- | --- |
| `--serial COM4` | read the board directly. Omit it and the feeder listens on UDP 5005 instead, for the Raspberry Pi path |
| `--dry-run` | no virtual pad, print only. Good for testing without ViGEmBus |
| `--ema-ms` | output filter time constant, 10 by default. Lower is more direct, higher is smoother |
| `--deadband` | soft deadband on yaw, pitch and roll, 1.5 raw units by default |
| `--vel-tau-ms` | how long the last known velocity is held across a gap, 100 by default |
| `--mode buffer --delay 0.012` | render a delayed buffer and interpolate instead of extrapolating. Smoothest, costs the delay as extra latency |
| `--rate` | pad push rate, 250Hz by default |
| `--log PATH` | where to write the run log, `feeder.log` beside the script by default |
| `--no-log` | turn the run log off |

`--inv-*` flags, `--no-smooth` and the rest are listed by `py feeder.py --help`.

## The run log

Every run truncates `feeder.log` and writes it fresh, one timestamped line per status line and per
probe line, with a header recording the port and settings. That file is what to attach to an issue.

## Reading the numbers

These are the two numbers that matter, and they measure different things:

    [feeder]  132.3 pkt/s  pad 249.6/s last 0.00s ago  frame gap p50=  4.0 p95= 28.0 max=  48.0ms
              usb delay p50= 0.5 p95=  1.0ms  anchors 16/ 8.0Hz cyc=128ms  last packet 0.0s ago

- `frame gap` is the radio, measured from the board's own clock. It is how far apart the decoded
  frames actually were. Sparse and clumpy is normal for this controller, and the feeder's
  extrapolation exists to cover it.
- `usb delay` is the PC side. It is the difference between when a frame was on air and when this
  program got it. Single digit milliseconds is healthy. Hundreds of milliseconds means the reader
  is batching and every setting downstream is fighting lost time.
- `anchors` and `cyc` are the controller's cycle rate and length, measured independently of the
  firmware. Expect near 8Hz and 128ms.
- `slots=` in the probe line is how many frames each of the 16 cycle slots delivered in the last
  10 seconds. A healthy run spreads across all 16.

The firmware prints a probe line every 10 seconds:

    Q,walk,ch=42,cyc=443,period=128000,walkH=221,parkH=28,aborts=0,conf=15/16,guard=2024,emin=824,
      cycms=128,demote=3,slots=28/0/13/13/10/0/23/44/27/11/14/1/14/29/21/1,p0=...,p1=...

`conf=15/16` is how many of the 16 slots have a learned channel, `demote=` counts slots that were
dropped back into search after coming up empty ten visits in a row, `guard` is how far ahead of a
slot the radio retunes and `emin` is how late in the slot the earliest frame of a pair arrived.
`p0` and `p1` are the two learned hop tables, one per cycle parity.

## How it works

The controller hops a 16 slot cycle of 128ms, one channel per slot, and sends a pair of identical
frames about 4ms apart in each slot, so roughly 250 frames per second go out over the air. A
receiver parked on a single channel only hears that channel's turn, about 15.6 frames per second,
which is far too choppy to fly with.

The firmware therefore does not park. It anchors on the first frame it hears, then walks the 16
slots in step with the transmitter, retuning once per slot to a stored channel, with a guard
interval so the radio is already listening when a slot's pair arrives. It learns which channel
belongs to which slot at runtime, and it keeps two tables because the cycle alternates between two
sequences. A slot that stops delivering is dropped back into a search after ten empty visits,
which matters because a wrong entry can never correct itself while it is trusted: you listen on
the wrong channel, hear nothing, and never get the evidence that would fix it.

Everything the board decodes goes to the PC as one short line per frame with the board's own
millisecond stamp. The feeder times frames from that stamp rather than from when the bytes turned
up, resamples onto a steady 250Hz output, and smooths with a damped velocity estimate so a gap
glides rather than freezing.

Details of the protocol, including the frame layout and the address, are in [PROTOCOL.md](PROTOCOL.md).

## Troubleshooting

**The board shows up as a drive but the flash does nothing.**
Drag the file onto the drive and wait for the copy to finish. The board reboots itself, so the
drive disappearing is the success case, not a failure.

**Two slow clicks do not enter the bootloader.**
They have to be fast, like a mouse double click, under half a second apart. Clicks a second apart
just reboot the running firmware.

**The feeder says it cannot open the port.**
Only one program can hold the serial port. Close anything else that might have it, including an
earlier copy of the feeder. Check the port name, it changes when you use a different USB socket.

**The pad appears but the sticks are centred and never move.**
Check `pkt/s` first. Zero means the radio side is not decoding, usually the controller is off, out
of range, or in bind mode. A healthy stream with a dead pad is nearly always a driver or install
problem, check ViGEmBus is installed and that `import vgamepad` works.

**It flies but feels laggy.**
Read `usb delay` in the status line. If its p95 is in the hundreds of milliseconds, something in
the chain is buffering, and that is worth reporting.

**It flies but feels twitchy.**
Raise `--ema-ms`, for example `--ema-ms 16`. If you would rather trade latency for smoothness,
`--mode buffer --delay 0.015`.

**Windows Firewall.**
Only relevant for the Raspberry Pi path over UDP. The direct USB path does not need it.

## Building the firmware from source

You only need this if you want to change the firmware. The prebuilt UF2 in this repo is the
working build.

    arduino-cli compile --fqbn arduinonrf:nrf52:promicro_nrf52840:bootloader=promicronosduf2 \
      --output-dir build gr129_probe
    python3 uf2conv.py build/gr129_probe.ino.hex -c -f 0xada52840 -o gr129_v17.uf2

The FQBN matters. The `arduinonrf` core, installed from GitHub as `dunknowcoding/ArduinoNRF`, links
the application at 0x1000 and sets up the vector table the way the nice!nano bootloader expects. The
stock Adafruit nRF52 core compiles and even flashes, then hangs before USB comes up.

The board needs the `nrf_to_nrf` library, which gives an RF24 style API on the nRF24 compatible
radio in the nRF52840.

## Files

| File | What it is |
| --- | --- |
| `gr129_v17.uf2` | prebuilt firmware, drag onto the bootloader drive |
| `gr129_probe/gr129_probe.ino` | the firmware source, the slot walker |
| `feeder.py` | the Windows side, resampling and the virtual pad |
| `forwarder.py` | optional Raspberry Pi side, serial to CSV plus UDP JSON |
| `setup_and_run.ps1` | Windows one shot setup and run |
| `sim_smooth.py` | offline harness that replays synthetic streams through the resampler and scores smoothness |
| `uf2conv.py`, `uf2families.json` | UF2 conversion, from Microsoft's uf2 repository |
| `PROTOCOL.md` | the reverse engineered radio protocol |

## Credits

- `uf2conv.py` and `uf2families.json` come from [microsoft/uf2](https://github.com/microsoft/uf2) and
  stay under their MIT license.
- [nrf_to_nrf](https://github.com/TMRh20/nrf_to_nrf) provides the radio API.
- [ViGEmBus](https://github.com/nefarius/ViGEmBus) and
  [vgamepad](https://github.com/yannbouteiller/vgamepad) provide the virtual pad.

## License

MIT, see [LICENSE](LICENSE).

# Wireless CAN bridge (Seeed XIAO nRF54L15 + MCP2515)

Two identical boards, each on its own CAN bus, forwarding every frame to the other over a
2.4 GHz radio link (Nordic ESB). The goal is to replicate the Kvaser Air Bridge. Zephyr / nRF
Connect SDK **v3.1.0**, board `xiao_nrf54l15/nrf54l15/cpuapp`, CAN controller MCP2515 on SPI
(Seeed CAN bus breakout, `spi00`, 4 MHz, 16 MHz crystal, 500 kbit/s).

## Build, flash, read logs (Windows)

```powershell
. .\ncs-env.ps1          # puts the NCS v3.1.0 toolchain and west on PATH
west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build_a --no-sysbuild
west flash -d build_a --runner openocd --cmd-pre-init "cmsis_dap_serial <SERIAL>"   # one board at a time
```

- NCS v3.1.0 does not ship the XIAO nRF54L15 board, so its definition lives in
  `boards/seeed/xiao_nrf54l15/` (copied from Seeed's platform-seeedboards repo) and `CMakeLists.txt`
  adds the project folder to `BOARD_ROOT`. No `-DBOARD_ROOT` argument is needed. Don't point the
  nRF Connect extension at the whole Seeed repo: it lists boards (e.g. nRF54LM20B) that this SDK
  doesn't know and shows an error. Always build with `--no-sysbuild`.
- The boards flash and log over their onboard CMSIS-DAP. **Serial output needs DTR asserted**
  (a plain terminal that doesn't set it shows nothing), and only one program can open a COM port:
  close VS Code serial terminals before capturing from a script.
- Flashing does not erase the stored pairing (it lives in the `storage` partition).
- Use separate build directories for different configurations (`build_a` = production, others
  are git-ignored scratch). Never leave a debug build on the boards without saying so.
- Do not run `west` from a nested `powershell -c ...`: script execution is disabled there.
- A reused build directory keeps old Kconfig values: after changing a `default` in `Kconfig`
  rebuild with `--pristine` (a stale `BRIDGE_PLAYOUT_DELAY_US` bit us once). Check
  `build_a/zephyr/.config` when a behaviour doesn't match the source.
- When passing a value to `-D` from a PowerShell variable, quote the whole argument:
  `"-DCONFIG_X=$value"`.

## Behaviour

- **Pairing:** short press of the user button on both boards (within 30 s) pairs them; LED blinks
  fast. Hold 5 s to forget the pairing. Roles are automatic: lower device id = master (ESB PTX).
  The pairing (peer id + random key) is stored with Zephyr settings/ZMS.
- **Link:** fixed **3 ms** TDD cycle. Each cycle is one ESB exchange on a new channel: master
  packet, and the slave's frames come back in the ACK payload. Up to ~15 full CAN frames per
  packet each way. Hop table (16 slots from a 26-channel pool) is adapted by the master to
  interference and sent in every packet; a slave that loses the master scans the whole pool.
- **Reliability:** frames carry a 16-bit index; the receiver accepts only the next expected index.
  Master to slave uses the radio's own ACK; slave to master uses go-back-N with the master's
  echoed `ack_idx` (see `esb_link.c`).
- **Timestamps:** every frame carries its age. The receiving board schedules it
  `BRIDGE_PLAYOUT_DELAY_US` (20 ms) after it arrived on the source bus, preserving frame spacing.
  Boards' clocks are never synchronised; only locally measured intervals are exchanged.
- **CAN side:** queues drop the *oldest* frame when full; `can_send` never blocks for more than
  5 ms. Stats (rates, drops, late frames, `can_send` time) print every 5 s over the console.

## Code map

| File | Role |
|---|---|
| `src/main.c` | init, queue definitions |
| `src/can_communication.c` | MCP2515 rx callback (timestamps), tx thread (playout), stats |
| `src/esb_communication.c` | core: events/timers, HF clock, `radio_setup`, public API, radio thread |
| `src/esb_link.c` | send queue, go-back-N, TDD cycle, timestamps |
| `src/esb_afh.c` | adaptive frequency hopping, link address derivation |
| `src/esb_pairing.c` | pairing handshake, persistent storage |
| `src/esb_ui.c` | button, LED, statistics, synthetic traffic |
| `include/esb_internal.h` | tunables, wire format, shared state of the esb_*.c files |

Everything on the radio side runs in one cooperative-priority thread (`esb_tx_thread`), woken by an
event word; ISRs only post events. Keep it that way.

## Hard-won lessons (do not re-learn these)

- Call `esb_stop_rx()` before every `esb_init()`; without it the next transmit never completes.
- On the PRX (slave) side never `esb_flush_tx()` before queuing a new ACK payload. ESB keeps the
  sent payload at the FIFO front until the next new packet and then pops it; a flush makes it
  discard the new payload and send an empty ACK.
- ESB does no frequency hopping or channel adaptation at all; that is all our code.
- The 2.4 GHz band on the bench is busy with Wi-Fi: channels 2405-2450 MHz fail 20-45 %, 2460-2480
  MHz 1-7 %. Long packets are far more exposed than short ones.
- SPI stays at 4 MHz. 8 MHz only made `can_send` ~10 % faster (the time is bus waiting) and the
  user saw corrupted IDs at 6 MHz on the dev kit. nRF54L15 `spi00` can do 32 MHz, the other SPI
  instances only 8 MHz.
- The build uses `-std=c99`: no `typeof`, declare named structs.
- `bridge_now_us()` wraps after ~71 min; always compare with `bridge_us_diff()`.

## Bench setup and measured results

- Board A: id `0xafd00aaa`, CMSIS-DAP `5BF86608`, usually **COM5**, becomes **master**.
  Board B: id `0xfc0eb4df`, `CB3564E9`, usually **COM6**, becomes **slave**, sits on the CAN bus
  where the user replays a ~1800 frame/s (70 % load) log. The replay sometimes stops; check the
  `CAN rx` rate before blaming the firmware. The user occasionally unplugs a board.
- One direction, 70 % load: all frames delivered, none lost in steady state, ~14-16 % of radio
  cycles lost to interference, source-to-destination delay avg 7.6 ms (max 17 ms).
- Radio-only stress test (`BRIDGE_SYNTHETIC_TRAFFIC` + `_ISOLATE`, synthetic 8-byte frames, no
  CAN bus involved), both directions at the same time:
  - 1800 frames/s each way (the 70 % load figure): all delivered, ~14 % of cycles lost, queues
    stay around 35 frames (~20 ms), few link drops after startup. Looks the same as one direction.
  - 6000 frames/s each way (saturation): the link delivers about **3000 frames/s per direction
    (~6000 aggregate)**, 41 % of cycles lost (two full 250-byte packets per cycle are very exposed
    to interference), and the link drops ~160-180 times a minute (8 consecutive lost cycles) and
    recovers each time. The CAN bus itself tops out around 3300-3800 frames/s per direction at
    100 % load, so a fully loaded bus both ways would not quite fit.
  - Not yet measured: both CAN buses loaded at once (each MCP2515 then does ~1800 rx and ~1800 tx
    per second over one 4 MHz SPI). Watch the `overrun` counter and `can_send` time there.

## Bench / debug Kconfig options (all default off)

`BRIDGE_COUNTER_CHECK` (+ `BRIDGE_COUNTER_ID`, default `0x14FF009E`: the user's J1939 test
message, 4-bit counter at bit 8) reports missing frames at both ends.
`BRIDGE_AUTO_PAIR_ON_BOOT`, `BRIDGE_SYNTHETIC_TRAFFIC` (+ `_RATE` frames/s per direction,
`_ISOLATE` = radio-only test that ignores and never transmits on the CAN bus),
`BRIDGE_PLAYOUT_DELAY_US`.

## Not done / ideas

Start the hop table from the best channels (and store them), encrypt or authenticate pairing,
adaptive cycle length, README/sample.yaml still describe the old CAN sample, split-bus
bidirectional test with real traffic on both buses.

# Wireless CAN bridge

Two identical boards, each on its own CAN bus, forwarding every frame to the other over a
2.4 GHz radio link. The goal is an open replacement for the Kvaser Air Bridge.

- **Hardware:** [Seeed XIAO nRF54L15](https://wiki.seeedstudio.com/xiao_nrf54l15_sense_getting_started/)
  + MCP2515 CAN controller (Seeed CAN bus breakout, SPI 4 MHz, 500 kbit/s).
- **Software:** Zephyr / nRF Connect SDK **v3.1.0**, Nordic ESB radio protocol.

## How it works

- **Pairing:** short-press the user button on both boards within 30 s. Hold 5 s to forget the
  pairing. The pairing is stored in flash. The board with the lower device id becomes master.
- **Link:** a fixed 3 ms cycle, each on a new channel. The master sends a packet and the slave's
  frames come back in the ACK, up to about 15 CAN frames each way per cycle. The master adapts the
  hop table to interference.
- **Reliability:** frames carry an index and are retransmitted until acknowledged, so none are
  lost or reordered.
- **Timing:** every frame is replayed on the far bus 20 ms after it arrived on the source bus, with
  the original spacing preserved. The two boards' clocks are never synchronised.
- **Measured:** at 70 % bus load (~1800 frames/s) all frames are delivered with 7.6 ms average
  delay. The link saturates at about 3000 frames/s per direction.

## Build and flash (Windows)

```powershell
. .\ncs-env.ps1     # NCS v3.1.0 toolchain on PATH; edit the path inside if yours differs
west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build_a --no-sysbuild
west flash -d build_a --runner openocd --cmd-pre-init "cmsis_dap_serial <SERIAL>"
```

Flash one board at a time. The board definition is included in `boards/seeed/`, because NCS 3.1.0
doesn't ship it. Always build with `--no-sysbuild`. Console output (statistics every 5 s) needs DTR
asserted.

Bench and test options (synthetic traffic, counter check, ...) are in [Kconfig](Kconfig).

## Source layout

| File | Role |
|---|---|
| `src/main.c` | init, queues |
| `src/can_communication.c` | MCP2515 rx/tx, playout timing, statistics |
| `src/esb_communication.c` | radio core and public API |
| `src/esb_link.c` | send queue, retransmission, TDD cycle |
| `src/esb_afh.c` | adaptive frequency hopping |
| `src/esb_pairing.c` | pairing and persistent storage |
| `src/esb_ui.c` | button, LED, statistics |

See [CLAUDE.md](CLAUDE.md) for detailed notes and hard-won lessons.

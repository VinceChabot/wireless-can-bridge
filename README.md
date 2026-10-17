# Wireless CAN bridge

Two identical boards, each on its own CAN bus, forwarding every frame to the other over a
2.4 GHz radio link.

- **Hardware:** 
  - [Seeed XIAO nRF54L15](https://wiki.seeedstudio.com/xiao_nrf54l15_sense_getting_started/)
  - MCP2515 CAN controller
- **Software:**
  - Zephyr
  - nRF Connect SDK

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

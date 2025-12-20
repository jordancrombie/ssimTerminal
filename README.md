# ssimTerminal

ESP32-based hardware payment terminal for displaying QR codes and transaction status. Part of the SSIM (Simple Store Simulator) ecosystem.

## Overview

ssimTerminal is firmware for an ESP32-S3 device that serves as a dedicated, customer-facing payment display. When a merchant initiates a mobile wallet payment through SSIM, the terminal:

1. Receives the payment request via WebSocket
2. Displays a QR code for the customer to scan with their mobile wallet (mwsim)
3. Shows real-time transaction status (approved/declined/error)
4. Returns to idle state ready for the next transaction

The terminal is intentionally "dumb" - it handles display only, with all payment logic residing on the SSIM server.

## Hardware

**Target Device:** [Waveshare ESP32-S3-Touch-AMOLED-1.8](https://www.waveshare.com/esp32-s3-touch-amoled-1.8.htm)

| Component | Specification |
|-----------|---------------|
| MCU | ESP32-S3R8 (Dual-core LX7, 240MHz) |
| RAM | 512KB SRAM + 8MB PSRAM |
| Flash | 16MB |
| Display | 1.8" AMOLED, 368x448 pixels |
| Display Driver | SH8601 (QSPI) |
| Touch | FT3168 capacitive (I2C) |
| Connectivity | WiFi 2.4GHz, Bluetooth 5 |
| Power | AXP2101 PMU, optional LiPo battery |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                           SSIM Server                           │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐   │
│  │   Express    │    │  WebSocket   │    │    Terminal      │   │
│  │   Routes     │◄──►│   Server     │◄──►│    Registry      │   │
│  │  (REST API)  │    │   (wss://)   │    │  (Map<id, ws>)   │   │
│  └──────────────┘    └──────────────┘    └──────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ▲
                              │ WebSocket (wss://)
                              │
                        ┌─────┴─────┐
                        │ ssimTerminal │
                        │  (ESP32)  │
                        └───────────┘
```

### Terminal States

```
BOOT → WIFI_SETUP → PAIRING → CONNECTING → IDLE ⇄ QR_DISPLAY → RESULT → IDLE
```

- **BOOT**: Hardware initialization
- **WIFI_SETUP**: WiFi network selection and password entry
- **PAIRING**: First-time setup with SSIM server (entering pairing code)
- **CONNECTING**: Establishing WebSocket connection
- **IDLE**: Ready for payment, showing store branding
- **QR_DISPLAY**: Showing QR code with amount and countdown
- **RESULT**: Payment outcome (3 seconds), then return to IDLE

## Development Setup

### Prerequisites

- [VSCode](https://code.visualstudio.com/)
- [PlatformIO Extension](https://platformio.org/install/ide?install=vscode)
- USB-C cable for flashing

### Getting Started

```bash
# Clone the repository
git clone git@github.com:jordancrombie/ssimTerminal.git
cd ssimTerminal

# Open in VSCode with PlatformIO
code .

# Build and upload (connect ESP32 via USB)
pio run -t upload

# Monitor serial output
pio device monitor
```

### Project Structure

```
ssimTerminal/
├── platformio.ini          # PlatformIO configuration
├── include/
│   ├── pins_config.h       # GPIO pin definitions for Waveshare board
│   ├── lv_conf.h           # LVGL configuration
│   ├── display.h           # Display driver interface
│   └── touch.h             # Touch controller interface
├── src/
│   ├── main.cpp            # Main firmware entry point
│   ├── display.cpp         # SH8601 QSPI display driver
│   └── touch.cpp           # FT3168 I2C touch driver
└── lib/                    # Local libraries
```

### Tech Stack

| Component | Choice |
|-----------|--------|
| Framework | Arduino (via PlatformIO) |
| Display Driver | Arduino_GFX (SH8601 QSPI) |
| Graphics | LVGL v8.3.x |
| QR Generation | ricmoo/QRCode |
| WebSocket Client | links2004/WebSockets |
| JSON | ArduinoJson v7 |

## WebSocket Protocol

### Connection

```
wss://<ssim-server>/terminal/ws?apiKey=<api_key>
```

### Messages (Server → Terminal)

| Type | Description |
|------|-------------|
| `display_qr` | Show QR code with payment details |
| `payment_result` | Transaction outcome (approved/declined/failed/cancelled/expired) |
| `clear` | Return to idle screen |
| `config` | Update terminal configuration |

### Messages (Terminal → Server)

| Type | Description |
|------|-------------|
| `heartbeat` | Keep-alive with device telemetry |
| `status` | Current screen state |
| `ack` | Message acknowledgment |

See [SSIM Terminal Integration Proposal](https://github.com/jordancrombie/ssim/blob/main/LOCAL_DEPLOYMENT_PLANS/SSIM_TERMINAL_INTEGRATION_PROPOSAL.md) for full protocol specification.

## Implementation Roadmap

### Phase 1 (Current)

- [x] PlatformIO project setup
- [x] LVGL configuration
- [x] Main firmware skeleton with state machine
- [x] WebSocket client integration
- [x] Secure storage for API key (NVS)
- [x] SH8601 display driver (via Arduino_GFX + TCA9554 expander)
- [x] FT3168 touch driver (I2C)
- [x] Basic UI screens (boot, idle, connecting, result, error)
- [x] WiFi provisioning UI (network scan, selection, password keyboard)
- [x] QR code generation and display
- [x] Environment selector (Development/Production toggle)
- [ ] Pairing flow UI (code entry screen)
- [ ] Hardware testing and pin verification

### Phase 2 (Future)

- [ ] OTA firmware updates
- [ ] Sound feedback (ES8311 audio codec)
- [ ] Low-power/sleep modes
- [ ] Multi-language support

## Related Projects

| Project | Description | Repository |
|---------|-------------|------------|
| **SSIM** | Simple Store Simulator (merchant POS) | [jordancrombie/ssim](https://github.com/jordancrombie/ssim) |
| **WSIM** | Wallet Simulator API (payment processing) | - |
| **mwsim** | Mobile Wallet Simulator (customer app) | - |

## License

TBD

## Contributing

This project is in early development. See the integration proposal for current status and open questions.

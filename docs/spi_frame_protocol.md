# SPI Camera Frame Protocol (Linux Master <-> ESP32-CAM Slave)

## 1. Overview

This document defines the SPI communication protocol between a Linux-based
master and an ESP32-CAM (slave). The protocol is designed to be configurable
and robust for transferring camera frames.

- **Master**: Any Linux SBC (Raspberry Pi, Milk-V Duo / SG2002, Orange Pi, etc.)
- **Slave**: ESP32-CAM with OV3660 (or OV2640) camera module
- **Communication**: Half-duplex. The master sends a command, and the slave responds with an acknowledgment or data.
- **Data Integrity**: CRC checks are used for both commands and data frames to ensure reliability.

## 2. Control Commands (Master -> Slave)

All control commands sent from the master to the slave are **16 bytes** long.

### Command Structure

| Offset | Size | Field           | Description                                       |
|--------|------|-----------------|---------------------------------------------------|
| 0      | 1    | `SOF`           | Start of Frame (Constant: `0xA5`)                 |
| 1      | 1    | `CMD_ID`        | Command Identifier (e.g., `0x01` for `SET_FMT`)   |
| 2      | 1    | `SEQ`           | Sequence number, incremented by the master        |
| 3      | 1    | `LEN`           | Payload length in bytes (0-8)                     |
| 4      | 8    | `PAYLOAD`       | Command-specific payload data                     |
| 12     | 1    | `CRC8`          | CRC8 checksum of bytes from offset 0 to 11        |
| 13     | 1    | `EOF`           | End of Frame (Constant: `0x5A`)                   |
| 14     | 2    | `RESERVED`      | Reserved for future use, should be `0x0000`       |

### Command Set (`CMD_ID`)

| ID     | Command             | Payload Description                                                                                             |
|--------|---------------------|-----------------------------------------------------------------------------------------------------------------|
| `0x01` | `SET_FMT`           | `pixfmt(1)`, `width(2)`, `height(2)`, `quality(1, for JPEG)`. Sets the image format and resolution.             |
| `0x02` | `SET_MAX_FRAME_SIZE`| `max_bytes(4)`. Informs the slave of the maximum frame size the master can receive in one transfer.              |
| `0x03` | `SET_FPS`           | `fps(1)`. Sets the target frames per second for streaming.                                                      |
| `0x04` | `CAPTURE_ONCE`      | `flags(1)`. Requests a single frame. Flags can specify compression, etc.                                        |
| `0x05` | `STREAM_ON`         | No payload. Starts continuous frame streaming.                                                                  |
| `0x06` | `STREAM_OFF`        | No payload. Stops frame streaming.                                                                              |
| `0x07` | `GET_STATUS`        | No payload. Requests the slave's status.                                                                        |
| `0x08` | `RESET`             | No payload. Resets the ESP32-CAM application.                                                                   |

---

## 3. Acknowledgment (Slave -> Master)

For every command received, the slave must send an **8-byte** acknowledgment (ACK).

### ACK Structure

| Offset | Size | Field      | Description                                       |
|--------|------|------------|---------------------------------------------------|
| 0      | 1    | `SOF`      | Start of Frame (Constant: `0xCC`)                 |
| 1      | 1    | `CMD_ID`   | The `CMD_ID` of the command being acknowledged    |
| 2      | 1    | `SEQ`      | The `SEQ` of the command being acknowledged       |
| 3      | 1    | `STATUS`   | `0x00` for OK, other values for error codes       |
| 4      | 2    | `INFO`     | Additional status information (e.g., error details) |
| 6      | 1    | `CRC8`     | CRC8 checksum of bytes from offset 0 to 5         |
| 7      | 1    | `EOF`      | End of Frame (Constant: `0x33`)                   |

---

## 4. Data Frame (Slave -> Master)

When the slave sends a camera frame, it uses the following structure, which consists of a header, a payload (the image data), and an optional trailer.

### Frame Header (32 bytes)

| Offset | Size | Field          | Description                                                                                             |
|--------|------|----------------|---------------------------------------------------------------------------------------------------------|
| 0      | 2    | `SOF`          | Start of Frame (Constant: `0xF1F2`)                                                                     |
| 2      | 1    | `VER`          | Protocol version (e.g., `0x01`)                                                                         |
| 3      | 1    | `FLAGS`        | Bit flags: `b0`: JPEG, `b1`: Data CRC32 enabled, `b2`: Keyframe, `b7`: Error frame                      |
| 4      | 2    | `SEQ`          | Frame sequence number                                                                                   |
| 6      | 2    | `WIDTH`        | Image width in pixels (little-endian)                                                                   |
| 8      | 2    | `HEIGHT`       | Image height in pixels (little-endian)                                                                  |
| 10     | 1    | `PIXFMT`       | Pixel Format: `0`:RGB565, `1`:RGB888, `2`:YUV422, `3`:JPEG                                                |
| 11     | 1    | `RESERVED`     | Reserved                                                                                                |
| 12     | 4    | `FRAME_SIZE`   | Size of the frame data (payload) in bytes (little-endian)                                               |
| 16     | 4    | `TIMESTAMP`    | Capture timestamp in milliseconds or microseconds (little-endian)                                       |
| 20     | 4    | `HEADER_CRC32` | CRC32 checksum of the header from offset 0 to 19                                                        |
| 24     | 4    | `RESERVED`     | Reserved                                                                                                |
| 28     | 2    | `EOF`          | End of Frame (Constant: `0x2F2E`)                                                                       |
| 30     | 2    | `RESERVED`     | Reserved                                                                                                |

### Frame Payload

The actual image data, with a length of `FRAME_SIZE` bytes, immediately follows the header.

### Frame Trailer (Optional, 4 bytes)

If the `FLAGS.bit1` is set, a 4-byte CRC32 checksum of the `Frame Payload` is appended at the end of the data.

| Field        | Size | Description                  |
|--------------|------|------------------------------|
| `DATA_CRC32` | 4    | CRC32 checksum of the payload |

## 5. SPI Transfer Flow Example (Capture Once)

1.  **Master -> Slave**: Master sends a `CAPTURE_ONCE` command (16 bytes).
2.  **Slave -> Master**: Slave immediately sends an ACK (8 bytes) to confirm receipt.
3.  **Slave**: ESP32 captures an image and prepares the data frame.
4.  **Master**: Master initiates a read transaction.
5.  **Slave -> Master**: Slave sends the `Frame Header` (32 bytes).
6.  **Master**: Master parses the header, checks `HEADER_CRC32`, and reads `FRAME_SIZE` to determine how much more data to read.
7.  **Slave -> Master**: Slave sends the `Frame Payload` and optional `Frame Trailer`.
8.  **Master**: Master reads the payload and optional trailer, verifying the `DATA_CRC32` if present.

This completes the transfer of a single frame.

---

## 6. Hardware Setup

### ESP32-CAM Slave (AI-Thinker board + OV3660 module)

The OV3660 uses the same physical GPIO pin assignments as the stock OV2640 on
the AI-Thinker ESP32-CAM. No hardware re-wiring is needed when substituting an
OV3660 module — the `esp_camera` library auto-detects the sensor over SCCB.

| Signal     | ESP32 GPIO |
|------------|-----------|
| PWDN       | 32        |
| XCLK       | 0         |
| SIOD (SDA) | 26        |
| SIOC (SCL) | 27        |
| D0–D7      | 5,18,19,21,36,39,34,35 |
| VSYNC      | 25        |
| HREF       | 23        |
| PCLK       | 22        |
| SPI CS     | 15        |
| SPI SCLK   | 14        |
| SPI MOSI   | 13        |
| SPI MISO   | 12        |
| Handshake  | 16        |

### Linux Master Wiring

Connect the ESP32-CAM SPI pins to the Linux board's SPI controller and one
free GPIO for the handshake (ready) line:

| Linux board signal | ESP32-CAM |
|--------------------|-----------|
| SPI SCLK           | GPIO 14   |
| SPI MOSI           | GPIO 13   |
| SPI MISO           | GPIO 12   |
| SPI CS0            | GPIO 15   |
| GPIO input (e.g. GPIO 24) | GPIO 16 (Handshake) |
| GND                | GND       |

### Building and Running the Linux Master

```sh
# Build
cd src/linux_master
make

# Capture a 640×480 JPEG and save it to out.jpg
sudo ./linux_spi_master -d /dev/spidev0.0 -s 10000000 -g 24 \
                        -w 640 -h 480 -q 10 -o out.jpg
```

Available options:

| Option | Default         | Description                          |
|--------|-----------------|--------------------------------------|
| `-d`   | /dev/spidev0.0  | SPI device node                      |
| `-s`   | 10000000        | SPI clock speed in Hz                |
| `-g`   | 24              | Handshake GPIO number on Linux board |
| `-w`   | 320             | Requested frame width (pixels)       |
| `-h`   | 240             | Requested frame height (pixels)      |
| `-q`   | 12              | JPEG quality (1–63, lower = bigger)  |
| `-o`   | frame.jpg       | Output file name                     |


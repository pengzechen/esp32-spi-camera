/**
 * @file spi_protocol.h
 * @brief Defines the SPI communication protocol structures and constants.
 */

#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

// Protocol Constants
#define CMD_SOF 0xA5
#define CMD_EOF 0x5A
#define ACK_SOF 0xCC
#define ACK_EOF 0x33
#define FRAME_SOF1 0xF1
#define FRAME_SOF2 0xF2
#define FRAME_EOF1 0x2F
#define FRAME_EOF2 0x2E

#define PROTOCOL_VERSION 1

// Command IDs
enum spi_cmd_id {
    CMD_ID_SET_FMT = 0x01,
    CMD_ID_SET_MAX_FRAME_SIZE = 0x02,
    CMD_ID_SET_FPS = 0x03,
    CMD_ID_CAPTURE_ONCE = 0x04,
    CMD_ID_STREAM_ON = 0x05,
    CMD_ID_STREAM_OFF = 0x06,
    CMD_ID_GET_STATUS = 0x07,
    CMD_ID_RESET = 0x08,
};

// Pixel Formats
enum spi_pixformat {
    PIXFMT_RGB565 = 0,
    PIXFMT_RGB888 = 1,
    PIXFMT_YUV422 = 2,
    PIXFMT_JPEG = 3,
};

// Frame Flags
#define FRAME_FLAG_JPEG (1 << 0)
#define FRAME_FLAG_DATA_CRC32 (1 << 1)
#define FRAME_FLAG_KEYFRAME (1 << 2)
#define FRAME_FLAG_ERROR (1 << 7)

// ACK Status
enum ack_status {
    ACK_OK = 0x00,
    ACK_ERR_CRC = 0x01,
    ACK_ERR_CMD = 0x02,
    ACK_ERR_BUSY = 0x03,
};

#pragma pack(push, 1)

// Master -> Slave Command Structure (16 bytes)
typedef struct {
    uint8_t sof;
    uint8_t cmd_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[8];
    uint8_t crc8;
    uint8_t eof;
    uint16_t reserved;
} spi_command_t;

// Slave -> Master ACK Structure (8 bytes)
typedef struct {
    uint8_t sof;
    uint8_t cmd_id;
    uint8_t seq;
    uint8_t status;
    uint16_t info;
    uint8_t crc8;
    uint8_t eof;
} spi_ack_t;

// Slave -> Master Frame Header Structure (32 bytes)
typedef struct {
    uint8_t sof[2];
    uint8_t ver;
    uint8_t flags;
    uint16_t seq;
    uint16_t width;
    uint16_t height;
    uint8_t pixfmt;
    uint8_t reserved1;
    uint32_t frame_size;
    uint32_t timestamp;
    uint32_t header_crc32;
    uint32_t reserved2;
    uint8_t eof[2];
    uint16_t reserved3;
} spi_frame_header_t;

#pragma pack(pop)

// CRC calculation function prototypes (to be implemented)
uint8_t crc8(const uint8_t *data, size_t len);
uint32_t crc32(const uint8_t *data, size_t len);

#endif // SPI_PROTOCOL_H

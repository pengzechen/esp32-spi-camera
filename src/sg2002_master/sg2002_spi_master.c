/**
 * @file sg2002_spi_master.c
 * @brief SG2002 SPI Master - Pseudo-code Implementation
 */

#include "spi_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

// Platform-specific SPI and GPIO functions (to be implemented by user)
void spi_init_master();
void spi_cs_low();
void spi_cs_high();
uint8_t spi_transfer_byte(uint8_t byte);
void spi_transfer_block(uint8_t* tx_buf, uint8_t* rx_buf, size_t len);

// Hardware handshake & delay functions (to be implemented by user)
int gpio_read_handshake_pin(); // Returns 1 if HIGH, 0 if LOW
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

// Polling the Ready/Handshake Pin with Timeout
bool wait_for_slave_ready(uint32_t timeout_ms) {
    uint32_t elapsed = 0;
    while (gpio_read_handshake_pin() == 0) {
        if (elapsed >= timeout_ms) return false;
        delay_ms(1);
        elapsed++;
    }
    return true;
}

// --- CRC Implementation (Example) ---
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint32_t crc32(const uint8_t *data, size_t len) {
    // A proper CRC32 implementation should be used.
    // This is a placeholder.
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}


// --- Protocol Functions ---

static uint8_t command_seq = 0;

/**
 * @brief Sends a command and waits for an ACK.
 * @return 0 on success, -1 on error.
 */
int send_command_and_wait_ack(spi_command_t* cmd, spi_ack_t* ack) {
    cmd->seq = command_seq++;
    cmd->sof = CMD_SOF;
    cmd->eof = CMD_EOF;
    cmd->crc8 = crc8((uint8_t*)cmd, sizeof(spi_command_t) - 4); // Exclude CRC and beyond

    spi_cs_low();
    spi_transfer_block((uint8_t*)cmd, NULL, sizeof(spi_command_t));
    spi_cs_high(); // Crucial: terminate the command transfer
    
    // Wait a short time for slave's FreeRTOS task to process command and queue ACK
    delay_ms(2); 

    spi_cs_low(); // Begin new transaction for ACK
    spi_transfer_block(NULL, (uint8_t*)ack, sizeof(spi_ack_t));
    spi_cs_high();

    // Validate ACK
    uint8_t received_crc = ack->crc8;
    ack->crc8 = 0; // Zero out for calculation
    if (received_crc != crc8((uint8_t*)ack, sizeof(spi_ack_t) - 2)) {
        printf("ACK CRC error!\n");
        return -1;
    }
    if (ack->sof != ACK_SOF || ack->eof != ACK_EOF) {
        printf("ACK SOF/EOF error!\n");
        return -1;
    }
    if (ack->seq != cmd->seq) {
        printf("ACK sequence mismatch!\n");
        return -1;
    }
    if (ack->status != ACK_OK) {
        printf("Slave returned error status: %d\n", ack->status);
        return -1;
    }

    return 0;
}

/**
 * @brief Receives a single frame from the slave using the Two-Stage Handshake.
 * @param buffer A buffer to store the frame data.
 * @param buffer_size The size of the buffer.
 * @return The size of the received frame, or -1 on error.
 */
int receive_frame(uint8_t* buffer, uint32_t buffer_size) {
    spi_frame_header_t header;
    
    // --- STAGE 1: Wait for and read Frame Header ---
    printf("Waiting for Slave Header Ready...\n");
    if (!wait_for_slave_ready(2000)) { // 2 seconds timeout for capture
        printf("Timeout waiting for Frame Header!\n");
        return -1;
    }

    spi_cs_low();
    // Master sends dummy bytes to read the header
    spi_transfer_block(NULL, (uint8_t*)&header, sizeof(spi_frame_header_t));
    spi_cs_high(); // CRITICAL: Stop SPI transaction after header
    
    // Validate Header
    if (header.sof[0] != FRAME_SOF1 || header.sof[1] != FRAME_SOF2 ||
        header.eof[0] != FRAME_EOF1 || header.eof[1] != FRAME_EOF2) {
        printf("Frame header SOF/EOF error! [%02x %02x ... %02x %02x]\n", 
                header.sof[0], header.sof[1], header.eof[0], header.eof[1]);
        return -1;
    }

    uint32_t received_crc = header.header_crc32;
    header.header_crc32 = 0;
    if (received_crc != crc32((uint8_t*)&header, 20)) {
        printf("Frame header CRC error!\n");
        return -1;
    }
    
    if (header.flags & FRAME_FLAG_ERROR) {
        printf("Slave indicated an error frame.\n");
        return -1;
    }

    if (header.frame_size > buffer_size) {
        printf("Frame size (%u) exceeds buffer size (%u)!\n", header.frame_size, buffer_size);
        return -1; // Abort, we can't save it. Next capture will reset slave state.
    }

    // --- STAGE 2: Wait for and read Frame Payload ---
    printf("Header Valid! Size=%u. Waiting for Slave Payload Ready...\n", header.frame_size);
    if (!wait_for_slave_ready(500)) { // Normally very quick after header
        printf("Timeout waiting for Frame Payload!\n");
        return -1;
    }

    spi_cs_low();
    
    // Read frame data
    spi_transfer_block(NULL, buffer, header.frame_size);

    // Handle ESP32 4-byte padding requirement (flush the padding bytes)
    // The ESP32 SPI Slave driver forces the transaction length out to a multiple of 4 bytes.
    uint32_t padded_size = (header.frame_size + 3) & ~3;
    uint32_t padding_bytes = padded_size - header.frame_size;
    for (uint32_t i = 0; i < padding_bytes; i++) {
        spi_transfer_byte(0xFF);
    }

    // Read and verify data CRC if present
    if (header.flags & FRAME_FLAG_DATA_CRC32) {
        uint32_t data_crc_from_slave;
        spi_transfer_block(NULL, (uint8_t*)&data_crc_from_slave, 4);
        if (data_crc_from_slave != crc32(buffer, header.frame_size)) {
            printf("Frame data CRC error!\n");
            spi_cs_high();
            return -1;
        }
    }

    spi_cs_high(); // Transaction complete

    printf("Frame successfully received: %dx%d, size=%u, seq=%u\n", header.width, header.height, header.frame_size, header.seq);
    return header.frame_size;
}


// --- Main Application Logic (Example) ---

#define FRAME_BUFFER_SIZE (320 * 240 * 2) // Example for QVGA RGB565

int main() {
    spi_init_master();

    uint8_t* frame_buffer = (uint8_t*)malloc(FRAME_BUFFER_SIZE);
    if (!frame_buffer) {
        printf("Failed to allocate frame buffer.\n");
        return -1;
    }

    spi_command_t cmd;
    spi_ack_t ack;

    // 1. Configure the camera format
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_id = CMD_ID_SET_FMT;
    cmd.len = 6;
    cmd.payload[0] = PIXFMT_JPEG; // format
    *(uint16_t*)&cmd.payload[1] = 320; // width
    *(uint16_t*)&cmd.payload[3] = 240; // height
    cmd.payload[5] = 12; // quality
    if (send_command_and_wait_ack(&cmd, &ack) != 0) {
        printf("Failed to set format.\n");
        free(frame_buffer);
        return -1;
    }
    printf("Camera format set successfully.\n");

    // 2. Request a single frame
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_id = CMD_ID_CAPTURE_ONCE;
    cmd.len = 0;
    if (send_command_and_wait_ack(&cmd, &ack) != 0) {
        printf("Failed to send capture command.\n");
        free(frame_buffer);
        return -1;
    }
    printf("Capture command sent, waiting for frame...\n");

    // 3. Receive the frame
    // The slave needs time to capture, so a delay or a ready signal is needed.
    // For simplicity, we just wait.
    // delay_ms(100); 
    
    int frame_size = receive_frame(frame_buffer, FRAME_BUFFER_SIZE);
    if (frame_size > 0) {
        printf("Successfully received a frame of %d bytes.\n", frame_size);
        // Process the frame_buffer...
    } else {
        printf("Failed to receive frame.\n");
    }

    free(frame_buffer);
    return 0;
}

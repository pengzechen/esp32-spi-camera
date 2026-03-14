/* Must be defined before any system headers to expose POSIX extensions
 * (struct timespec, nanosleep, usleep, etc.) under -std=c99. */
#define _POSIX_C_SOURCE 200112L

/**
 * @file linux_spi_master.c
 * @brief Linux SPI Master for ESP32-CAM SPI Slave
 *
 * Implements the platform-specific SPI and GPIO functions using standard
 * Linux kernel interfaces so this master runs on any Linux SBC
 * (Raspberry Pi, Milk-V Duo / SG2002, Orange Pi, etc.).
 *
 * SPI  : /dev/spidevX.Y  (Linux spidev)
 * GPIO : /sys/class/gpio  (sysfs – no extra libraries required)
 * Time : nanosleep()
 *
 * Build:
 *   cd src/linux_master && make
 *
 * Usage (must run as root or with SPI/GPIO permissions):
 *   sudo ./linux_spi_master [options]
 *
 * Options:
 *   -d <device>    SPI device node   (default: /dev/spidev0.0)
 *   -s <hz>        SPI clock speed   (default: 10000000  = 10 MHz)
 *   -g <num>       Handshake GPIO number that the ESP32 drives HIGH when
 *                  data is ready     (default: 24)
 *   -w <pixels>    Requested frame width   (default: 320)
 *   -h <pixels>    Requested frame height  (default: 240)
 *   -q <1-63>      JPEG quality            (default: 12)
 *   -o <file>      Output file for the captured JPEG (default: frame.jpg)
 */

#include "../spi_protocol.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

/* -------------------------------------------------------------------------
 * Global configuration (set via command-line options in main())
 * ---------------------------------------------------------------------- */
static int      spi_fd          = -1;
static uint32_t spi_speed_hz    = 10000000; /* 10 MHz */
static char     spi_device[64]  = "/dev/spidev0.0";
static int      handshake_gpio  = 24;       /* GPIO number on the Linux board */
static char     gpio_value_path[128];       /* filled by gpio_init_handshake() */

/* -------------------------------------------------------------------------
 * Platform: SPI (Linux spidev)
 * ---------------------------------------------------------------------- */

void spi_init_master(void)
{
    spi_fd = open(spi_device, O_RDWR);
    if (spi_fd < 0) {
        perror("spi_init_master: open");
        exit(EXIT_FAILURE);
    }

    uint8_t  mode  = SPI_MODE_0;
    uint8_t  bits  = 8;
    uint32_t speed = spi_speed_hz;

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("spi_init_master: SPI_IOC_WR_MODE");
        exit(EXIT_FAILURE);
    }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("spi_init_master: SPI_IOC_WR_BITS_PER_WORD");
        exit(EXIT_FAILURE);
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("spi_init_master: SPI_IOC_WR_MAX_SPEED_HZ");
        exit(EXIT_FAILURE);
    }

    printf("SPI master initialised: %s @ %u Hz\n", spi_device, spi_speed_hz);
}

/* With spidev the kernel toggles CS automatically for each ioctl transfer,
 * so these are intentional no-ops. */
void spi_cs_low(void)  {}
void spi_cs_high(void) {}

uint8_t spi_transfer_byte(uint8_t tx_byte)
{
    uint8_t rx_byte = 0;
    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)&tx_byte,
        .rx_buf        = (unsigned long)&rx_byte,
        .len           = 1,
        .speed_hz      = spi_speed_hz,
        .bits_per_word = 8,
        .delay_usecs   = 0,
    };
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("spi_transfer_byte");
    }
    return rx_byte;
}

void spi_transfer_block(uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
    if (len == 0) return;

    /* spidev requires valid pointer for both tx and rx sides of a transfer.
     * Allocate a zeroed temporary buffer for whichever side is NULL. */
    uint8_t *tmp_tx = NULL;
    uint8_t *tmp_rx = NULL;

    if (tx_buf == NULL) {
        tmp_tx = (uint8_t *)calloc(len, 1);
        if (!tmp_tx) { perror("spi_transfer_block: calloc tx"); return; }
        tx_buf = tmp_tx;
    }
    if (rx_buf == NULL) {
        tmp_rx = (uint8_t *)calloc(len, 1);
        if (!tmp_rx) { perror("spi_transfer_block: calloc rx"); free(tmp_tx); return; }
        rx_buf = tmp_rx;
    }

    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)tx_buf,
        .rx_buf        = (unsigned long)rx_buf,
        .len           = (uint32_t)len,
        .speed_hz      = spi_speed_hz,
        .bits_per_word = 8,
        .delay_usecs   = 0,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("spi_transfer_block");
    }

    free(tmp_tx);
    free(tmp_rx);
}

/* -------------------------------------------------------------------------
 * Platform: GPIO (Linux sysfs)
 * ---------------------------------------------------------------------- */

static void gpio_export(int gpio_num)
{
    char path[64];
    /* Skip if already exported */
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", gpio_num);
    if (access(path, F_OK) == 0) return;

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        perror("gpio_export: open /sys/class/gpio/export");
        exit(EXIT_FAILURE);
    }
    char buf[16];
    int  n = snprintf(buf, sizeof(buf), "%d", gpio_num);
    if (write(fd, buf, n) < 0) {
        /* EBUSY means it's already exported – treat as success */
        if (errno != EBUSY) {
            perror("gpio_export: write");
            close(fd);
            exit(EXIT_FAILURE);
        }
    }
    close(fd);
}

static void gpio_set_direction(int gpio_num, const char *direction)
{
    char path[128];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio_num);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("gpio_set_direction: open");
        /* Direction file may not be available immediately after export;
         * wait briefly and retry once. */
        struct timespec wait = { .tv_sec = 0, .tv_nsec = 100000000L }; /* 100 ms */
        nanosleep(&wait, NULL);
        fd = open(path, O_WRONLY);
        if (fd < 0) { perror("gpio_set_direction: open retry"); exit(EXIT_FAILURE); }
    }
    if (write(fd, direction, strlen(direction)) < 0) {
        perror("gpio_set_direction: write");
    }
    close(fd);
}

void gpio_init_handshake(int gpio_num)
{
    gpio_export(gpio_num);
    gpio_set_direction(gpio_num, "in");
    snprintf(gpio_value_path, sizeof(gpio_value_path),
             "/sys/class/gpio/gpio%d/value", gpio_num);
    printf("Handshake GPIO %d configured as input (%s)\n",
           gpio_num, gpio_value_path);
}

int gpio_read_handshake_pin(void)
{
    char val[4] = {0};
    int  fd = open(gpio_value_path, O_RDONLY);
    if (fd < 0) return 0;
    int n = read(fd, val, sizeof(val) - 1);
    close(fd);
    return (n > 0) ? atoi(val) : 0;
}

/* -------------------------------------------------------------------------
 * Platform: Timing
 * ---------------------------------------------------------------------- */

void delay_us(uint32_t us)
{
    struct timespec ts = {
        .tv_sec  = (time_t)(us / 1000000U),
        .tv_nsec = (long)((us % 1000000U) * 1000UL),
    };
    nanosleep(&ts, NULL);
}

void delay_ms(uint32_t ms)
{
    struct timespec ts = {
        .tv_sec  = (time_t)(ms / 1000U),
        .tv_nsec = (long)((ms % 1000U) * 1000000UL),
    };
    nanosleep(&ts, NULL);
}

/* -------------------------------------------------------------------------
 * CRC (must match the ESP32 slave implementation)
 * ---------------------------------------------------------------------- */

uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
        }
    }
    return ~crc;
}

/* -------------------------------------------------------------------------
 * Protocol helpers
 * ---------------------------------------------------------------------- */

static uint8_t command_seq = 0;

/** Polls the handshake pin until HIGH or timeout. */
static bool wait_for_slave_ready(uint32_t timeout_ms)
{
    for (uint32_t elapsed = 0; elapsed < timeout_ms; elapsed++) {
        if (gpio_read_handshake_pin()) return true;
        delay_ms(1);
    }
    return false;
}

/**
 * @brief Sends a command and waits for an ACK.
 * @return 0 on success, -1 on error.
 */
static int send_command_and_wait_ack(spi_command_t *cmd, spi_ack_t *ack)
{
    cmd->seq = command_seq++;
    cmd->sof = CMD_SOF;
    cmd->eof = CMD_EOF;
    cmd->crc8 = crc8((uint8_t *)cmd, sizeof(spi_command_t) - 4);

    /* Send command */
    spi_cs_low();
    spi_transfer_block((uint8_t *)cmd, NULL, sizeof(spi_command_t));
    spi_cs_high();

    /* Give the ESP32 FreeRTOS task time to process the command */
    delay_ms(2);

    /* Read ACK */
    spi_cs_low();
    spi_transfer_block(NULL, (uint8_t *)ack, sizeof(spi_ack_t));
    spi_cs_high();

    /* Validate ACK */
    uint8_t recv_crc = ack->crc8;
    ack->crc8 = 0;
    if (recv_crc != crc8((uint8_t *)ack, sizeof(spi_ack_t) - 2)) {
        fprintf(stderr, "send_command_and_wait_ack: ACK CRC error\n");
        return -1;
    }
    if (ack->sof != ACK_SOF || ack->eof != ACK_EOF) {
        fprintf(stderr, "send_command_and_wait_ack: ACK SOF/EOF error "
                "[sof=%02x eof=%02x]\n", ack->sof, ack->eof);
        return -1;
    }
    if (ack->seq != cmd->seq) {
        fprintf(stderr, "send_command_and_wait_ack: ACK sequence mismatch "
                "(got %u, expected %u)\n", ack->seq, cmd->seq);
        return -1;
    }
    if (ack->status != ACK_OK) {
        fprintf(stderr, "send_command_and_wait_ack: slave error status %u\n",
                ack->status);
        return -1;
    }
    return 0;
}

/**
 * @brief Receives a single frame using the Two-Stage Handshake.
 * @param buffer      Buffer to store the frame payload.
 * @param buffer_size Size of buffer.
 * @return Payload size in bytes, or -1 on error.
 */
static int receive_frame(uint8_t *buffer, uint32_t buffer_size)
{
    spi_frame_header_t header;

    /* --- Stage 1: wait for and read the Frame Header --- */
    printf("Waiting for slave header ready signal...\n");
    if (!wait_for_slave_ready(2000)) {
        fprintf(stderr, "receive_frame: timeout waiting for Frame Header\n");
        return -1;
    }

    spi_cs_low();
    spi_transfer_block(NULL, (uint8_t *)&header, sizeof(spi_frame_header_t));
    spi_cs_high();

    if (header.sof[0] != FRAME_SOF1 || header.sof[1] != FRAME_SOF2 ||
        header.eof[0] != FRAME_EOF1 || header.eof[1] != FRAME_EOF2) {
        fprintf(stderr, "receive_frame: header SOF/EOF error "
                "[%02x %02x ... %02x %02x]\n",
                header.sof[0], header.sof[1],
                header.eof[0], header.eof[1]);
        return -1;
    }

    uint32_t recv_crc = header.header_crc32;
    header.header_crc32 = 0;
    if (recv_crc != crc32((uint8_t *)&header, 20)) {
        fprintf(stderr, "receive_frame: header CRC32 mismatch\n");
        return -1;
    }

    if (header.flags & FRAME_FLAG_ERROR) {
        fprintf(stderr, "receive_frame: slave flagged an error frame\n");
        return -1;
    }

    if (header.frame_size > buffer_size) {
        fprintf(stderr, "receive_frame: frame size %u exceeds buffer %u\n",
                header.frame_size, buffer_size);
        return -1;
    }

    /* --- Stage 2: wait for and read the Frame Payload --- */
    printf("Header OK: %ux%u %u-byte JPEG (seq=%u). Waiting for payload...\n",
           header.width, header.height, header.frame_size, header.seq);

    if (!wait_for_slave_ready(500)) {
        fprintf(stderr, "receive_frame: timeout waiting for Frame Payload\n");
        return -1;
    }

    spi_cs_low();

    spi_transfer_block(NULL, buffer, header.frame_size);

    /* The ESP32 SPI slave driver pads every transaction to a 4-byte boundary;
     * flush any padding bytes so the CS line is released cleanly. */
    uint32_t padded   = (header.frame_size + 3u) & ~3u;
    uint32_t pad_left = padded - header.frame_size;
    for (uint32_t i = 0; i < pad_left; i++) {
        spi_transfer_byte(0xFF);
    }

    /* Verify optional data CRC32 if the slave attached one */
    if (header.flags & FRAME_FLAG_DATA_CRC32) {
        uint32_t data_crc_from_slave = 0;
        spi_transfer_block(NULL, (uint8_t *)&data_crc_from_slave, 4);
        if (data_crc_from_slave != crc32(buffer, header.frame_size)) {
            fprintf(stderr, "receive_frame: data CRC32 mismatch\n");
            spi_cs_high();
            return -1;
        }
    }

    spi_cs_high();

    printf("Frame received: %ux%u, %u bytes, seq=%u\n",
           header.width, header.height, header.frame_size, header.seq);
    return (int)header.frame_size;
}

/* -------------------------------------------------------------------------
 * main()
 * ---------------------------------------------------------------------- */

/* Maximum frame buffer: 256 KB covers any JPEG the OV3660 is likely to
 * produce within the ESP32's DMA buffer (currently 128 KB). */
#define FRAME_BUFFER_SIZE (256 * 1024)

static void write_le16(uint8_t *dest, uint16_t val)
{
    dest[0] = (uint8_t)(val & 0xFFu);
    dest[1] = (uint8_t)(val >> 8);
}

static void print_usage(const char *prog)
{    printf("Usage: %s [options]\n"
           "  -d <device>   SPI device  (default: /dev/spidev0.0)\n"
           "  -s <hz>       SPI speed   (default: 10000000)\n"
           "  -g <gpio>     Handshake GPIO number (default: 24)\n"
           "  -w <width>    Frame width  in pixels (default: 320)\n"
           "  -h <height>   Frame height in pixels (default: 240)\n"
           "  -q <quality>  JPEG quality 1-63      (default: 12)\n"
           "  -o <file>     Output JPEG file        (default: frame.jpg)\n",
           prog);
}

int main(int argc, char *argv[])
{
    uint16_t    req_width   = 320;
    uint16_t    req_height  = 240;
    uint8_t     req_quality = 12;
    char        out_file[256] = "frame.jpg";
    int         opt;

    while ((opt = getopt(argc, argv, "d:s:g:w:h:q:o:")) != -1) {
        switch (opt) {
            case 'd': strncpy(spi_device, optarg, sizeof(spi_device) - 1); break;
            case 's': spi_speed_hz = (uint32_t)atoi(optarg); break;
            case 'g': handshake_gpio = atoi(optarg); break;
            case 'w': req_width  = (uint16_t)atoi(optarg); break;
            case 'h': req_height = (uint16_t)atoi(optarg); break;
            case 'q': req_quality = (uint8_t)atoi(optarg); break;
            case 'o': strncpy(out_file, optarg, sizeof(out_file) - 1); break;
            default:
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    /* 1. Initialise hardware */
    spi_init_master();
    gpio_init_handshake(handshake_gpio);

    /* 2. Allocate frame buffer */
    uint8_t *frame_buf = (uint8_t *)malloc(FRAME_BUFFER_SIZE);
    if (!frame_buf) {
        fprintf(stderr, "Failed to allocate %d-byte frame buffer\n",
                FRAME_BUFFER_SIZE);
        return EXIT_FAILURE;
    }

    spi_command_t cmd;
    spi_ack_t     ack;

    /* 3. Set camera format (pixfmt, width, height, quality) */
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_id     = CMD_ID_SET_FMT;
    cmd.len        = 6;
    cmd.payload[0] = PIXFMT_JPEG;
    write_le16(&cmd.payload[1], req_width);
    write_le16(&cmd.payload[3], req_height);
    cmd.payload[5] = req_quality;

    printf("Setting format: %ux%u JPEG quality=%u ...\n",
           req_width, req_height, req_quality);
    if (send_command_and_wait_ack(&cmd, &ack) != 0) {
        fprintf(stderr, "Failed to set camera format\n");
        free(frame_buf);
        return EXIT_FAILURE;
    }
    printf("Camera format set successfully.\n");

    /* 4. Request a single capture */
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_id = CMD_ID_CAPTURE_ONCE;
    cmd.len    = 0;

    printf("Sending CAPTURE_ONCE command...\n");
    if (send_command_and_wait_ack(&cmd, &ack) != 0) {
        fprintf(stderr, "Failed to send CAPTURE_ONCE command\n");
        free(frame_buf);
        return EXIT_FAILURE;
    }
    printf("Capture command acknowledged. Waiting for frame...\n");

    /* 5. Receive the frame */
    int frame_size = receive_frame(frame_buf, FRAME_BUFFER_SIZE);
    if (frame_size <= 0) {
        fprintf(stderr, "Failed to receive frame\n");
        free(frame_buf);
        return EXIT_FAILURE;
    }

    /* 6. Save to file */
    FILE *fp = fopen(out_file, "wb");
    if (!fp) {
        perror("fopen output file");
        free(frame_buf);
        return EXIT_FAILURE;
    }
    if (fwrite(frame_buf, 1, (size_t)frame_size, fp) != (size_t)frame_size) {
        perror("fwrite");
        fclose(fp);
        free(frame_buf);
        return EXIT_FAILURE;
    }
    fclose(fp);

    printf("Frame saved to '%s' (%d bytes).\n", out_file, frame_size);

    free(frame_buf);
    close(spi_fd);
    return EXIT_SUCCESS;
}

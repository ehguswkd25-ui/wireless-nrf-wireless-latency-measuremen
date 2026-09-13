#include "nrf24l01_v2.h"
#include <util/delay.h>

#define CMD_R_REGISTER    0x00
#define CMD_W_REGISTER    0x20
#define CMD_R_RX_PAYLOAD  0x61
#define CMD_W_TX_PAYLOAD  0xA0
#define CMD_FLUSH_TX      0xE1
#define CMD_FLUSH_RX      0xE2
#define CMD_NOP           0xFF

#define REG_CONFIG        0x00
#define REG_EN_AA         0x01
#define REG_EN_RXADDR     0x02
#define REG_SETUP_AW      0x03
#define REG_SETUP_RETR    0x04
#define REG_RF_CH         0x05
#define REG_RF_SETUP      0x06
#define REG_STATUS        0x07
#define REG_OBSERVE_TX    0x08
#define REG_RX_ADDR_P0    0x0A
#define REG_TX_ADDR       0x10
#define REG_RX_PW_P0      0x11
#define REG_FIFO_STATUS   0x17

#define STATUS_RX_DR      (1 << 6)
#define STATUS_TX_DS      (1 << 5)
#define STATUS_MAX_RT     (1 << 4)

static nrf_dev_t *active_dev; /* CSN 제어용으로 현재 SPI 트랜잭션 중인 디바이스 */

static void ce_high(nrf_dev_t *d) { *(d->ce_port)  |=  (1 << d->ce_pin); }
static void ce_low(nrf_dev_t *d)  { *(d->ce_port)  &= ~(1 << d->ce_pin); }
static void csn_high(nrf_dev_t *d){ *(d->csn_port) |=  (1 << d->csn_pin); }
static void csn_low(nrf_dev_t *d) { *(d->csn_port) &= ~(1 << d->csn_pin); }

void spi_init(void)
{
    DDRB |= (1 << PB1) | (1 << PB2); /* SCK, MOSI = output (SPI 공유버스) */
    DDRB &= ~(1 << PB3);             /* MISO = input */
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); /* Master, Fosc/16 */
}

static uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

void nrf_dev_init(nrf_dev_t *dev,
                   volatile uint8_t *ce_ddr, volatile uint8_t *ce_port, uint8_t ce_pin,
                   volatile uint8_t *csn_ddr, volatile uint8_t *csn_port, uint8_t csn_pin)
{
    dev->ce_port = ce_port;
    dev->ce_pin = ce_pin;
    dev->csn_port = csn_port;
    dev->csn_pin = csn_pin;

    *ce_ddr  |= (1 << ce_pin);
    *csn_ddr |= (1 << csn_pin);
    ce_low(dev);
    csn_high(dev);
}

static uint8_t nrf_read_reg(nrf_dev_t *dev, uint8_t reg)
{
    uint8_t val;
    csn_low(dev);
    spi_transfer(CMD_R_REGISTER | reg);
    val = spi_transfer(CMD_NOP);
    csn_high(dev);
    return val;
}

static void nrf_write_reg(nrf_dev_t *dev, uint8_t reg, uint8_t val)
{
    csn_low(dev);
    spi_transfer(CMD_W_REGISTER | reg);
    spi_transfer(val);
    csn_high(dev);
}

static void nrf_write_reg_buf(nrf_dev_t *dev, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    csn_low(dev);
    spi_transfer(CMD_W_REGISTER | reg);
    for (uint8_t i = 0; i < len; i++) spi_transfer(buf[i]);
    csn_high(dev);
}

static void nrf_common_init(nrf_dev_t *dev, uint8_t channel, const uint8_t *addr5)
{
    nrf_write_reg(dev, REG_EN_AA, 0x01);      /* pipe0 Auto-ACK 켜짐 */
    nrf_write_reg(dev, REG_SETUP_AW, 0x03);   /* 주소 5바이트 */
    nrf_write_reg(dev, REG_SETUP_RETR, 0x2F); /* 재전송 대기 750us, 최대 15회 */
    nrf_write_reg(dev, REG_RF_CH, channel);
    nrf_write_reg(dev, REG_RF_SETUP, 0x06);   /* 1Mbps, 0dBm */
    nrf_write_reg(dev, REG_RX_PW_P0, NRF_PAYLOAD_SIZE);
    nrf_write_reg_buf(dev, REG_RX_ADDR_P0, addr5, 5);
    nrf_write_reg_buf(dev, REG_TX_ADDR, addr5, 5); /* TX도 P0 주소로 ACK 수신 */
    nrf_write_reg(dev, REG_EN_RXADDR, 0x01);

    csn_low(dev); spi_transfer(CMD_FLUSH_TX); csn_high(dev);
    csn_low(dev); spi_transfer(CMD_FLUSH_RX); csn_high(dev);
    nrf_write_reg(dev, REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
}

void nrf_configure_tx(nrf_dev_t *dev, uint8_t channel, const uint8_t *addr5)
{
    nrf_common_init(dev, channel, addr5);
    nrf_write_reg(dev, REG_CONFIG, 0x0E); /* PWR_UP=1, PRIM_RX=0 (TX), CRC 1byte */
    _delay_ms(5);
}

void nrf_configure_rx(nrf_dev_t *dev, uint8_t channel, const uint8_t *addr5)
{
    nrf_common_init(dev, channel, addr5);
    nrf_write_reg(dev, REG_CONFIG, 0x0F); /* PWR_UP=1, PRIM_RX=1 (RX) */
    _delay_ms(5);
    ce_high(dev); /* 수신 대기 시작, ACK 응답은 하드웨어가 자동으로 처리 */
}

void nrf_set_channel(nrf_dev_t *dev, uint8_t channel)
{
    nrf_write_reg(dev, REG_RF_CH, channel); /* 이 레지스터 쓰면 PLOS_CNT 자동 리셋됨 */
}

uint8_t nrf_send_and_wait(nrf_dev_t *dev, const uint8_t *data)
{
    ce_low(dev);
    csn_low(dev);
    spi_transfer(CMD_W_TX_PAYLOAD);
    for (uint8_t i = 0; i < NRF_PAYLOAD_SIZE; i++) spi_transfer(data[i]);
    csn_high(dev);

    ce_high(dev);
    _delay_us(15);
    ce_low(dev);

    uint16_t timeout = 20000;
    while (timeout--) {
        uint8_t status = nrf_read_reg(dev, REG_STATUS);
        if (status & STATUS_TX_DS) {
            nrf_write_reg(dev, REG_STATUS, STATUS_TX_DS);
            return 1; /* ACK 수신 성공 */
        }
        if (status & STATUS_MAX_RT) {
            nrf_write_reg(dev, REG_STATUS, STATUS_MAX_RT);
            csn_low(dev); spi_transfer(CMD_FLUSH_TX); csn_high(dev);
            return 0; /* 재시도 소진, 실패 */
        }
    }
    return 0; /* 타임아웃 (이론상 거의 안 옴) */
}

uint8_t nrf_data_ready(nrf_dev_t *dev)
{
    return (nrf_read_reg(dev, REG_STATUS) & STATUS_RX_DR) ? 1 : 0;
}

void nrf_receive(nrf_dev_t *dev, uint8_t *data)
{
    csn_low(dev);
    spi_transfer(CMD_R_RX_PAYLOAD);
    for (uint8_t i = 0; i < NRF_PAYLOAD_SIZE; i++) data[i] = spi_transfer(CMD_NOP);
    csn_high(dev);
    nrf_write_reg(dev, REG_STATUS, STATUS_RX_DR);
}

uint8_t nrf_read_observe_tx(nrf_dev_t *dev)
{
    return nrf_read_reg(dev, REG_OBSERVE_TX);
}

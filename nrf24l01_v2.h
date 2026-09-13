#ifndef NRF24L01_H
#define NRF24L01_H

#include <avr/io.h>
#include <stdint.h>

#define NRF_PAYLOAD_SIZE 4  /* [seq_hi][seq_lo][marker][checksum] */

/* 모듈 하나를 나타내는 구조체 - 보드 1개에 모듈 2개를 동시에 물릴 때
 * 이 구조체를 2개 만들어서 각각 다른 CE/CSN 핀을 지정하면 됨.
 * SPI(SCK/MOSI/MISO)는 두 모듈이 공유. */
typedef struct {
    volatile uint8_t *ce_port;
    uint8_t ce_pin;
    volatile uint8_t *csn_port;
    uint8_t csn_pin;
} nrf_dev_t;

void spi_init(void);

/* ce_ddr/csn_ddr: 해당 핀의 DDR 레지스터 (예: &DDRG)
 * ce_port/csn_port: 해당 핀의 PORT 레지스터 (예: &PORTG) */
void nrf_dev_init(nrf_dev_t *dev,
                   volatile uint8_t *ce_ddr, volatile uint8_t *ce_port, uint8_t ce_pin,
                   volatile uint8_t *csn_ddr, volatile uint8_t *csn_port, uint8_t csn_pin);

/* 송신(TX) 모드 설정 - Auto-ACK 켜짐 (지연시간 측정을 위해 필수) */
void nrf_configure_tx(nrf_dev_t *dev, uint8_t channel, const uint8_t *addr5);

/* 수신(RX) 모드 설정 - Auto-ACK 켜짐 (하드웨어가 자동으로 ACK 응답, MCU 개입 불필요) */
void nrf_configure_rx(nrf_dev_t *dev, uint8_t channel, const uint8_t *addr5);

/* 채널만 바꿀 때 (재초기화 없이) - PLOS_CNT가 이때 0으로 리셋됨 */
void nrf_set_channel(nrf_dev_t *dev, uint8_t channel);

/* 패킷 전송, CE 펄스를 준 시점부터 TX_DS/MAX_RT 뜰 때까지 폴링.
 * 반환값: 1=ACK 수신 성공, 0=실패(MAX_RT, 재시도 소진)
 * out_ticks: 이 함수 안에서 소요된 타이머1 카운트값 (호출 전 TCNT1을 0으로 리셋해두고 호출) */
uint8_t nrf_send_and_wait(nrf_dev_t *dev, const uint8_t *data);

uint8_t nrf_data_ready(nrf_dev_t *dev);
void nrf_receive(nrf_dev_t *dev, uint8_t *data);

/* OBSERVE_TX 레지스터 읽기: 상위4비트=PLOS_CNT(누적손실), 하위4비트=ARC_CNT(마지막 전송 재시도횟수) */
uint8_t nrf_read_observe_tx(nrf_dev_t *dev);

#endif

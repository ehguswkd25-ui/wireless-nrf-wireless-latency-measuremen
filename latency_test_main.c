/*
 * 무선 통신 지연시간 / 패킷 손실률 / 채널별 간섭 테스트
 * - ATmega128 보드 1개에 NRF24L01 모듈 2개를 동시에 연결해서 진행
 * - 모듈 A = TX 역할, 모듈 B = RX 역할 (Auto-ACK 하드웨어 기능 사용)
 * - Timer1으로 "CE 펄스 시작 ~ ACK 수신 확인(TX_DS)까지" 왕복 지연시간 측정
 * - 결과를 UART로 CSV 형태로 출력 -> PC에서 pc_logger.py로 저장/그래프화
 *
 * 배선 (SPI는 공유, CE/CSN만 별도 핀)
 *   공통  SCK/MOSI/MISO/GND는 보드의 ISP 6핀 헤더에서 인출
 *         (표준 배치: 1=MISO, 2=VCC, 3=SCK, 4=MOSI, 5=RESET, 6=GND)
 *   모듈A(TX) CE=PF2, CSN=PF3   (J3 커넥터에서 인출)
 *   모듈B(RX) CE=PF4, CSN=PF5   (J3 커넥터에서 인출)
 *   두 모듈 다 VCC는 3.3V(레귤레이터 통해), GND 공통, 100uF(or 10uF 여러개 병렬) 커패시터 VCC-GND
 */
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "nrf24l01_v2.h"

#define BAUD 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD) - 1)

#define PACKETS_PER_CHANNEL 30

static const uint8_t pipe_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t test_channels[] = {2, 40, 76, 100, 120};
#define NUM_CHANNELS (sizeof(test_channels) / sizeof(test_channels[0]))

static nrf_dev_t devTX, devRX;

/* ===== UART (결과를 PC로 전송) ===== */
static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_print(const char *s)
{
    while (*s) uart_putc(*s++);
}

/* ===== Timer1: 왕복 지연시간 측정용 (프리스케일러 8 -> 0.5us/tick @16MHz) ===== */
static void timer1_init(void)
{
    TCCR1A = 0;
    TCCR1B = (1 << CS11); /* 분주비 8 */
}

int main(void)
{
    uart_init();
    spi_init();
    timer1_init();

    /* 모듈A = TX (CE=PF2, CSN=PF3), 모듈B = RX (CE=PF4, CSN=PF5) - J3 커넥터에서 인출 */
    nrf_dev_init(&devTX, &DDRF, &PORTF, PF2, &DDRF, &PORTF, PF3);
    nrf_dev_init(&devRX, &DDRF, &PORTF, PF4, &DDRF, &PORTF, PF5);

    nrf_configure_tx(&devTX, test_channels[0], pipe_addr);
    nrf_configure_rx(&devRX, test_channels[0], pipe_addr);

    _delay_ms(200);

    uart_print("channel,total,success,fail,avg_latency_us,plos_cnt,arc_cnt\n");

    while (1) {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            uint8_t channel = test_channels[ch];

            nrf_set_channel(&devTX, channel); /* PLOS_CNT 리셋됨 */
            nrf_set_channel(&devRX, channel);
            _delay_ms(50); /* 채널 안정화 대기 */

            uint16_t success = 0, fail = 0;
            uint32_t latency_sum_ticks = 0;

            for (uint16_t seq = 0; seq < PACKETS_PER_CHANNEL; seq++) {
                uint8_t payload[4];
                payload[0] = (uint8_t)(seq >> 8);
                payload[1] = (uint8_t)(seq & 0xFF);
                payload[2] = 0xAA; /* 마커 */
                payload[3] = payload[0] ^ payload[1] ^ payload[2];

                TCNT1 = 0;
                uint8_t ok = nrf_send_and_wait(&devTX, payload);
                uint16_t ticks = TCNT1;

                if (ok) {
                    success++;
                    latency_sum_ticks += ticks;
                } else {
                    fail++;
                }

                /* RX 모듈의 FIFO를 비워줌 (안 비우면 3개 쌓인 뒤 이후 패킷 드롭될 수 있음) */
                if (nrf_data_ready(&devRX)) {
                    uint8_t rxbuf[4];
                    nrf_receive(&devRX, rxbuf);
                }

                _delay_ms(20); /* 패킷 간 간격 */
            }

            uint8_t observe = nrf_read_observe_tx(&devTX);
            uint8_t plos_cnt = (observe >> 4) & 0x0F;
            uint8_t arc_cnt = observe & 0x0F;

            uint32_t avg_latency_us = success ? (latency_sum_ticks / success) / 2 : 0;

            char line[64];
            sprintf(line, "%u,%u,%u,%u,%lu,%u,%u\n",
                    channel, PACKETS_PER_CHANNEL, success, fail,
                    avg_latency_us, plos_cnt, arc_cnt);
            uart_print(line);
        }
        _delay_ms(1000); /* 한 사이클 끝나면 잠깐 쉬고 채널 스윕 반복 */
    }
}

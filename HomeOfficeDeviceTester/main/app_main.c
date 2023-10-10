/**
 * @file app_main.c
 * @author Klaus Becker (bekerklaus@edu.unisinos.br)
 * @brief Main file for tester the Home Office Device
 * @version 1.0
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5

#define SENDER_HOST VSPI_HOST

static struct gs_ina219Measuring
{
    float voltage;
    float current;
    float power;
} gs_ina219Measuring;

static bool gs_relayState;

void app_main(void)
{
    spi_device_handle_t handle;

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 5000000,
        .duty_cycle_pos = 128,
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .cs_ena_posttrans = 3, 
        .queue_size = 3};

    int n = 1;
    char sendbuf[128] = {0};
    char recvbuf[128] = {0};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SENDER_HOST, &devcfg, &handle);

    while (1)
    {
        t.length = sizeof(sendbuf) * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        
        memset(sendbuf, 0, sizeof(sendbuf));
        
        if (++n >= 6){
            n = 1;
        }

        if (n == 5){
            sendbuf[1] = !gs_relayState;
        }
        
        sendbuf[0] = n;

        spi_device_transmit(handle, &t);
        printf("Received: %d\n", recvbuf[0]);
        switch (recvbuf[0])
        {
        case 1:
            memcpy(&gs_ina219Measuring.voltage, &recvbuf[1], sizeof(float));
            break;
        case 2:
            memcpy(&gs_ina219Measuring.current, &recvbuf[1], sizeof(float));
            break;
        case 3:
            memcpy(&gs_ina219Measuring.power, &recvbuf[1], sizeof(float));
            break;
        case 4:
            gs_relayState = recvbuf[1];
            break;
        case 5:
            gs_relayState = recvbuf[1];
            break;
        default:
            printf("Invalid command received\n");
        }

        printf("V: %05.2f V | I: %05.2f mA | P: %05.2f mW | R: %s\n",
            gs_ina219Measuring.voltage,
            gs_ina219Measuring.current * 1000,
            gs_ina219Measuring.power * 1000,
            gs_relayState ? "ON" : "OFF");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    spi_bus_remove_device(handle);
}

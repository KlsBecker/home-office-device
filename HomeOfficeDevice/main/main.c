/**
 * @file main.c
 * @author Klaus Becker (bekerklaus@edu.unisinos.br)
 * @brief Main file for the TGA-KLS project
 * @version 1.0
 * 
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ina219.h>
#include <string.h>
#include <esp_log.h>
#include <assert.h>
#include <driver/spi_common.h>
#include <pcd8544.h>
#include <driver/spi_slave.h>
#include <freertos/semphr.h>


/**
 * @brief INA219 configuration
*/
#define INA219_I2C_MASTER_SDA 21
#define INA219_I2C_MASTER_SCL 22
#define INA219_MAX_CURRENT 3.2f
#define INA219_SHUNT_RESISTOR 0.1f


/**
 * @brief SPI slave configuration
*/
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCLK 18
#define SPI_CS 5
#define RCV_HOST VSPI_HOST

/**
 * @brief Relay configuration
 */
#define RELAY_GPIO 2
#define BUTTON_GPIO 32


#define CMD_READ_VOLTAGE 0x01       /* SPI Read Voltage command */
#define CMD_READ_CURRENT 0x02       /* SPI Read Current command */
#define CMD_READ_POWER 0x03         /* SPI Read Power command */
#define CMD_READ_RELAY 0x04         /* SPI Read Relay command */
#define CMD_READ_ALL 0x05           /* SPI Read All command */
#define CMD_SET_RELAY_ON 0x06       /* SPI Set Relay On command */
#define CMD_SET_RELAY_OFF 0x07      /* SPI Set Relay Off command */

const static char *TAG = "TGA-KLS";

/**
 * @brief INA219 measuring variables
*/
static struct gs_ina219Measuring
{
    float voltage;
    float current;
    float power;
}gs_ina219Measuring;

/**
 * @brief Relay state variable
 */
static bool gs_relayState;

/* Homeoffice device struct data */
typedef struct homeoffice_data{
    float voltage;
    float current;
    float power;
    uint8_t relay;
}__attribute__((__packed__)) homeoffice_data;

/**
 * INA219 sensor task
*/
void ina219(void *pvParameters)
{
    ina219_t dev = {0};

    ESP_ERROR_CHECK(i2cdev_init());

    ESP_ERROR_CHECK(ina219_init_desc(&dev, INA219_ADDR_GND_GND, I2C_NUM_0, INA219_I2C_MASTER_SDA, INA219_I2C_MASTER_SCL));
    ESP_LOGI(TAG, "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&dev));

    ESP_LOGI(TAG, "Configuring INA219");
    ESP_ERROR_CHECK(ina219_configure(&dev, INA219_BUS_RANGE_32V, INA219_GAIN_0_125,
                                     INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS));

    ESP_LOGI(TAG, "Calibrating INA219");
    ESP_ERROR_CHECK(ina219_calibrate(&dev, INA219_MAX_CURRENT, INA219_SHUNT_RESISTOR));

    ESP_LOGI(TAG, "Starting INA219 loop");
    while (1)
    {
        ESP_ERROR_CHECK(ina219_get_bus_voltage(&dev, &gs_ina219Measuring.voltage));
        ESP_ERROR_CHECK(ina219_get_current(&dev, &gs_ina219Measuring.current));
        ESP_ERROR_CHECK(ina219_get_power(&dev, &gs_ina219Measuring.power));

        // printf("V: %05.2f V | I: %05.2f mA | P: %05.2f mW \n ",
        //        gs_ina219Measuring.voltage,
        //        gs_ina219Measuring.current * 1000,
        //        gs_ina219Measuring.power * 1000);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief PCD8544 display task
*/
void pcd8544(void *pvParameters)
{
    pcd8544_config_t config = {
        .spi_host = HSPI_HOST,
        .is_backlight_common_anode = true,
    };

    ESP_LOGI(TAG, "Initializing PCD8544");
    pcd8544_init(&config);

    ESP_LOGI(TAG, "Starting PCD8544 loop");
    while (1)
    {
        pcd8544_clear_display();
        pcd8544_finalize_frame_buf();
        pcd8544_set_pos(0, 0);
        pcd8544_printf("V: %05.2f V", gs_ina219Measuring.voltage);
        pcd8544_set_pos(0, 1);
        pcd8544_printf("I: %05.2f mA", gs_ina219Measuring.current * 1000);
        pcd8544_set_pos(0, 2);
        pcd8544_printf("P: %05.2f mW", gs_ina219Measuring.power * 1000);
        pcd8544_sync_and_gc();
        pcd8544_set_pos(0, 3);
        pcd8544_printf("R: %s", gs_relayState ? "ON" : "OFF");
        pcd8544_sync_and_gc();

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Get the SPI command string
 * 
 * @param cmd SPI Command
 * @return char* 
 */
static char *get_cmd_str(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_READ_VOLTAGE:
        return "READ VOLTAGE";
    case CMD_READ_CURRENT:
        return "READ CURRENT";
    case CMD_READ_POWER:
        return "READ POWER";
    case CMD_READ_RELAY:
        return "READ RELAY";
    case CMD_READ_ALL:
        return "READ ALL";
    case CMD_SET_RELAY_ON:
        return "SET RELAY ON";
    case CMD_SET_RELAY_OFF:
        return "SET RELAY OFF";
    default:
        return "UNKNOWN";
    }
}

/*
 * @brief SPI slave task
*/
void spiSlave(void *param)
{
    int ret,n=0,cmd;
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = SPI_CS,
        .queue_size = 1,
        .flags = 0,
        .post_setup_cb = 0,
        .post_trans_cb = 0,
    };

    gpio_set_pull_mode(SPI_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_CS, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "Initializing SPI slave");
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    WORD_ALIGNED_ATTR uint8_t sendbuf[128] = "";
    WORD_ALIGNED_ATTR uint8_t recvbuf[128] = "";

    spi_slave_transaction_t t;



    while (1) {
        memset(&t, 0, sizeof(t));
        memset(sendbuf, 0, sizeof(sendbuf));
        memset(recvbuf, 0, sizeof(recvbuf));

        t.length = 20 * 8;
        t.tx_buffer = NULL;
        t.rx_buffer = recvbuf;

        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        cmd = recvbuf[0];
        printf("CMD: %s (%d)\n", get_cmd_str(cmd), cmd);
    
        switch (cmd)
        {
        case CMD_READ_VOLTAGE:
            sendbuf[0] = CMD_READ_VOLTAGE; 
            memcpy(&sendbuf[1], &gs_ina219Measuring.voltage, sizeof(float));
            break;
        case CMD_READ_CURRENT:
            sendbuf[0] = CMD_READ_CURRENT; 
            memcpy(&sendbuf[1], &gs_ina219Measuring.current, sizeof(float));
            break;
        case CMD_READ_POWER:
            sendbuf[0] = CMD_READ_POWER; 
            memcpy(&sendbuf[1], &gs_ina219Measuring.power, sizeof(float));
            break;
        case CMD_READ_RELAY:
            sendbuf[0] = CMD_READ_RELAY; 
            sendbuf[1] = gs_relayState;
            break;
        case CMD_READ_ALL:
            sendbuf[0] = CMD_READ_ALL; 
            memcpy(&sendbuf[1], &gs_ina219Measuring.voltage, sizeof(float));
            memcpy(&sendbuf[5], &gs_ina219Measuring.current, sizeof(float));
            memcpy(&sendbuf[9], &gs_ina219Measuring.power, sizeof(float));
            sendbuf[13] = gs_relayState;
            break;
        case CMD_SET_RELAY_ON:
            sendbuf[0] = CMD_SET_RELAY_ON; 
            gs_relayState = 1;
            sendbuf[1] = gs_relayState;
            break;
        case CMD_SET_RELAY_OFF:
            sendbuf[0] = CMD_SET_RELAY_OFF; 
            gs_relayState = 0;
            sendbuf[1] = gs_relayState;
            break;
        default:
            break;
        }
        
        memset(&t, 0, sizeof(t));

        t.length = 20 * 8;
        t.trans_len = 20 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = NULL;
        
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
    }

}

/**
 * @brief Button interrupt handler
*/
static void IRAM_ATTR button_isr_handler(void* arg) 
{
    gs_relayState = !gs_relayState;
}

/**
 * @brief Relay control task
*/
static void relay(void* arg) 
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1<<RELAY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL<<BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    ESP_LOGI(TAG, "Starting Relay loop");
    while(1) {
        gpio_set_level(RELAY_GPIO, gs_relayState);
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

/**
 * @brief Main function
 * 
 */
void app_main()
{
    xTaskCreate(ina219, "ina219", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(pcd8544, "pcd8544", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(spiSlave, "spiSlave", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(relay, "relay", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

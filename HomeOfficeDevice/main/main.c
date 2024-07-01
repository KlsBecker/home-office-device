/**
 * @file main.c
 * @author Klaus Becker (bekerklaus@edu.unisinos.br)
 * @brief Main file for the TGB-KLS project
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
#include <freertos/semphr.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

/**
 * @brief INA219 configuration
*/
#define INA219_I2C_MASTER_SDA 21
#define INA219_I2C_MASTER_SCL 22
#define INA219_MAX_CURRENT 3.2f
#define INA219_SHUNT_RESISTOR 0.1f

/**
 * @brief Relay configuration
 */
#define RELAY_GPIO 2
#define BUTTON_GPIO 32

/**
 * @brief Wi-Fi configuration
 */
#define EXAMPLE_ESP_WIFI_SSID      "iPhone de Klaus"
#define EXAMPLE_ESP_WIFI_PASS      "batebola"
#define EXAMPLE_MAX_STA_CONN       4

#define CMD_READ_VOLTAGE 0x01       /* TCP Read Voltage command */
#define CMD_READ_CURRENT 0x02       /* TCP Read Current command */
#define CMD_READ_POWER 0x03         /* TCP Read Power command */
#define CMD_READ_RELAY 0x04         /* TCP Read Relay command */
#define CMD_READ_ALL 0x05           /* TCP Read All command */
#define CMD_SET_RELAY_ON 0x06       /* TCP Set Relay On command */
#define CMD_SET_RELAY_OFF 0x07      /* TCP Set Relay Off command */
#define CMD_UNKNOWN 0xFF            /* TCP Unknown command */

const static char *TAG = "TGB-KLS";

/**
 * @brief INA219 measuring variables
*/
static struct gs_ina219Measuring
{
    float voltage;
    float current;
    float power;
} gs_ina219Measuring;

/**
 * @brief Relay state variable
 */
static bool gs_relayState;

/* Homeoffice device struct data */
typedef struct homeoffice_data
{
    float voltage;
    float current;
    float power;
    uint8_t relay;
} __attribute__((__packed__)) homeoffice_data;

/**
 * @brief Wi-Fi event handler
 */
static bool wifi_connected = false;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
        wifi_connected = false;
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->ip_info.ip));
        wifi_connected = true;
    }
}

void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

    // Espere até que a conexão Wi-Fi seja estabelecida
    while (!wifi_connected)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Connected to AP successfully");
}

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
 * @brief Get the TCP command string
 * 
 * @param cmd TCP Command
 * @return char* 
 */
static char *get_cmd_str(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_READ_POWER:
        return "READ POWER";
    case CMD_READ_CURRENT:
        return "READ CURRENT";
    case CMD_READ_VOLTAGE:
        return "READ VOLTAGE";
    case CMD_READ_RELAY:
        return "READ RELAY";
    case CMD_READ_ALL:
        return "READ ALL";
    case CMD_SET_RELAY_ON:
        return "SET RELAY ON";
    case CMD_SET_RELAY_OFF:
        return "SET RELAY OFF";
    case CMD_UNKNOWN:
        return "UNKNOWN CMD ERROR";
    default:
        return "";
    }
}

/*
 * @brief TCP server task
*/
void tcp_server(void *pvParameters)
{
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in destAddr;

    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(3333);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", 3333);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        while (1) {
            int len;
            char rx_buffer[128];
            len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            } else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            } else {
                rx_buffer[len] = 0;
                uint8_t cmd = rx_buffer[0];
                char tx_buffer[128] = {0};

                switch (cmd) {
                case CMD_READ_VOLTAGE:
                    tx_buffer[0] = CMD_READ_VOLTAGE;
                    memcpy(&tx_buffer[1], &gs_ina219Measuring.voltage, sizeof(float));
                    break;
                case CMD_READ_CURRENT:
                    tx_buffer[0] = CMD_READ_CURRENT;
                    memcpy(&tx_buffer[1], &gs_ina219Measuring.current, sizeof(float));
                    break;
                case CMD_READ_POWER:
                    tx_buffer[0] = CMD_READ_POWER;
                    memcpy(&tx_buffer[1], &gs_ina219Measuring.power, sizeof(float));
                    break;
                case CMD_READ_RELAY:
                    tx_buffer[0] = CMD_READ_RELAY;
                    tx_buffer[1] = gs_relayState;
                    break;
                case CMD_READ_ALL:
                    tx_buffer[0] = CMD_READ_ALL;
                    memcpy(&tx_buffer[1], &gs_ina219Measuring.voltage, sizeof(float));
                    memcpy(&tx_buffer[5], &gs_ina219Measuring.current, sizeof(float));
                    memcpy(&tx_buffer[9], &gs_ina219Measuring.power, sizeof(float));
                    tx_buffer[13] = gs_relayState;
                    break;
                case CMD_SET_RELAY_ON:
                    tx_buffer[0] = CMD_SET_RELAY_ON;
                    gs_relayState = 1;
                    tx_buffer[1] = gs_relayState;
                    break;
                case CMD_SET_RELAY_OFF:
                    tx_buffer[0] = CMD_SET_RELAY_OFF;
                    gs_relayState = 0;
                    tx_buffer[1] = gs_relayState;
                    break;
                default:
                    tx_buffer[0] = CMD_UNKNOWN;
                    break;
                }

                int to_write = sizeof(tx_buffer);
                while (to_write > 0) {
                    int written = send(sock, tx_buffer, to_write, 0);
                    if (written < 0) {
                        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        break;
                    }
                    to_write -= written;
                }
            }
        }

        shutdown(sock, 0);
        close(sock);
    }

    close(listen_sock);
    vTaskDelete(NULL);
}

/**
 * @brief Button interrupt handler
*/
static void IRAM_ATTR button_isr_handler(void *arg)
{
    gs_relayState = !gs_relayState;
}

/**
 * @brief Relay control task
*/
static void relay(void *arg)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1 << RELAY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};

    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    ESP_LOGI(TAG, "Starting Relay loop");
    while (1)
    {
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
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_sta();

    xTaskCreate(ina219, "ina219", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(tcp_server, "tcp_server", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(relay, "relay", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

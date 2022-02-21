#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/task_snapshot.h"

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "soc/soc.h"

#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

#define GPIO_INPUT_PIN_MASK 0xFFF000

#define CORE_0 0
#define CORE_1 1
#define EVENT_LOOP_WIFI_QUEUE_SIZE 5
#define EVENT_LOOP_TASK_NAME "Wifi Event Handler"
#define EVENT_LOOP_HANDLER_PRIORITY 2
#define NETWORK_SSID "NET_2GD7CF90"
#define NETWORK_PASSWD "B7D7CF90"
#define UDP_PORT 59005
#define WIFI_START_BIT (1<<0)
#define WIFI_CONNECTED_BIT (1<<1)
#define WIFI_IP_SET_BIT (1<<2)
#define WIFI_DISCONNECTED (1<<3)
#define WIFI_IP_LOST_BIT (1<<4)


static volatile QueueHandle_t gxButtonMailbox;
static SemaphoreHandle_t gxSerialMutex;
static int32_t giSocketFd;
static struct sockaddr_in gxSocketConfig;
static EventGroupHandle_t gxWiFiEventGroup;

volatile uint32_t giInterruptCounter = 0;

volatile esp_netif_t *gpxNetifWifi;

void printString(const char *msg)
{
    if(pdTRUE == xSemaphoreTake(gxSerialMutex, ( TickType_t ) 10 ))
    {
        Serial.println(msg);
        xSemaphoreGive(gxSerialMutex);
    }
}

void printStringPlusNumber(const char *msg, int number)
{
    char buffer[100] = {};
    char value[5] = {};
    snprintf(buffer, sizeof(buffer), msg);
    snprintf(value, sizeof(value), "%i", number);
    strcat(buffer, value);

    printString(buffer);
}

void IRAM_ATTR GpioISRHandler(void *arg)
{
    BaseType_t high_priority_task;
    uint32_t uc_pin = (uint32_t) arg;

    xQueueSendFromISR(gxButtonMailbox, (void *) &uc_pin, &high_priority_task);

    if(high_priority_task)
    {
        portYIELD_FROM_ISR();
    }
}

void GetData(void *pvParameters)
{
    uint32_t button_pressed = 0;
    char serial_msg[100];
    for(;;)
    {
        if(pdTRUE == xQueueReceive(gxButtonMailbox, (void *) &button_pressed, portMAX_DELAY))
        {
            snprintf(serial_msg, sizeof(serial_msg), "button_pressed: %i", button_pressed);
            printString(serial_msg);
            snprintf(serial_msg, sizeof(serial_msg), "contador: %i", giInterruptCounter);
            printString(serial_msg);
            button_pressed = 0;
        }

        snprintf(serial_msg, sizeof(serial_msg), "Executado SendDataWifi");
        printString(serial_msg);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

static void WifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    printString("Handler accessed");
    if(WIFI_EVENT == event_base)
    {
        switch(event_id)
        {
            case WIFI_EVENT_STA_START:
            {
                //printf("state machine step: WIFI_EVENT_STA_START");
                printString("state machine step: WIFI_EVENT_STA_START");
                ESP_ERROR_CHECK(esp_wifi_connect());
                xEventGroupSetBits(gxWiFiEventGroup, WIFI_START_BIT);
                break;
            }
            case WIFI_EVENT_STA_CONNECTED:
            {
                //printf("state machine step: WIFI_EVENT_STA_CONNECTED");
                //printf("DHCP process triggered");
                printString("state machine step: WIFI_EVENT_STA_CONNECTED");
                printString("DHCP process triggered");
                xEventGroupSetBits(gxWiFiEventGroup, WIFI_CONNECTED_BIT);
                break;
            }

            case WIFI_EVENT_STA_DISCONNECTED:
            {
                //printf("state machine step: WIFI_EVENT_STA_DISCONNECTED");
                printString("state machine step: WIFI_EVENT_STA_DISCONNECTED");
                xEventGroupClearBits(gxWiFiEventGroup, WIFI_START_BIT|WIFI_CONNECTED_BIT|WIFI_IP_SET_BIT);
                xEventGroupSetBits(gxWiFiEventGroup, WIFI_DISCONNECTED);
                break;
            }
        }
    }
    else if(IP_EVENT == event_base)
    {
        switch(event_id)
        {
            case IP_EVENT_STA_GOT_IP:
            {
                //printf("state machine step: IP_EVENT_STA_GOT_IP");
                //printf("");
                char ip_string[100];
                ip_event_got_ip_t* event_ip = (ip_event_got_ip_t*) event_data;

                gxSocketConfig.sin_addr.s_addr = event_ip->ip_info.ip.addr;

                snprintf(ip_string, sizeof(ip_string),"state machine step: IP_EVENT_STA_GOT_IP, ip address: %i.%i.%i.%i", IP2STR(event_ip->ip_info.ip));
                printString(ip_string);
                xEventGroupSetBits(gxWiFiEventGroup, WIFI_IP_SET_BIT);
                break;
            }
            case IP_EVENT_STA_LOST_IP:
            {
                //printf("state machine step: IP_EVENT_STA_LOST_IP");
                printString("state machine step: IP_EVENT_STA_LOST_IP");
                xEventGroupClearBits(gxWiFiEventGroup, WIFI_IP_SET_BIT);
                xEventGroupSetBits(gxWiFiEventGroup, WIFI_IP_LOST_BIT);
                break;
            }
        }
    }
}

bool vSocketInitialization()
{
    bool retVal = false;
    gxSocketConfig.sin_len = sizeof(struct sockaddr_in);
    gxSocketConfig.sin_family = AF_INET;
    gxSocketConfig.sin_port = htons(UDP_PORT);

    if(-1 == (giSocketFd = socket(AF_INET, SOCK_DGRAM, 0)))
    {
        printString("Socket Initialization Error");
    }
    else if(-1 == bind(giSocketFd, (struct sockaddr *) &gxSocketConfig, gxSocketConfig.sin_len))
    {
        printString("Socket binding error");
    }
    else
        retVal = true;

    return retVal;
}

/*

void PrintStackContent(StackType_t *pStackBeginning, StackType_t *pStackEnding)
{
    if(pStackBeginning > pStackEnding)
    {
        printString("Stack decreases");
        for(;pStackBeginning > pStackEnding; pStackBeginning--)
        {
            printStringPlusNumber("",(uint32_t) (*pStackBeginning));
        }
    }
    else
    {
        printString("Stack increases");
        for(;pStackBeginning < pStackEnding; pStackBeginning++)
        {
            printStringPlusNumber("",(uint32_t) (*pStackBeginning));
        }
    }
    esp_deep_sleep_start
    esp_sleep_enable_timer_wakeup
}


void vSocketHandler(void *pvParameters)
{
    TaskSnapshot_t pxStackInfo;
    vTaskGetSnapshot(NULL, &pxStackInfo);

    StackType_t

    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printStringPlusNumber("Remaining stack size (1): ", uxHighWaterMark);
    printStackContent(pxStackInfo.pxTopOfStack, pxStackInfo.pxEndOfStack);

    EventBits_t uxWaitBits = WIFI_START_BIT| WIFI_CONNECTED_BIT | WIFI_IP_SET_BIT;
    printString("Initializing socket");
    xEventGroupWaitBits(gxWiFiEventGroup, uxWaitBits, pdTRUE, pdTRUE, portMAX_DELAY);

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printStringPlusNumber("Remaining stack size (2): ", uxHighWaterMark);
    printStackContent(pxStackInfo.pxTopOfStack, pxStackInfo.pxEndOfStack);
    vSocketInitialization();
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printStringPlusNumber("Remaining stack size (3): ", uxHighWaterMark);

    for(;;)
    {
        printString("Socket running");
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printStringPlusNumber("Remaining stack size (4): ", uxHighWaterMark);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
*/

void vSocketHandler(void *pvParameters)
{
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printStringPlusNumber("Remaining stack size (1): ", uxHighWaterMark);

    EventBits_t uxWaitBits = WIFI_START_BIT| WIFI_CONNECTED_BIT | WIFI_IP_SET_BIT;
    printString("Initializing socket");
    xEventGroupWaitBits(gxWiFiEventGroup, uxWaitBits, pdTRUE, pdTRUE, portMAX_DELAY);

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printStringPlusNumber("Remaining stack size (2): ", uxHighWaterMark);
    if(vSocketInitialization())
    {
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printStringPlusNumber("Remaining stack size (3): ", uxHighWaterMark);

        for(;;)
        {
            printString("Socket running");
            uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
            printStringPlusNumber("Remaining stack size (4): ", uxHighWaterMark);
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    }
}

/*
    Initialization order defined by the esp idf documentation.
    Link: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-station-general-scenario
    Subtitle: ESP32 Wi-Fi Station General Scenario
*/
void vWifiInitialization()
{
    xEventGroupClearBits(gxWiFiEventGroup, WIFI_START_BIT| WIFI_CONNECTED_BIT | WIFI_IP_SET_BIT | WIFI_DISCONNECTED | WIFI_IP_LOST_BIT);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpxNetifWifi = esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_configuration = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_configuration);

    esp_event_handler_instance_t wifi_event_handler;
    esp_event_handler_instance_t ip_event_handler;


    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, WifiEventHandler, NULL, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, WifiEventHandler, NULL, &ip_event_handler));

    wifi_config_t wifi_configuration;

    snprintf((char*) wifi_configuration.sta.ssid, sizeof(wifi_configuration.sta.ssid), NETWORK_SSID);
    snprintf((char*) wifi_configuration.sta.password, sizeof(wifi_configuration.sta.password), NETWORK_PASSWD);
    wifi_configuration.sta.scan_method = WIFI_FAST_SCAN;
    wifi_configuration.sta.bssid_set = false;
    wifi_configuration.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_configuration.sta.channel = 0;
    wifi_configuration.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_configuration.sta.pmf_cfg.capable = true;
    wifi_configuration.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration));

    printString("starting Wifi");
    ESP_ERROR_CHECK(esp_wifi_start());
}

void setup()
{
    Serial.begin(9600);
    gxSerialMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(gxSerialMutex);

    gxWiFiEventGroup = xEventGroupCreate();


    vWifiInitialization();

    //GPIO settings and interruption
    gpio_config_t gpio_configuration = {GPIO_INPUT_PIN_MASK, GPIO_MODE_INPUT, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_NEGEDGE};
    esp_err_t config_error = gpio_config(&gpio_configuration);
    if(ESP_OK != config_error)
    {
        char error_log[50];
        snprintf(error_log, sizeof(error_log), "error code: %i", config_error);
        printString(error_log);
    }

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1|ESP_INTR_FLAG_EDGE);

    for(uint8_t pin_number = GPIO_NUM_12; pin_number <= GPIO_NUM_23; pin_number++)
    {
        gpio_isr_handler_add((gpio_num_t)pin_number, GpioISRHandler, (void *) pin_number);
    }

    gxButtonMailbox = xQueueCreate(1, sizeof(uint32_t));

    xTaskCreatePinnedToCore(GetData, "GetData", 5*configMINIMAL_STACK_SIZE, NULL, 1, NULL, CORE_1);
    xTaskCreatePinnedToCore(vSocketHandler, "SocketHandler", 3*configMINIMAL_STACK_SIZE, NULL, 1, NULL, CORE_0);
}

void loop()
{
    vTaskDelay(1000/portTICK_PERIOD_MS);
}

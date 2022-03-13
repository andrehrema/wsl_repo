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

#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "soc/soc.h"
#include "soc/rtc.h"

#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "lwip/sockets.h"

#include "esp32/ulp.h"
#include "ulp_main.h"
#include "cJSON.h"

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
#define SNTP_CONNECTED_STATUS (1 << 8)
#define SNTP_RESET_STATUS (1<<9)
#define SNTP_IN_PROGRESS_STATUS (1<<10)

#define MAX_SNTP_CONNECTION_RETRIES 3

#define PERIOD_0 0
//period in us which ulp processor will be waken up
#define ULP_WAKEUP_PERIOD 75000

static volatile QueueHandle_t gxButtonMailbox;
static SemaphoreHandle_t gxSerialMutex;
static int32_t giSocketFd;
static struct sockaddr_in gxSocketConfig;
static EventGroupHandle_t gxNetworkEventGroup;

static const EventBits_t g_uxWifiWaitBits = (WIFI_START_BIT| WIFI_CONNECTED_BIT | WIFI_IP_SET_BIT);
static const EventBits_t g_uxSNTPWaitBits = (SNTP_CONNECTED_STATUS | SNTP_RESET_STATUS | SNTP_IN_PROGRESS_STATUS);

volatile uint32_t giInterruptCounter = 0;
volatile esp_netif_t *gpxNetifWifi;

extern const uint8_t bin_start[] asm("_binary_ulp_ulpcode_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_ulpcode_bin_end");

typedef enum
{
    UNKNOWN,
    RETRY,
    CONNECTED,
    CANCELLED
}SNTP_TRIES;

typedef enum
{
    DISCONNECTED,
    CONNECTED,
    SOCKET_ESTABLISHED,
    DATA_SENT,
    ANSWER_RECEIVED,
    COMMUNICATION_SUCCESSFUL
}COMMUNICATION_STATUS;

void printString(const char *msg)
{
    if(pdTRUE == xSemaphoreTake(gxSerialMutex, ( TickType_t ) 10 ))
    {
        printf(msg);
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


bool vInitSocket()
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

/*-------------------------------------------- SNTP functions ---------------------------------------------*/
void SNTPCallbackFunction(struct timeval *tv)
{
    sntp_sync_status_t sntp_status = sntp_get_sync_status();

    switch(sntp_status)
    {
        case SNTP_SYNC_STATUS_RESET:
        {
            xEventGroupSetBits(gxNetworkEventGroup, SNTP_RESET_STATUS);
            break;
        }

        case SNTP_SYNC_STATUS_COMPLETED:
        {
            xEventGroupSetBits(gxNetworkEventGroup, SNTP_CONNECTED_STATUS);
            break;
        }

        case SNTP_SYNC_STATUS_IN_PROGRESS:
        {
            xEventGroupSetBits(gxNetworkEventGroup, SNTP_IN_PROGRESS_STATUS);
            break;
        }
    }
}

void SynchronizeSNTP()
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "a.st1.ntp.br");
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);

    sntp_set_time_sync_notification_cb(SNTPCallbackFunction);
    sntp_init();
}

SNTP_TRIES SNTP_Tries()
{
    static uint8_t connection_try = 0;
    SNTP_TRIES retval = RETRY;

    EventBits_t sntp_status = xEventGroupGetBits(gxNetworkEventGroup);

    if(SNTP_SYNC_STATUS_COMPLETED & sntp_status)
    {
        connection_try = 0;
        retval = CONNECTED;
        printf("Sucessfully Connected to SNTP server");
        sntp_stop();
    }
    else if(connection_try < MAX_SNTP_CONNECTION_RETRIES)
    {
        sntp_restart();
        connection_try++;
    }
    else
    {
        retval = CANCELLED;
        connection_try = 0;
        printf("Problem during connection to SNTP server");
        sntp_stop();
    }
    xEventGroupClearBits(gxNetworkEventGroup, g_uxSNTPWaitBits);

    return retval;
}
/*------------------------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------- Wifi Initialization ------------------------------------------------

    Initialization order defined by the esp idf documentation.
    Link: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-station-general-scenario
    Subtitle: ESP32 Wi-Fi Station General Scenario
*/

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
                xEventGroupSetBits(gxNetworkEventGroup, WIFI_START_BIT);
                break;
            }
            case WIFI_EVENT_STA_CONNECTED:
            {
                //printf("state machine step: WIFI_EVENT_STA_CONNECTED");
                //printf("DHCP process triggered");
                printString("state machine step: WIFI_EVENT_STA_CONNECTED");
                printString("DHCP process triggered");
                xEventGroupSetBits(gxNetworkEventGroup, WIFI_CONNECTED_BIT);
                break;
            }

            case WIFI_EVENT_STA_DISCONNECTED:
            {
                //printf("state machine step: WIFI_EVENT_STA_DISCONNECTED");
                printString("state machine step: WIFI_EVENT_STA_DISCONNECTED");
                xEventGroupClearBits(gxNetworkEventGroup, WIFI_START_BIT|WIFI_CONNECTED_BIT|WIFI_IP_SET_BIT);
                xEventGroupSetBits(gxNetworkEventGroup, WIFI_DISCONNECTED);
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

                snprintf(ip_string, sizeof(ip_string),"state machine step: IP_EVENT_STA_GOT_IP, ip address: %i.%i.%i.%i", IP2STR(&(event_ip->ip_info.ip)));
                printString(ip_string);
                xEventGroupSetBits(gxNetworkEventGroup, WIFI_IP_SET_BIT);
                break;
            }
            case IP_EVENT_STA_LOST_IP:
            {
                //printf("state machine step: IP_EVENT_STA_LOST_IP");
                printString("state machine step: IP_EVENT_STA_LOST_IP");
                xEventGroupClearBits(gxNetworkEventGroup, WIFI_IP_SET_BIT);
                xEventGroupSetBits(gxNetworkEventGroup, WIFI_IP_LOST_BIT);
                break;
            }
        }
    }
}

bool CheckWifiConnection()
{
    bool retval = false;
    xEventGroupWaitBits(gxNetworkEventGroup, g_uxWifiWaitBits, pdTRUE, pdTRUE, 10/portTICK_PERIOD_MS);
    if(g_uxWifiWaitBits == xEventGroupGetBits(gxNetworkEventGroup))
    {
        retval = true;
    }
    return retval;
}

void vInitWiFi(void *pvParameters)
{
    for(;;)
    {
        if(DISCONNECTED == connection_status)
        {
            xEventGroupClearBits(gxNetworkEventGroup, WIFI_START_BIT| WIFI_CONNECTED_BIT | WIFI_IP_SET_BIT | WIFI_DISCONNECTED | WIFI_IP_LOST_BIT);

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

            if(CheckWifiConnection())
            {
                connection_status = CONNECTED;
                vTaskDelay(50/portTICK_PERIOD_MS);
            }
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------------------------*/

void vInitGPIO()
{
    //GPIO settings
    gpio_config_t gpio_configuration = {GPIO_INPUT_PIN_MASK, GPIO_MODE_INPUT, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&gpio_configuration));
    gxButtonMailbox = xQueueCreate(1, sizeof(uint32_t));
}

/*--------------------------------------------ULP Functions-----------------------------------------------------------*/
void CalibrateRTCFastClk(void)
{
    uint32_t rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 100);
    uint32_t rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;

}

void DisableConnections()
{
    communication_status = DISCONNECTED;
    esp_wifi_stop();
}

void EnterDeepSleep()
{
    DisableConnections();
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    esp_deep_sleep_start();
}

void StartULP()
{
    ESP_ERROR_CHECK(ulp_set_wakeup_period(PERIOD_0, ULP_WAKEUP_PERIOD));
    ESP_ERROR_CHECK(ulp_load_binary(0, bin_start, (bin_end - bin_start)/sizeof(uint32_t)));
    ESP_ERROR_CHECK(ulp_run(&ulp_entry - RTC_SLOW_MEM));
}

void InitializeULP(void)
{
    CalibrateRTCFastClk();
    StartULP();
    EnterDeepSleep();
}
/*------------------------------------------------------------------------------------------------------------------*/

bool HandleHostAnswer()
{
    char buffer[50] = {};
    int correctly_received = 0;
    recvfrom(giSocketFd, buffer, sizeof(buffer), 0, (struct sockaddr *) &giSocketFd, &size_socket);
    cJSON *json_answer = cJSON_Parse(buffer);
    correctly_received = cJSON_GetObjectItem(json_answer,"HostAnswer")->valueint;
    cJson_Delete(json_answer);

    return correctly_received;
}

void vReceiveData(COMMUNICATION_STATUS *communication_status)
{
    //Waiting for answer from peer
    for(uint8_t answer_periods = 0; answer_periods < MAX_SOCKET_TRY && communication_status; answer_periods++)
    {
        struct pollfd sock_poll_fd;
        sock_poll_fd.fd = giSocketFd;
        sock_poll_fd.events = POLLIN;

        if(poll(&sock_poll_fd, 1, 10) > 0)
        {
            char buffer[30];
            uint32_t size_socket = sizeof(struct sockaddr_in);

            if(1 == TreatHostAnswer())
            {
                *communication_status = COMMUNICATION_SUCCESSFUL;
                break;
            }
        }
    }
}

void vSendData(void *pvParameters)
{
    COMMUNICATION_STATUS communication_status = DISCONNECTED;
    for(;;)
    {
        vPrepareData();
        for(uint8_t socket_try = 0; socket_try < MAX_SOCKET_TRY; socket_try++)
        {
            if(CONNECTED == communication_status)
            {
                if(vInitSocket())
                {
                    communication_status = SOCKET_ESTABLISHED;
                }
            }
            else if(SOCKET_ESTABLISHED == communication_status)
            {
                sendto(giSocketFd, (void *)buffer, sizeof(buffer), 0, (struct sockaddr *) &giSocketFd, sizeof(struct sockaddr_in));
                communication_status = DATA_SENT;
            }
            else if(DATA_SENT == communication_status)
            {
                vReceiveData(communication_status);
            }
            else
            {

            }

            if(COMMUNICATION_SUCCESSFUL == communication_status)
            {
                printf("Data exchange successful");
                break;
            }

            //wait 10ms to retry socket establishment
            vTaskDelay(10/portTICK_PERIOD_MS);
        }


        if(COMMUNICATION_SUCCESSFUL != communication_status)
        {
            printf("Error during communication with host");
        }

        InitializeULP();
    }
}

void app_main(void)
{
    gxSerialMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(gxSerialMutex);

    gxNetworkEventGroup = xEventGroupCreate();

    vInitGPIO();

    vInitWiFi();
    xEventGroupWaitBits(gxNetworkEventGroup, g_uxWifiWaitBits, pdTRUE, pdTRUE, portMAX_DELAY);

    SynchronizeSNTP();
    SNTP_TRIES sntp_connection_status = UNKNOWN;
    do
    {
        xEventGroupWaitBits(gxNetworkEventGroup, g_uxWifiWaitBits, pdFALSE, pdFALSE, 3000/portTICK_PERIOD_MS);
        sntp_stop();
        sntp_connection_status = SNTP_Tries();
        /* code */
    } while (CONNECTED != sntp_connection_status || CANCELLED != sntp_connection_status);

    ulp_gpio_values = 0;
    ulp_read_counter = 0;
    InitializeULP();

    xTaskCreatePinnedToCore(vInitWiFi, "vInitWifi", 3*configMINIMAL_STACK_SIZE, NULL, 2, NULL, CORE0);
    xTaskCreatePinnedToCore(vSendData, "SendData", 5*configMINIMAL_STACK_SIZE, NULL, 2, NULL, CORE1);

    while(1)
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
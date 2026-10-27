#include "Remote-Air.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "Ir-Lbs.h"

static const char *TAG = "IrRemoteAir";

void IrRemoteAir::begin(gpio_num_t rx_gpio_num)
{
    // The air is free until an edge says otherwise.
    _last_edge_us = static_cast<uint32_t>(esp_timer_get_time()) - IR_LBS_QUIET_US;

    // The Arduino core installs the same service from attachInterrupt() and accepts ESP_ERR_INVALID_STATE.
    const esp_err_t service = gpio_install_isr_service(0);
    if (service != ESP_OK && service != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "GPIO ISR service not installed: %s; sends will not listen first", esp_err_to_name(service));
        return;
    }
    esp_err_t err = gpio_set_intr_type(rx_gpio_num, GPIO_INTR_ANYEDGE);
    if (err == ESP_OK)
        err = gpio_isr_handler_add(rx_gpio_num, _onEdge, this);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "edge interrupt on GPIO %d failed: %s; sends will not listen first", (int)rx_gpio_num, esp_err_to_name(err));
}

void IRAM_ATTR IrRemoteAir::_onEdge(void *arg)
{
    static_cast<IrRemoteAir *>(arg)->_last_edge_us = static_cast<uint32_t>(esp_timer_get_time());
}

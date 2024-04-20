/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <nvs_flash.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <esp_matter.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

uint16_t door_sensor_endpoint_id = 0;
static esp_timer_handle_t s_reporting_timer;
bool s_is_open = false;

esp_event_loop_handle_t loop_handle;
ESP_EVENT_DECLARE_BASE(TEEL_DOOR_EVENTS);
ESP_EVENT_DEFINE_BASE(TEEL_DOOR_EVENTS);
enum {
    TEEL_DOOR_EVENT
};


static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        esp_timer_start_periodic(s_reporting_timer, (uint64_t)30000000ULL);
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        /* Driver update */
    }

    return err;
}

// 1. Define the event handler
static void event_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    s_reporting_timer_callback(event_data);
}

static void s_reporting_timer_callback(void* arg) {
    // Enter deep sleep
    ESP_LOGI(TAG, "Setting sensor value");
    uint16_t endpoint_id = door_sensor_endpoint_id;
    uint32_t cluster_id = BooleanState::Id;
    uint32_t attribute_id = BooleanState::Attributes::StateValue::Id;
    node_t *node = node::get();
    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
    cluster_t *cluster = cluster::get(endpoint, cluster_id);
    attribute_t *attribute = attribute::get(cluster, attribute_id);
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.b = !val.val.b;
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

uint64_t time_door_event;
bool inHandleButton = false;
uint64_t lastIRSTime = 0;
void AppTask::gpio_isr_contact_sensor_handler(void* arg) {
    if((esp_timer_get_time() - lastIRSTime) > 1000000) {
        inHandleButton = false;
    }
    lastIRSTime = esp_timer_get_time();
    if(inHandleButton) {
        return;
    }
    inHandleButton = true;

    esp_event_post_to(loop_handle, TEEL_DOOR_EVENTS, TEEL_DOOR_EVENT, ...);
}

esp_err_t setup_door_sensor() {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL<<CONTACT_SENSOR_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)CONTACT_SENSOR_PIN, 
        gpio_isr_contact_sensor_handler, (void*) CONTACT_SENSOR_PIN);
}

extern "C" void app_main()
{

    esp_err_t err = ESP_OK;

    const esp_timer_create_args_t s_reporting_timer_args = {
            .callback = &s_reporting_timer_callback,
            .name = "reporting-timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&s_reporting_timer_args, &s_reporting_timer));


    esp_event_loop_args_t loop_task_args = {
        .queue_size = 5,
        .task_name = NULL // no task will be created
    };
    
    esp_event_loop_create(&loop_args, &loop_handle);
    esp_event_handler_register_with(loop_handle, TEEL_DOOR_EVENTS, TEEL_DOOR_EVENT, event_handler, NULL);


    /* Initialize the ESP NVS layer */
    nvs_flash_init();

#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    err = esp_pm_configure(&pm_config);
#endif
    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    //endpoint::on_off_light::config_t endpoint_config;
    //endpoint_t *app_endpoint = endpoint::on_off_light::create(node, &endpoint_config, ENDPOINT_FLAG_NONE, NULL);
    //ABORT_APP_ON_FAILURE(app_endpoint != nullptr, ESP_LOGE(TAG, "Failed to create on off light endpoint"));

    endpoint::contact_sensor::config_t contact_sensor_conf_t;
    contact_sensor_conf_t.boolean_state.state_value = true;
    endpoint_t *contact_sensor_endpoint = endpoint::contact_sensor::create(node, &contact_sensor_conf_t, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(contact_sensor_endpoint != nullptr, ESP_LOGE(TAG, "Failed to create contact sensor endpoint"));

    door_sensor_endpoint_id = endpoint::get_id(contact_sensor_endpoint);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));
}

#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define SPP_TAG "ESP_BL_COMM"

#define SPP_SERVER_NAME "ESP_DRONE"
#define EXAMPLE_DEVICE_NAME "ESP_THRUST"

class Bl_comm {
    char bda_str[18] = {0};
    static bool isWriting;
    static uint32_t clientHandle;
    static bool isConnecting;
    const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
    const bool esp_spp_enable_l2cap_ertm = true;

    long data_num = 0;

    static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
    static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;
    static char* bda2str(uint8_t *bda, char *str, size_t size);
    static void (*command_cb)(uint8_t* data, uint16_t len);
    
    static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

public:
    esp_err_t begin();

    char* get_bt_addr();

    bool isClientConnecting();
    void setCommandCb(void (*command_cb)(uint8_t* data, uint16_t len));
    esp_err_t sendMsg(char* msg, uint8_t len);
};
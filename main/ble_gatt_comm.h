#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SVC_INST_ID                 0
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
constexpr uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
constexpr uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
constexpr uint8_t char_prop_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
constexpr uint8_t char_prop_read_write_notify_nr = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

enum {
    IDX_SVC,
    IDX_CHAR_A, IDX_CHAR_VAL_A, IDX_CHAR_CFG_A,
    IDX_CHAR_B, IDX_CHAR_VAL_B, IDX_CHAR_CFG_B,
    IDX_CHAR_C, IDX_CHAR_VAL_C, IDX_CHAR_CFG_C,
    IDX_CHAR_D, IDX_CHAR_VAL_D, IDX_CHAR_CFG_D,
    IDX_CHAR_E, IDX_CHAR_VAL_E, IDX_CHAR_CFG_E,
    IDX_NB
};

struct prepare_type_env_t {
    uint8_t* prepare_buf;
    int prepare_len;
    uint16_t target_handle;
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t conn_id;
};

struct notify_target_t {
    uint16_t cfg_handle;
    uint16_t val_handle;
    uint8_t* value_ptr;
    size_t value_len;
    const char* label;
};

class Ble_comm {
public:
    Ble_comm(float* xhat_value_p, float* PRY_value_p, float* controlU_p, float* controlGain_p);
    esp_err_t begin();
    void setCommandCb(void (*cb)(uint8_t*, uint16_t)) { command_cb = cb; }
    void sendMsg(char* msg, uint8_t len);
    void sendTelemetry();
    bool isConnected();

private:
    static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
    static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                    esp_ble_gatts_cb_param_t* param);
    static void prepare_write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param,
                                        prepare_type_env_t* env);
    static void exec_write_event_env(esp_ble_gatts_cb_param_t* param,
                                     prepare_type_env_t* env_list[], size_t env_count);
    static void handleWrite(uint16_t handle, const uint8_t* data, size_t len);

    static constexpr uint8_t cfg_val_on[2] = {0x01, 0x00};
    static constexpr esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    static constexpr uint16_t GATTS_SERVICE_UUID                  = 0x00FF;
    static constexpr uint16_t GATTS_CHAR_UUID_Xhat_Telem         = 0xFF01;
    static constexpr uint16_t GATTS_CHAR_UUID_PRY_Telem          = 0xFF02;
    static constexpr uint16_t GATTS_CHAR_UUID_contU_TelemWrite   = 0xFF03;
    static constexpr uint16_t GATTS_CHAR_UUID_ContGain_Upd       = 0xFF04;
    static constexpr uint16_t GATTS_CHAR_UUID_Command            = 0xFF05;
    static constexpr uint16_t PRIMARY_SERVICE_UUID = 0x2800;
    static constexpr uint16_t CHARACTER_DECLARATION_UUID = 0x2803;
    static constexpr uint16_t CHARACTER_CLIENT_CONFIG_UUID = 0x2902;

    static float* xhat_value;
    static float* PRY_value;
    static float* controlU;
    static float* controlGain;
    static void (*command_cb)(uint8_t*, uint16_t);
    static gatts_profile_inst profile_tab[PROFILE_NUM];
    static esp_gatts_attr_db_t gatt_db[IDX_NB];
    static notify_target_t notify_targets[5];
    static prepare_type_env_t* env_list[5];
    static uint8_t adv_config_done;
};

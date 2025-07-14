#include <bit>
#include <array>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include "esp_log.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "ble_gatt_comm.h"
#include "esp_bt.h"
#include "esp_gatt_common_api.h"

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)
#define GATTS_TABLE_TAG      "BLE GATT"

namespace {
constexpr uint16_t MAX_GAIN_SIZE = 30;
constexpr uint16_t PROFILE_IDX = 0;
constexpr const char* TAG = "BLE";

using NotifyArray = std::array<notify_target_t, 5>;
NotifyArray notify_targets{
    notify_target_t{0, 0, nullptr, sizeof(float)*6, "A"},
    notify_target_t{0, 0, nullptr, sizeof(float)*3, "B"},
    notify_target_t{0, 0, nullptr, sizeof(float)*5, "C"},
    notify_target_t{0, 0, nullptr, sizeof(float)*MAX_GAIN_SIZE, "D"},
    notify_target_t{0, 0, nullptr, sizeof(uint8_t)*2, "E"}
};
}
notify_target_t Ble_comm::notify_targets[5];
float *Ble_comm::xhat_value, *Ble_comm::PRY_value, *Ble_comm::controlU, *Ble_comm::controlGain;
prepare_type_env_t* Ble_comm::env_list[5]{};
gatts_profile_inst Ble_comm::profile_tab[1]{
    {
        .gatts_if = ESP_GATT_IF_NONE,
        .conn_id = 0xFFFF,
    }
};
static uint8_t command_value_buf[GATTS_DEMO_CHAR_VAL_LEN_MAX] = {0};
esp_gatts_attr_db_t Ble_comm::gatt_db[IDX_NB] = {
    [IDX_SVC] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&PRIMARY_SERVICE_UUID,
        ESP_GATT_PERM_READ,
        sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID),
        (uint8_t*)&GATTS_SERVICE_UUID
    }},

    // Xhat Notify
    [IDX_CHAR_A] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_DECLARATION_UUID,
        ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
        (uint8_t*)&char_prop_read_write_notify
    }},
    [IDX_CHAR_VAL_A] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&GATTS_CHAR_UUID_Xhat_Telem,
        ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(float)*6, (uint8_t*)Ble_comm::xhat_value
    }},
    [IDX_CHAR_CFG_A] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_CLIENT_CONFIG_UUID,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)Ble_comm::cfg_val_on
    }},

    // PRY Notify
    [IDX_CHAR_B] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_DECLARATION_UUID,
        ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
        (uint8_t*)&char_prop_read_write_notify
    }},
    [IDX_CHAR_VAL_B] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&GATTS_CHAR_UUID_PRY_Telem,
        ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(float)*3, (uint8_t*)Ble_comm::PRY_value
    }},
    [IDX_CHAR_CFG_B] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_CLIENT_CONFIG_UUID,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)Ble_comm::cfg_val_on
    }},

    // Control U Write/Notify
    [IDX_CHAR_C] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_DECLARATION_UUID,
        ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
        (uint8_t*)&char_prop_read_write_notify
    }},
    [IDX_CHAR_VAL_C] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&GATTS_CHAR_UUID_contU_TelemWrite,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(float)*5, (uint8_t*)Ble_comm::controlU
    }},
    [IDX_CHAR_CFG_C] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_CLIENT_CONFIG_UUID,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)Ble_comm::cfg_val_on
    }},

    // Control Gain Write/Notify
    [IDX_CHAR_D] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_DECLARATION_UUID,
        ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
        (uint8_t*)&char_prop_read_write_notify
    }},
    [IDX_CHAR_VAL_D] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&GATTS_CHAR_UUID_ContGain_Upd,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(float)*30, (uint8_t*)Ble_comm::controlGain
    }},
    [IDX_CHAR_CFG_D] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_CLIENT_CONFIG_UUID,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)Ble_comm::cfg_val_on
    }},

    // Command Write
    [IDX_CHAR_E] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&CHARACTER_DECLARATION_UUID,
        ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
        (uint8_t*)&char_prop_read_write_notify
    }},
    [IDX_CHAR_VAL_E] = {{
        ESP_GATT_AUTO_RSP
    }, {
        ESP_UUID_LEN_16, (uint8_t*)&GATTS_CHAR_UUID_Command,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, nullptr
    }},
};

Ble_comm::Ble_comm(float* xhat, float* pry, float* u, float* gain) {
    xhat_value = xhat; controlU = u; controlGain = gain; PRY_value = pry;
    /*
    notify_targets[0].value_ptr = reinterpret_cast<uint8_t*>(xhat_value);
    notify_targets[1].value_ptr = reinterpret_cast<uint8_t*>(PRY_value);
    notify_targets[2].value_ptr = reinterpret_cast<uint8_t*>(controlU);
    notify_targets[3].value_ptr = reinterpret_cast<uint8_t*>(controlGain);
    gatt_db[IDX_CHAR_VAL_A].att_desc.value = reinterpret_cast<uint8_t*>(xhat);
    gatt_db[IDX_CHAR_VAL_B].att_desc.value = reinterpret_cast<uint8_t*>(pry);
    gatt_db[IDX_CHAR_VAL_C].att_desc.value = reinterpret_cast<uint8_t*>(u);
    gatt_db[IDX_CHAR_VAL_D].att_desc.value = reinterpret_cast<uint8_t*>(gain);
    */
   
    notify_targets[0].value_ptr = (uint8_t*)xhat_value;
    notify_targets[1].value_ptr = (uint8_t*)(PRY_value);
    notify_targets[2].value_ptr = (uint8_t*)(controlU);
    notify_targets[3].value_ptr = (uint8_t*)(controlGain);
    notify_targets[0].
    gatt_db[IDX_CHAR_VAL_A].att_desc.value = (uint8_t*)(xhat);
    gatt_db[IDX_CHAR_VAL_B].att_desc.value = (uint8_t*)(pry);
    gatt_db[IDX_CHAR_VAL_C].att_desc.value = (uint8_t*)(u);
    gatt_db[IDX_CHAR_VAL_D].att_desc.value = (uint8_t*)(gain);
}

void Ble_comm::sendMsg(char* msg, uint8_t len) {
    if (isConnected()) {
        esp_ble_gatts_send_indicate(profile_tab[PROFILE_IDX].gatts_if,
            profile_tab[PROFILE_IDX].conn_id, notify_targets[4].val_handle,
            len, reinterpret_cast<uint8_t*>(msg), false);
    }
}

void Ble_comm::sendTelemetry() {
    if(!isConnected()) return;
    for (int i : {0, 1, 2}) {
        const auto& t = notify_targets[i];
        esp_ble_gatts_send_indicate(profile_tab[PROFILE_IDX].gatts_if,
            profile_tab[PROFILE_IDX].conn_id, t.val_handle,
            t.value_len, t.value_ptr, false);
    }
}

bool Ble_comm::isConnected() {
    return profile_tab[PROFILE_IDX].gatts_if != ESP_GATT_IF_NONE &&
           profile_tab[PROFILE_IDX].conn_id != 0xFFFF;
}

void Ble_comm::prepare_write_event_env(esp_gatt_if_t gatts_if,
    esp_ble_gatts_cb_param_t* param, prepare_type_env_t* env) {

    if (param->write.offset + param->write.len > PREPARE_BUF_MAX_SIZE) return;
    if (!env->prepare_buf) {
        env->prepare_buf = new uint8_t[PREPARE_BUF_MAX_SIZE];
        env->prepare_len = 0;
        env->target_handle = param->write.handle;
    }

    if (param->write.need_rsp) {
        esp_gatt_rsp_t rsp{};
        rsp.attr_value.len = param->write.len;
        rsp.attr_value.handle = param->write.handle;
        rsp.attr_value.offset = param->write.offset;
        std::copy_n(param->write.value, param->write.len, rsp.attr_value.value);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
            param->write.trans_id, ESP_GATT_OK, &rsp);
    }

    std::copy_n(param->write.value, param->write.len,
        env->prepare_buf + param->write.offset);
    env->prepare_len += param->write.len;
}

void Ble_comm::exec_write_event_env(esp_ble_gatts_cb_param_t* param,
    prepare_type_env_t* env_list[], size_t env_count) {

    auto* env = [&] {
        for (size_t i = 0; i < env_count; ++i)
            if (env_list[i] && env_list[i]->prepare_buf &&
                env_list[i]->target_handle == param->write.handle)
                return env_list[i];
        return static_cast<prepare_type_env_t*>(nullptr);
    }();

    if (env) {
        if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
            ESP_LOG_BUFFER_HEX(TAG, env->prepare_buf, env->prepare_len);
        delete[] env->prepare_buf;
        env->prepare_buf = nullptr;
        env->prepare_len = 0;
        env->target_handle = 0;
    }
}

void Ble_comm::handleWrite(uint16_t handle, const uint8_t* data, size_t len) {
    if (handle == notify_targets[2].val_handle && len == 5) {
        float value = std::bit_cast<float>(*reinterpret_cast<const uint32_t*>(data));
        uint8_t index = data[4];
        if (index < 5) controlU[index] = value;
    } else if (handle == notify_targets[3].val_handle && len == MAX_GAIN_SIZE * sizeof(float)) {
        const auto* raw = reinterpret_cast<const uint32_t*>(data);
        for (size_t i = 0; i < MAX_GAIN_SIZE; ++i)
            controlGain[i] = std::bit_cast<float>(raw[i]);
    } else if (handle == notify_targets[4].val_handle && len >= 2) {
        command_cb(const_cast<uint8_t*>(data), len);
    }
}

void Ble_comm::gap_event_handler(esp_gap_ble_cb_event_t e, esp_ble_gap_cb_param_t* p) {
    switch (e) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE advertising data configured, starting advertising...");
            esp_ble_gap_start_advertising(const_cast<esp_ble_adv_params_t*>(&adv_params));
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (p->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
                ESP_LOGE(TAG, "BLE advertising failed to start");
            else
                ESP_LOGI(TAG, "BLE advertising started successfully");
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE advertising stopped");
            break;
        default:
            break;
    }
}

void Ble_comm::gatts_event_handler(esp_gatts_cb_event_t e, esp_gatt_if_t gatt_if,
                                   esp_ble_gatts_cb_param_t* param) {
    if (e == ESP_GATTS_REG_EVT && param->reg.status == ESP_GATT_OK) {
        profile_tab[0].gatts_if = gatt_if;
        ESP_LOGI(TAG, "GATT server registered successfully (app_id: %04x)", param->reg.app_id);
    }
    else if (e == ESP_GATTS_CREAT_ATTR_TAB_EVT) {
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            // 各Characteristic/Descriptorのハンドルをnotify_targets等にセット
            notify_targets[0].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_A];
            notify_targets[0].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_A];
            notify_targets[1].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_B];
            notify_targets[1].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_B];
            notify_targets[2].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_C];
            notify_targets[2].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_C];
            notify_targets[3].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_D];
            notify_targets[3].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_D];
            notify_targets[4].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_E];
            notify_targets[4].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_E];
            esp_ble_gatts_start_service(param->add_attr_tab.handles[IDX_SVC]);
        } else {
            ESP_LOGE(TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        return;
    }
    // 接続／切断ログの追加
    else if (e == ESP_GATTS_CONNECT_EVT) {
        ESP_LOGI(TAG, "🟢 BLE Connected: conn_id = %d", param->connect.conn_id);
        ESP_LOG_BUFFER_HEX(TAG, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        profile_tab[0].conn_id = param->connect.conn_id;
        profile_tab[0].gatts_if = gatt_if; // 接続時にGATTインターフェースを更新
    }
    else if (e == ESP_GATTS_DISCONNECT_EVT) {
        ESP_LOGI(TAG, "🔴 BLE Disconnected: reason = 0x%X", param->disconnect.reason);
        esp_ble_gap_start_advertising(const_cast<esp_ble_adv_params_t*>(&adv_params));
        profile_tab[0].gatts_if = gatt_if;
        profile_tab[0].conn_id = 0xFFFF; // 接続IDをリセット
        ESP_LOGI(TAG, "BLE advertising restarted after disconnect");
    }
     // ★ Read/Writeイベントのフック
    else if (e == ESP_GATTS_READ_EVT) {
        ESP_LOGI(TAG, "GATT READ: handle=0x%04x, conn_id=%d, offset=%d",
                 param->read.handle, param->read.conn_id, param->read.offset);
        // 必要に応じてここで値をセットしたり、レスポンスを返す
        // 例: esp_ble_gatts_send_response(gatt_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
    }
    else if (e == ESP_GATTS_WRITE_EVT) {
        ESP_LOGI(TAG, "GATT WRITE: handle=0x%04x, conn_id=%d, len=%d, need_rsp=%d",
                 param->write.handle, param->write.conn_id, param->write.len, param->write.need_rsp);
        //ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);

        // 必要に応じて書き込み値を処理
        handleWrite(param->write.handle, param->write.value, param->write.len);

        // レスポンスが必要な場合は応答
        if (param->write.need_rsp) {
            esp_gatt_rsp_t rsp{};
            rsp.attr_value.handle = param->write.handle;
            rsp.attr_value.len = param->write.len;
            rsp.attr_value.offset = param->write.offset;
            memcpy(rsp.attr_value.value, param->write.value, param->write.len);
            esp_ble_gatts_send_response(gatt_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);
        }
    }

    // 通常のイベントハンドリング
    if (gatt_if == ESP_GATT_IF_NONE || gatt_if == profile_tab[0].gatts_if)
        if (profile_tab[0].gatts_cb)
            profile_tab[0].gatts_cb(e, gatt_if, param);
}

esp_err_t Ble_comm::begin() {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(ESP_APP_ID));

    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));

    // 属性テーブルを GATT に登録
    esp_err_t ret = esp_ble_gatts_create_attr_tab(gatt_db, profile_tab[PROFILE_APP_IDX].gatts_if, IDX_NB, SVC_INST_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create attribute table: %s", esp_err_to_name(ret));
        return ret;
    }

    // デバイス名を設定（例: "MyBLEDevice"）
    const char* device_name = "ESP_DRONE";
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(device_name));

    // --- ここからadvertise処理を追加 ---
    // 広告データの設定
    esp_ble_adv_data_t adv_data = {};
    adv_data.set_scan_rsp = false;
    adv_data.include_name = true;
    adv_data.include_txpower = true;
    adv_data.min_interval = 0x20;
    adv_data.max_interval = 0x40;
    adv_data.appearance = 0x00;
    adv_data.manufacturer_len = 0;
    adv_data.p_manufacturer_data = nullptr;
    adv_data.service_data_len = 0;
    adv_data.p_service_data = nullptr;
    adv_data.service_uuid_len = 0;
    adv_data.p_service_uuid = nullptr;
    adv_data.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));    

    return ESP_OK;
}

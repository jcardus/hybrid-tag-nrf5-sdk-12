#include <stdint.h>
#include <string.h>
#include "ble_advdata.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CENTRAL_LINK_COUNT              0
#define PERIPHERAL_LINK_COUNT           1   // Need 1 for connectable

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(100, UNIT_0_625_MS)
#define APP_ADV_TIMEOUT                 0   // No timeout

#define DEAD_BEEF                       0xDEADBEEF

#define APP_TIMER_PRESCALER             0
#define APP_TIMER_OP_QUEUE_SIZE         4

// Custom service UUID: 12345678-1234-5678-1234-56789abcdef0
#define CUSTOM_SERVICE_UUID_BASE {0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
                                  0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12}
#define CUSTOM_SERVICE_UUID      0x1234
#define CUSTOM_CHAR_UUID         0x1235

// Google characteristic UUID (same service)
#define GOOGLE_CHAR_UUID         0x1236

#define APPLE_KEY_LENGTH         28
#define GOOGLE_KEY_LENGTH        20

static ble_gap_adv_params_t m_adv_params;
static uint16_t             m_conn_handle = BLE_CONN_HANDLE_INVALID;
static uint16_t             m_service_handle;
static ble_gatts_char_handles_t m_apple_char_handles;
static ble_gatts_char_handles_t m_google_char_handles;
static uint8_t              m_custom_uuid_type;

// Apple key storage - receives 28 bytes in two writes (20 + 8)
static uint8_t              apple_key[APPLE_KEY_LENGTH];
static uint8_t              m_apple_key_offset = 0;

// Google key storage - receives 20 bytes in one write operation
static uint8_t              google_key[GOOGLE_KEY_LENGTH];

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void on_write(ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // Handle Apple key writes (28 bytes in two writes: 20 + 8)
    if (p_evt_write->handle == m_apple_char_handles.value_handle)
    {
        uint16_t len = p_evt_write->len;

        // Ensure we don't overflow apple_key buffer
        if (m_apple_key_offset + len > APPLE_KEY_LENGTH)
        {
            len = APPLE_KEY_LENGTH - m_apple_key_offset;
        }

        // Copy data to apple_key at current offset
        memcpy(&apple_key[m_apple_key_offset], p_evt_write->data, len);
        m_apple_key_offset += len;

        NRF_LOG_INFO("Apple: received %d bytes (total: %d/%d)\r\n", p_evt_write->len, m_apple_key_offset, APPLE_KEY_LENGTH);

        // Check if we have received all 28 bytes
        if (m_apple_key_offset >= APPLE_KEY_LENGTH)
        {
            NRF_LOG_INFO("Apple key complete:\r\n");
            for (int i = 0; i < APPLE_KEY_LENGTH; i++)
            {
                NRF_LOG_RAW_INFO("%02X ", apple_key[i]);
            }
            NRF_LOG_RAW_INFO("\r\n");

            // Reset offset for next key
            m_apple_key_offset = 0;
        }
    }
    // Handle Google key writes (20 bytes in one write)
    else if (p_evt_write->handle == m_google_char_handles.value_handle)
    {
        uint16_t len = p_evt_write->len;
        if (len > GOOGLE_KEY_LENGTH)
        {
            len = GOOGLE_KEY_LENGTH;
        }

        memcpy(google_key, p_evt_write->data, len);

        NRF_LOG_INFO("Google key received (%d bytes):\r\n", len);
        for (int i = 0; i < len; i++)
        {
            NRF_LOG_RAW_INFO("%02X ", google_key[i]);
        }
        NRF_LOG_RAW_INFO("\r\n");
    }
}

static void ble_evt_handler(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // Restart advertising
            sd_ble_gap_adv_start(&m_adv_params);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt);
            break;

        default:
            break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_evt_handler(p_ble_evt);
}

static void services_init(void)
{
    // Add a custom UUID base
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    uint32_t err_code = sd_ble_uuid_vs_add(&base_uuid, &m_custom_uuid_type);
    APP_ERROR_CHECK(err_code);

    // Add service
    ble_uuid_t service_uuid;
    service_uuid.type = m_custom_uuid_type;
    service_uuid.uuid = CUSTOM_SERVICE_UUID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &m_service_handle);
    APP_ERROR_CHECK(err_code);

    // Common characteristic setup
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          char_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 1;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    // Add Apple characteristic (0x1235, 28 bytes)
    char_uuid.type = m_custom_uuid_type;
    char_uuid.uuid = CUSTOM_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = APPLE_KEY_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = APPLE_KEY_LENGTH;
    attr_char_value.p_value   = apple_key;

    err_code = sd_ble_gatts_characteristic_add(m_service_handle, &char_md, &attr_char_value, &m_apple_char_handles);
    APP_ERROR_CHECK(err_code);

    // Add Google characteristic (0x1236, 20 bytes)
    char_uuid.uuid = GOOGLE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = GOOGLE_KEY_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = GOOGLE_KEY_LENGTH;
    attr_char_value.p_value   = google_key;

    err_code = sd_ble_gatts_characteristic_add(m_service_handle, &char_md, &attr_char_value, &m_google_char_handles);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Service 0x%04X initialized\r\n", CUSTOM_SERVICE_UUID);
    NRF_LOG_INFO("  Apple char 0x%04X (handle 0x%04X)\r\n", CUSTOM_CHAR_UUID, m_apple_char_handles.value_handle);
    NRF_LOG_INFO("  Google char 0x%04X (handle 0x%04X)\r\n", GOOGLE_CHAR_UUID, m_google_char_handles.value_handle);
}

static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] = {{CUSTOM_SERVICE_UUID, m_custom_uuid_type}};

    // Build advertising data
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = flags;

    // Build scan response with service UUID
    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = 1;
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;  // Connectable
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT;
}

static void advertising_start(void)
{
    uint32_t err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Advertising started\r\n");
}

static void gap_params_init(void)
{
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    uint32_t err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) "hybrid-tag", 8);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS);
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS(75, UNIT_1_25_MS);
    gap_conn_params.slave_latency     = 0;
    gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void ble_stack_init(void)
{
    nrf_clock_lf_cfg_t clock_lf_cfg = {
        .source        = NRF_CLOCK_LF_SRC_RC,
        .rc_ctiv       = 16,
        .rc_temp_ctiv  = 2,
        .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM
    };

    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    uint32_t err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                             PERIPHERAL_LINK_COUNT,
                                                             &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Increase the GATTS attribute table size for our service
    ble_enable_params.gatts_enable_params.attr_tab_size = 0x500;

    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register BLE event handler
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Set a custom MAC address
    ble_gap_addr_t addr;
    addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    addr.addr[0] = 0x01;
    addr.addr[1] = 0xAC;
    addr.addr[2] = 0xBE;
    addr.addr[3] = 0xEE;
    addr.addr[4] = 0xFF;
    addr.addr[5] = 0xC0;
#if (NRF_SD_BLE_API_VERSION >= 3)
    err_code = sd_ble_gap_addr_set(&addr);
#else
    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
#endif
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(4);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
        addr.addr[5], addr.addr[4], addr.addr[3],
        addr.addr[2], addr.addr[1], addr.addr[0]);
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();

    NRF_LOG_INFO("BLE GATT Service started\r\n");

    advertising_start();

    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}

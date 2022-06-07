/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* Beacon sample code with BTSTACK v1 APIs
*
*/
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "cycfg_gatt_db.h"

extern const wiced_bt_cfg_settings_t app_cfg_settings;

/******************************************************************************
 *                             Local Function Definitions
 ******************************************************************************/
static wiced_bt_gatt_status_t   beacon_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);
static wiced_bt_gatt_status_t   beacon_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data);
static wiced_bt_gatt_status_t   beacon_indication_cfm_handler(uint16_t conn_id, uint16_t handle);

wiced_bt_gatt_status_t beacon_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
void beacon_set_app_advertisement_data();

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void beacon_set_app_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = app_gap_device_name_len;
    adv_elem[num_elem].p_data       = (uint8_t*)app_gap_device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t beacon_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    int to_copy;

#if OTA_FW_UPGRADE
    // if read request is for the OTA FW upgrade service, pass it to the library to process
    if (p_read_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }
#endif
    switch(p_read_data->handle)
    {
    case HDLC_GAP_DEVICE_NAME_VALUE:
        if (p_read_data->offset >= app_gap_device_name_len)
            return WICED_BT_GATT_INVALID_OFFSET;

        to_copy = app_gap_device_name_len - p_read_data->offset;
        if (*p_read_data->p_val_len < to_copy)
            to_copy = *p_read_data->p_val_len;

        memcpy(p_read_data->p_val, app_gap_device_name + p_read_data->offset, to_copy);
        *p_read_data->p_val_len = to_copy;
        break;

    case HDLC_GAP_APPEARANCE_VALUE:
        if (p_read_data->offset >= 2)
            return WICED_BT_GATT_INVALID_OFFSET;

        to_copy = 2 - p_read_data->offset;
        if (*p_read_data->p_val_len < to_copy)
            to_copy = *p_read_data->p_val_len;

        memcpy(p_read_data->p_val, ((uint8_t*)&app_cfg_settings.gatt_cfg.appearance) + p_read_data->offset, to_copy);
        *p_read_data->p_val_len = to_copy;
        break;

    default:
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t beacon_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
#if OTA_FW_UPGRADE
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (p_write_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_write_data);
    }
#endif

    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Process indication_confirm from peer device
 */
wiced_bt_gatt_status_t beacon_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
{
#if OTA_FW_UPGRADE
    // if indication confirmation is for the OTA FW upgrade service, pass it to the library to process
    if (handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Process GATT request from the peer
 */
wiced_bt_gatt_status_t beacon_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = beacon_gatts_req_read_handler(p_data->conn_id, &p_data->data.read_req);
        break;

    case GATTS_REQ_TYPE_WRITE:
    case GATTS_REQ_TYPE_PREP_WRITE:
        result = beacon_gatts_req_write_handler(p_data->conn_id, &p_data->data.write_req);
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATTS_REQ_TYPE_MTU:
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATTS_REQ_TYPE_CONF:
        result = beacon_indication_cfm_handler(p_data->conn_id, p_data->data.handle);
        break;

   default:
        WICED_BT_TRACE("%s: Unsupported type: %d\n", __func__, p_data->request_type);
        break;
    }
    return result;
}

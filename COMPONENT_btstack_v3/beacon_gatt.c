/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
* Beacon sample code with BTSTACK v3 APIs
*
*/
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_memory.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"

extern const wiced_bt_cfg_settings_t app_cfg_settings;

/******************************************************************************
 *                             Local Function Definitions
 ******************************************************************************/
wiced_bt_gatt_status_t beacon_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
void beacon_set_app_advertisement_data();

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 * Setup advertisement data with configured advertisment array from BTConfigurator
 * By default, includes 16 byte UUID and device name
 */
void beacon_set_app_advertisement_data(void)
{
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);
}

/*
 * Process Read request from peer device
 */
wiced_bt_gatt_status_t beacon_req_read_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_t *p_read_req,
        uint16_t len_requested)
{
    int to_copy;
    uint8_t * copy_from;

#if OTA_FW_UPGRADE
    // if read request is for the OTA FW upgrade service, pass it to the library to process
    if (p_read_req->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_req);
    }
#endif
    switch(p_read_req->handle)
    {
    case HDLC_GAP_DEVICE_NAME_VALUE:
        to_copy = app_gap_device_name_len;
        copy_from = (uint8_t *) app_gap_device_name;
        break;

    case HDLC_GAP_APPEARANCE_VALUE:
        to_copy = 2;
        copy_from = (uint8_t *) &app_cfg_settings.p_ble_cfg->appearance;
        break;

    default:
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    // Adjust copying location & length limit based on offset
    copy_from += p_read_req->offset;
    to_copy -= p_read_req->offset;

    // if copy length is not valid, then that means the offset of over the limit
    if (to_copy <= 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
            WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    // If the copying length is not specified or if it is over the size, use the max can copy
    if (len_requested == 0 || len_requested > to_copy)
    {
        len_requested = to_copy;
    }

    wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, len_requested, copy_from, NULL);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process Read by type request from peer device
 */
wiced_bt_gatt_status_t beacon_req_read_by_type_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_by_type_t *p_read_req,
        uint16_t len_requested)
{
    int         to_copy;
    uint8_t     * copy_from;
    uint16_t    attr_handle = p_read_req->s_handle;
    uint8_t    *p_rsp = wiced_bt_get_buffer(len_requested);
    uint16_t    rsp_len = 0;
    uint8_t     pair_len = 0;
    int used = 0;

    if (p_rsp == NULL)
    {
        WICED_BT_TRACE("[%s] no memory len_requested: %d!!\n", __FUNCTION__,
                len_requested);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle,
                WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        /// Add your code here
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                p_read_req->e_handle, &p_read_req->uuid);

        if (attr_handle == 0)
            break;

        switch(attr_handle)
        {
        case HDLC_GAP_DEVICE_NAME_VALUE:
            to_copy = app_gap_device_name_len;
            copy_from = (uint8_t *) app_gap_device_name;
            break;

        case HDLC_GAP_APPEARANCE_VALUE:
            to_copy = 2;
            copy_from = (uint8_t *) &app_cfg_settings.p_ble_cfg->appearance;
            break;

        default:
            WICED_BT_TRACE("[%s] found type but no attribute ??\n", __FUNCTION__);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                    p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            wiced_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(
                p_rsp + used,
                len_requested - used,
                &pair_len,
                attr_handle,
                to_copy,
                copy_from);

        if (filled == 0) {
            break;
        }
        used += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        WICED_BT_TRACE("[%s] attr not found 0x%04x -  0x%04x Type: 0x%04x\n",
                __FUNCTION__, p_read_req->s_handle, p_read_req->e_handle,
                p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                WICED_BT_GATT_INVALID_HANDLE);
        wiced_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len,
            used, p_rsp, (wiced_bt_gatt_app_context_t)wiced_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process read multi request from peer device
 */
wiced_bt_gatt_status_t beacon_req_read_multi_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_multiple_req_t *p_read_req,
        uint16_t len_requested)
{
    uint8_t     *p_rsp = wiced_bt_get_buffer(len_requested);
    int         used = 0;
    int         xx;
    uint16_t    handle;
    int         to_copy;
    uint8_t     * copy_from;

    handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);

    if (p_rsp == NULL)
    {
        WICED_BT_TRACE ("[%s] no memory len_requested: %d!!\n", __FUNCTION__,
                len_requested);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle,
                WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, xx);

        switch(handle)
        {
        case HDLC_GAP_DEVICE_NAME_VALUE:
            to_copy = app_gap_device_name_len;
            copy_from = (uint8_t *) app_gap_device_name;
            break;

        case HDLC_GAP_APPEARANCE_VALUE:
            to_copy = 2;
            copy_from = (uint8_t *) &app_cfg_settings.p_ble_cfg->appearance;
            break;

        default:
            WICED_BT_TRACE ("[%s] no handle 0x%04xn", __FUNCTION__, handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                    *p_read_req->p_handle_stream, WICED_BT_GATT_ERR_UNLIKELY);
            wiced_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }
        int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode,
                    p_rsp + used,
                    len_requested - used,
                    handle,
                    to_copy,
                    copy_from);

        if (!filled) {
            break;
        }
        used += filled;
    }

    if (used == 0)
    {
        WICED_BT_TRACE ("[%s] no attr found\n", __FUNCTION__);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE);
        wiced_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used, p_rsp,
            (wiced_bt_gatt_app_context_t)wiced_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t beacon_write_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_write_req_t* p_data)
{
    WICED_BT_TRACE("[%s] conn_id:%d handle:%04x\n", __FUNCTION__, conn_id,
            p_data->handle);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t beacon_req_mtu_handler( uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    wiced_bt_gatt_server_send_mtu_rsp(conn_id, mtu,
            app_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process indication confirm.
 */
wiced_bt_gatt_status_t beacon_req_value_conf_handler(uint16_t conn_id,
        uint16_t handle)
{
    WICED_BT_TRACE("[%s] conn_id:%d handle:%x\n", __FUNCTION__, conn_id, handle);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process GATT request from the peer
 */
wiced_bt_gatt_status_t beacon_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (p_data->opcode)
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
            result = beacon_req_read_handler(p_data->conn_id,
                    p_data->opcode,
                    &p_data->data.read_req,
                    p_data->len_requested);
            break;

        case GATT_REQ_READ_BY_TYPE:
            result = beacon_req_read_by_type_handler(p_data->conn_id,
                    p_data->opcode,
                    &p_data->data.read_by_type,
                    p_data->len_requested);
            break;

        case GATT_REQ_READ_MULTI:
        case GATT_REQ_READ_MULTI_VAR_LENGTH:
            result = beacon_req_read_multi_handler(p_data->conn_id,
                    p_data->opcode,
                    &p_data->data.read_multiple_req,
                    p_data->len_requested);
            break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
            result = beacon_write_handler(p_data->conn_id,
                    p_data->opcode,
                    &(p_data->data.write_req));
            if (result == WICED_BT_GATT_SUCCESS)
            {
                wiced_bt_gatt_server_send_write_rsp(
                        p_data->conn_id,
                        p_data->opcode,
                        p_data->data.write_req.handle);
            }
            else
            {
                wiced_bt_gatt_server_send_error_rsp(
                        p_data->conn_id,
                        p_data->opcode,
                        p_data->data.write_req.handle,
                        result);
            }
            break;

        case GATT_REQ_MTU:
            result = beacon_req_mtu_handler(p_data->conn_id,
                    p_data->data.remote_mtu);
            break;

        case GATT_HANDLE_VALUE_CONF:
            result = beacon_req_value_conf_handler(p_data->conn_id,
                    p_data->data.confirm.handle);
            break;

       default:
            WICED_BT_TRACE("Invalid GATT request conn_id:%d opcode:%d\n",
                    p_data->conn_id, p_data->opcode);
            break;
    }

    return result;
}

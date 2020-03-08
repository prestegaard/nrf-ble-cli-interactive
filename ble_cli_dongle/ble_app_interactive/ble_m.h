/**
 * Copyright (c) 2018 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @brief     Application BLE module.
 *  @details   This module contains most of the functions used
 *              by the application to manage BLE stack events
 *              and BLE connections.
 */

#ifndef BLE_M_H__
#define BLE_M_H__

#include <stdbool.h>
#include <stdlib.h>
#include "ble_gap.h"
#include "ble_gattc.h"
#include "sdk_config.h"
#include "ble_advertising.h"
#include "peer_manager.h"

#define MIN_CONNECTION_INTERVAL (uint16_t)MSEC_TO_UNITS(NRF_BLE_SCAN_MIN_CONNECTION_INTERVAL, UNIT_1_25_MS)  /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL (uint16_t)MSEC_TO_UNITS(NRF_BLE_SCAN_MAX_CONNECTION_INTERVAL, UNIT_1_25_MS)  /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY           NRF_BLE_SCAN_SLAVE_LATENCY                                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT     (uint16_t)MSEC_TO_UNITS(NRF_BLE_SCAN_SUPERVISION_TIMEOUT, UNIT_10_MS)        /**< Determines supervision time-out in units of 10 milliseconds. */

/**@brief The maximum number of peripheral and central links combined.
 */
#define NRF_BLE_LINK_COUNT      (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT)
#define ADDR_STRING_LEN         (2 * (BLE_GAP_ADDR_LEN)+6)                                           /**< Determines device BLE address length in string format. Address formatting: XX:XX:XX:XX:XX:XX. 
                                                                                                          The hex number in the string format takes twice as much space. 6 is added in place of ":" or spaces beetwen numbers and for the string terminator. */
#define HEX_BYTE_STRING_LEN     (3)                                                                  /**< Determines string length for a 1-byte hex number. */

/* 
Basic calculation of bytes used and available for custom data


flag: 2 + 1       = 3 bytes
short name: 6 + 2 = 8 bytes
tx power: 2 + 1   = 3 bytes
---------------------------
total:             14 bytes

manufacturer meta:  2 bytes
manufacturer id:    2 bytes
---------------------------
total:             18 bytes

13 bytes remaining for custom use

 */





#define APP_BEACON_INFO_LENGTH_MAX      0xd                               /**< Total length of information advertised by the Beacon. */
#define APP_SHORT_NAME_LENGTH           0x06
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_GAP_FLAGS                   BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE


/* UUID can be used as a uinque serial number  to differentiate beacons
 * Could be smaller to allow for more bytes for custom use 
 */
#define APP_BEACON_UUID_NUM_BYTES       0xd // only 13 available bytes for uuid
#define APP_BEACON_UUID_DEFAULT         0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define APP_CUSTOM_DEFAULT_VALUE1               0xCA
#define APP_CUSTOM_DEFAULT_VALUE2               0xFE
#define APP_CUSTOM_DEFAULT_VALUE3               0xBA

typedef struct
{
    bool    is_not_empty;                   /**< Indicates that the structure is not empty. */
    uint8_t addr[BLE_GAP_ADDR_LEN];         /**< Device address. */
    char    dev_name[DEVICE_NAME_MAX_SIZE]; /**< Device name. */
    uint8_t adv_data[32];                    /**< Advertisement data.  */
    uint8_t adv_data_len;                   /**< Lenght of advertisement data.  */
} scanned_device_t;

typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;

/**@brief Function for establishing a connection with a device that uses privacy.
 *
 * @param[in] p_peers_id  Pointer to a handle to uniquely identify a peer for which data has been persistently stored.
 */
void private_connect(pm_peer_id_t const * p_peers_id);


/**@brief Function for setting the address for the connection.
 *
 * @param[in]  address_str    Device address in string format.
 */
void addr_store_value_set(char const * const p_address_str);


/**@brief Function for checking scan state.
 *
 * @retval true   If scanning is in progress.
 * @retval false  Otherwise.
 */
bool is_scanning(void);


/**@brief Function for getting CCCD descriptors handle.
 *
 * @param[in] p_char_uuid_str  Characterisctic UUID in string format.
 *
 * @retval    If exist, CCCD descriptor handle, otherwise BLE_GATT_HANDLE_INVALID.
 */
uint16_t cccd_descriptors_handle_get(char const * const p_char_uuid_str);


/**@brief Function for getting a devices scan list.
 *
 * @return Pointer to an array containing information about nearby devices that are advertising.
           Array size is defined as DEVICE_TO_FIND_MAX.
 */
scanned_device_t * scan_device_info_get(void);


/**@brief Function for clearing the array of nearby advertisers.
 */
void scan_device_info_clear(void);


/**@brief Function for checking if the user needs to confirm a numerical value.
 *
 * @retval true        If the user needs to confirm a numerical value.
 * @retval false       Otherwise.
 */
bool is_numeric_match_requested(void);

/**
 * @brief Function for clearing the numeric match request flag.
 */
void numeric_match_request_clear(void);


/**@brief Function for getting the connection handle where the numeric match was requested.
 *
 * @return Connection handle.
 */
uint16_t numeric_match_request_conn_handle_get(void);


/**@brief Function for searching for the characteristic handle.
 *
 * @param[in] service_uuid_str     Primary service UUID.
 *
 * @return                         Characteristic handle.
 */
uint16_t handle_value_search(char const * const p_service_uuid_str);


/**@brief Function for searching for a service handle range.
 *
 * @param[in] service_uuid_str  Service UUID.
 * @param[in] conn_handle       The connection handle identifying the connection to perform this procedure on.
 *
 * @return                      A pointer to the range of handles of the characteristic to perform this procedure on.
 */
ble_gattc_handle_range_t * handle_range_search(char const * const p_service_uuid_str,
                                               uint16_t           conn_handle);


/**@brief Function for setting the preferred PHY.
 *
 * @param[in] p_phy             Pointer to the PHY structure.
 * @param[in] conn_handle       Connection handle identifying the connection to perform this procedure on.
 */
void preferred_phy_set(ble_gap_phys_t const * const p_phy, uint16_t conn_handle);


/**@brief Function for changing Link Layer data length for the connection.
 *
 * @param[in] data_length    New data length value.
 * @param[in] conn_handle    The connection handle identifying the connection to perform this procedure on.
 */
void link_layer_data_length_set(char * p_data_length, uint16_t conn_handle);


/**@brief Function for setting MTU for central and peripheral connections.
 *
 * @param[in] mtu_value      New MTU value.
 */
void gatt_mtu_set(char const * const p_mtu_value);


/**@brief Function for returning the current PHY mode in string format.
 *
 * @param[in] phys            PHY structure.
 *
 * @return                    PHY mode in string format.
 */
char const * phy_str(ble_gap_phys_t const * const p_phys);


/**@brief Function for converting a 6-byte int address into a hex string.
 *
 * @param[in]  addr        Device address.
 * @param[out] result      Output buffer for the device address in string.
 * @param[i]   result_len  Length of output buffer. Should be BLE_GAP_ADDR_LEN for BLE GAP address.
 */
void int_addr_to_hex_str(char * p_result, uint8_t result_len, uint8_t const * const p_addr);


/**@brief Function for finding a connection handle from a BLE address string.
 *
 * @param[in] addr               BLE device address in string format.
 *
 * @return                       The connection handle identifying the connection to perform this procedure on.
 */
uint16_t addr_string_to_conn_handle(char const * const p_addr);


/**@brief Function for comparing an int address with a string address.
 *
 * @param[in] p_connected_adr          Device address in hex format.
 * @param[in] addr                     Device address in string format.
 *
 * @return True if addresses are the same or false if addresses are different.
 */
bool is_address_compare(ble_gap_addr_t const * const p_connected_addr, char const * const p_addr);


/**@brief Function for initiating advertising.
 */
void adv_start(void);

/**@brief Function for disabling advertising.
 */
void adv_stop(void);

/**@brief Function for writing custom part of advetising packet
 */
void adv_write_custom_data(uint8_t adv_len, uint8_t * adv_data);
void adv_write_uuid(uint8_t * uuid);
void adv_write_company_id(uint16_t company_id);
void adv_write_flags(uint8_t flags);
void adv_write_tx_power(int8_t tx_power);

/**@brief Function for stopping scanning.
 */
void scan_stop(void);


/**@brief Function for initiating scanning.
 */
void scan_start(void);


/**@brief Function for initializing advertising.
 */
void advertising_init(void);


/**@brief Function for initializing the BLE module.
 *
 * @details Initializes buttons and LEDs, the block memory allocator module,the BLE stack, the GAP and GATT,
 *          the Connection Parameters module, the Heart Rate service, and the advertising functionality.
 */
void ble_m_init(void);


#endif // BLE_M_H_

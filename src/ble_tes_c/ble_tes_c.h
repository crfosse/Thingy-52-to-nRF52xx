/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
/**@file
 *
 * @defgroup ble_tes_c Thingy Enviroment Service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    The Thingy Enviroment Service client can be used to set a LED, and read a button state on a
 *           Thingy Enviroment service server.
 *
 * @details  This module contains the APIs and types exposed by the Thingy Enviroment Service Client
 *           module. These APIs and types can be used by the application to perform discovery of
 *           Thingy Enviroment Service at the peer and interact with it.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_tes_c_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_tes_C_BLE_OBSERVER_PRIO,
 *                                   ble_tes_c_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef BLE_TES_C_H__
#define BLE_TES_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_tes_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_TES_C_DEF(_name)                                                                        \
static ble_tes_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_TES_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_tes_c_on_ble_evt, &_name)

/**@brief   Macro for defining multiple ble_tes_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 */
#define BLE_TES_C_ARRAY_DEF(_name, _cnt)                                                            \
static ble_tes_c_t _name[_cnt];                                                                     \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                                                                \
                      BLE_TES_C_BLE_OBSERVER_PRIO,                                                  \
                      ble_tes_c_on_ble_evt, &_name, _cnt)


#define TES_BASE_UUID                 {0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x00, 0x68, 0xEF} /**< Used vendor specific UUID. */
#define BLE_UUID_TES_SERVICE          0x0200                      /**< The UUID of the Thingy Environment Service. */
#define BLE_UUID_TES_TEMPERATURE_CHAR 0x0201                      /**< The UUID of the temperature Characteristic. */
#define BLE_UUID_TES_PRESSURE_CHAR    0x0202                      /**< The UUID of the pressure Characteristic. */
#define BLE_UUID_TES_HUMIDITY_CHAR    0x0203                      /**< The UUID of the humidity Characteristic. */
#define BLE_UUID_TES_GAS_CHAR         0x0204                      /**< The UUID of the gas Characteristic. */
#define BLE_UUID_TES_COLOR_CHAR       0x0205                      /**< The UUID of the gas Characteristic. */
#define BLE_UUID_TES_CONFIG_CHAR      0x0206                      /**< The UUID of the config Characteristic. */


/**@brief TES Client event type. */
typedef enum
{
    BLE_TES_C_EVT_DISCOVERY_COMPLETE = 1,       /**< Event indicating that the Thingy enviroment Service has been discovered at the peer. */
    BLE_TES_C_EVT_TEMPERATURE_NOTIFICATION,     /**< Event indicating that a notification of the Thingy enviroment temperature characteristic has been received from the peer. */
    BLE_TES_C_EVT_PRESSURE_NOTIFICATION,        /**< Event indicating that a notification of the Thingy enviroment pressure characteristic has been received from the peer. */
    BLE_TES_C_EVT_HUMIDITY_NOTIFICATION,        /**< Event indicating that a notification of the Thingy enviroment humidity characteristic has been received from the peer. */ 
    BLE_TES_C_EVT_GAS_NOTIFICATION,             /**< Event indicating that a notification of the Thingy enviroment gas characteristic has been received from the peer. */
    BLE_TES_C_EVT_COLOR_NOTIFICATION,           /**< Event indicating that a notification of the Thingy enviroment color characteristic has been received from the peer. */
    BLE_TES_C_EVT_CONFIG_NOTIFICATION           /**< Event indicating that a notification of the Thingy enviroment config characteristic has been received from the peer. Can't handle this yet. */
} ble_tes_c_evt_type_t;

/**@brief Structure containing the event value received from the peer. */
typedef struct
{
    float evt_data;  /**< Data Value. */
} ble_evt_value_t;

/**@brief Structure containing the config value received from the peer. */
typedef struct
{
    uint8_t config_value;  /**< Config Value. */
} ble_config_t; 


/**@brief Structure containing the handles related to the Thingy Enviroment Service found on the peer. */
typedef struct
{
    uint16_t temperature_cccd_handle;       /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t pressure_cccd_handle;          /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t humidity_cccd_handle;          /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t gas_cccd_handle;               /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t color_cccd_handle;             /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t config_cccd_handle;            /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t temperature_handle;       /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t pressure_handle;          /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t humidity_handle;          /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t gas_handle;               /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t color_handle;             /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t config_handle;            /**< Handle of the <...> characteristic as provided by the SoftDevice. */
} tes_db_t;

/**@brief Thingy enviroment Event structure. */
typedef struct
{
    ble_tes_c_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the event occured.*/
    union
    {
        ble_evt_value_t value;          /**< The value of the event received. This will be filled if the evt_type is @ref BLE_TES_C_EVT_<...>_NOTIFICATION. */
        tes_db_t     peer_db;         /**< Thingy enviroment service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_tes_C_EVT_DISCOVERY_COMPLETE.*/
    } params;
} ble_tes_c_evt_t;

// Forward declaration of the ble_tes_c_t type.
typedef struct ble_tes_c_s ble_tes_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_tes_c_evt_handler_t) (ble_tes_c_t * p_ble_tes_c, ble_tes_c_evt_t * p_evt);

/**@brief Thingy Enviroment Client structure. */
struct ble_tes_c_s
{
    uint16_t                conn_handle;  /**< Connection handle as provided by the SoftDevice. */
    tes_db_t                peer_tes_db;  /**< Handles related to tes on the peer*/
    ble_tes_c_evt_handler_t evt_handler;  /**< Application event handler to be called when there is an event related to the Thingy Enviroment service. */
    uint8_t                 uuid_type;    /**< UUID type. */
};

/**@brief Thingy Enviroment Client initialization structure. */
typedef struct
{
    ble_tes_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Thingy Enviroment Client module whenever there is an event related to the Thingy Enviroment Service. */
} ble_tes_c_init_t;


/**@brief Function for initializing the Thingy Enviroment client module.
 *
 * @details This function will register with the DB Discovery module. There it registers for the
 *          Thingy Enviroment Service. Doing so will make the DB Discovery module look for the presence
 *          of a Thingy Enviroment Service instance at the peer when a discovery is started.
 *
 * @param[in] p_ble_tes_c      Pointer to the Thingy Enviroment client structure.
 * @param[in] p_ble_tes_c_init Pointer to the Thingy Enviroment initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_tes_c_init(ble_tes_c_t * p_ble_tes_c, ble_tes_c_init_t * p_ble_tes_c_init);


/**@brief Function for handling BLE events from the SoftDevice.
 *
 * @details This function will handle the BLE events received from the SoftDevice. If a BLE event
 *          is relevant to the Thingy Enviroment Client module, then it uses it to update interval
 *          variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the Thingy Enviroment client structure.
 */
void ble_tes_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for requesting the peer to start sending notification of the different characteristics
 *        Characteristic.
 *
 * @details This function will enable to notification of the characteristic at the peer
 *          by writing to the CCCD of the temperature Characteristic.
 *
 * @param[in] p_ble_tes_c Pointer to the Thingy Enviroment Client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 *          NRF_ERROR_INVALID_STATE if no connection handle has been assigned (@ref ble_tes_c_handles_assign)
 *          NRF_ERROR_NULL if the given parameter is NULL
 */
uint32_t ble_tes_c_temperature_notif_enable(ble_tes_c_t * p_ble_tes_c);
uint32_t ble_tes_c_pressure_notif_enable(ble_tes_c_t * p_ble_tes_c);
uint32_t ble_tes_c_humidity_notif_enable(ble_tes_c_t * p_ble_tes_c);
uint32_t ble_tes_c_gas_notif_enable(ble_tes_c_t * p_ble_tes_c);
uint32_t ble_tes_c_color_notif_enable(ble_tes_c_t * p_ble_tes_c);
uint32_t ble_tes_c_config_notif_enable(ble_tes_c_t * p_ble_tes_c);

/**@brief Function for handling events from the database discovery module.
 *
 * @details Call this function when getting a callback event from the DB discovery module. This
 *          function will handle an event from the database discovery module, and determine if it
 *          relates to the discovery of Thingy Enviroment service at the peer. If so, it will call the
 *          application's event handler indicating that the Thingy Enviroment service has been discovered
 *          at the peer. It also populates the event with the service related information before
 *          providing it to the application.
 *
 * @param[in] p_ble_tes_c Pointer to the Thingy Enviroment client structure.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 */
void ble_tes_on_db_disc_evt(ble_tes_c_t * p_ble_tes_c, const ble_db_discovery_evt_t * p_evt);


/**@brief     Function for assigning a Handles to this instance of tes_c.
 *
 * @details Call this function when a link has been established with a peer to associate this link
 *          to this instance of the module. This makes it  possible to handle several links and
 *          associate each link to a particular instance of this module.
 *
 * @param[in] p_ble_tes_c    Pointer to the Thingy Enviroment client structure instance to associate.
 * @param[in] conn_handle    Connection handle to associate with the given Thingy Enviroment Client Instance.
 * @param[in] p_peer_handles Thingy Enviroment Service handles found on the peer (from @ref BLE_tes_C_EVT_DISCOVERY_COMPLETE event).
 *
 */
uint32_t ble_tes_c_handles_assign(ble_tes_c_t *    p_ble_tes_c,
                                  uint16_t         conn_handle,
                                  const tes_db_t * p_peer_handles);


#ifdef __cplusplus
}
#endif

#endif // BLE_tes_C_H__

/** @} */

/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "app_log.h"
#include "em_iadc.h"
#include "gatt_db.h"
#include "sl_bt_api.h"
#include "dht11.h"
#include "app_timer.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static uint8_t app_connection = 0;

//Các hằng số phục vụ cho việc đo ADC bằng BGM220
#define CLK_SRC_ADC_FREQ 10000000
#define CLK_ADC_FREQ 10000000

#define IADC_INPUT0_PORT_PIN iadcPosInputPortAPin0
#define IADC_INPUT0_BUS ABUSALLOC
#define IADC_INPUT0_BUS_ALLOC GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_INPUT1_PORT_PIN iadcPosInputPortCPin0
#define IADC_INPUT1_BUS CDBUSALLOC
#define IADC_INPUT1_BUS_ALLOC GPIO_CDBUSALLOC_CDEVEN0_ADC0

//Các hắng sô chưa kết quả của cảm biến
static volatile IADC_Result_t sample;
static volatile uint32_t data_1;
static volatile uint32_t data_2;
int16_t _TEMP = 0;
int16_t _RH = 0;

//Hắng số cho app timer
static app_timer_t periodic_timer;
static void periodic_timer_cb(app_timer_t *timer, void *data);

void ADC_Init(){
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  IADC_ScanTable_t scanTable = IADC_SCANTABLE_DEFAULT;

  CMU_ClockEnable(cmuClock_IADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  IADC_reset(IADC0);

  CMU_ClockSelectSet(cmuClock_IADCCLK,cmuSelect_FSRCO);

  init.warmup = iadcWarmupKeepWarm;

  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
  initAllConfigs.configs[0].vRef = 3300;
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0, CLK_ADC_FREQ, 0, iadcCfgModeNormal, init.srcClkPrescale);

  initScan.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID2;

  initScan.showId = true;

  scanTable.entries[0].posInput = IADC_INPUT0_PORT_PIN;
  scanTable.entries[0].negInput = iadcNegInputGnd;
  scanTable.entries[0].includeInScan = true;

  scanTable.entries[1].posInput = IADC_INPUT1_PORT_PIN;
  scanTable.entries[1].negInput = iadcNegInputGnd;
  scanTable.entries[1].includeInScan = true;

  GPIO->IADC_INPUT0_BUS |= IADC_INPUT0_BUS_ALLOC;
  GPIO->IADC_INPUT1_BUS |= IADC_INPUT1_BUS_ALLOC;

  IADC_init(IADC0, &init, &initAllConfigs);

  IADC_initScan(IADC0, &initScan, &scanTable);

  IADC_clearInt(IADC0, _IADC_IF_MASK);

  IADC_enableInt(IADC0, IADC_IEN_SCANTABLEDONE);

  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

void IADC_IRQHandler(void){
  sample.id=0;
  sample.data=0;
  while(IADC_getScanFifoCnt(IADC0)){
      sample = IADC_pullScanFifoResult(IADC0);
      if(sample.id==0){
          data_1=sample.data;
      }else if(sample.id==1){
          data_2=sample.data;
      }
  }
  IADC_clearInt(IADC0, IADC_IF_SCANTABLEDONE);

}
/**************************************************************************//**
* Application Init.
*****************************************************************************/

SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  GPIO_PinModeSet(gpioPortA,4,gpioModePushPull,0);
  ADC_Init();
  dht22_init();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/

void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      app_log("Advertising \n");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log("Conn opened\n");
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sl_bt_connection_closed(evt->data.evt_connection_closed.reason,
                              evt->data.evt_connection_closed.connection);
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
      if(gattdb_sensor_data == evt->data.evt_gatt_server_characteristic_status.characteristic){
          if(sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags){
              sl_bt_sensor_indication_changed_cb(evt->data.evt_gatt_server_characteristic_status.connection,
                                                 (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags);
          }else if(sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags){
              sl_bt_sensor_indication_confirmed_cb(evt->data.evt_gatt_server_characteristic_status.connection);
          }
      }
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

void sl_bt_connection_closed(uint16_t reason, uint8_t connection){
  (void)reason;
  (void)connection;

  sl_status_t sc;
  sc = app_timer_stop(&periodic_timer);
  app_assert_status(sc);
}

void sl_bt_sensor_indication_changed_cb(uint8_t connection, sl_bt_gatt_client_config_flag_t client_config){
  sl_status_t sc;
  app_connection = connection;
  if(client_config == 0x2){
      sc = app_timer_start(&periodic_timer,
                           1000,
                           periodic_timer_cb,
                           NULL,
                           true);
      app_assert_status(sc);
      periodic_timer_cb(&periodic_timer, NULL);
  }else{
      (void)app_timer_stop(&periodic_timer);
  }
}

static void periodic_timer_cb(app_timer_t *timer, void *data){
  (void)timer;
  (void)data;
  sl_status_t sc;
  IADC_command(IADC0,iadcCmdStartScan);
  sc = dht22_getRHTdata(&_TEMP, &_RH);
  if(sc == SL_STATUS_FAIL){
      app_log("Fail \n");
  }
  app_log("%u %lu ", 0, data_1);
  app_log("%u %lu ", 1, data_2);
  app_log("%u %d ", 2, _TEMP);
  app_log("%u %d \n", 3, _RH);

  uint8_t buffer[12];
  buffer[0] = data_1 & 0xFF;
  buffer[1] = (data_1>>8) & 0xFF;
  buffer[2] = (data_1>>16) & 0xFF;
  buffer[3] = (data_1>>24) & 0xFF;

  buffer[4] = data_2 & 0xFF;
  buffer[5] = (data_2>>8) & 0xFF;
  buffer[6] = (data_2>>16) & 0xFF;
  buffer[7] = (data_2>>24) & 0xFF;

  buffer[8] = _TEMP & 0xFF;
  buffer[9] = (_TEMP>>8) & 0xFF;

  buffer[10] = _RH & 0xFF;
  buffer[11] = (_RH>>8) & 0xFF;

  sc = sl_bt_gatt_server_send_indication(app_connection, gattdb_sensor_data, sizeof(buffer), buffer);

}

void sl_bt_sensor_indication_confirmed_cb(uint8_t connection){
  (void) connection;
}

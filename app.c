/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
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
#include "app_log.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "em_gpio.h"
#include "app_timer.h"

#define thong_gio        gpioPortB, 1
#define bom_nuoc      gpioPortB, 2
#define phun_suong   gpioPortB, 3
#define mai_che      gpioPortB, 4

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static uint8_t app_connection = 0;

static app_timer_t periodic_timer;
static void periodic_timer_cb(app_timer_t *timer, void *data);

static volatile bool thong_gio_t = 0;
static volatile bool bom_nuoc_t = 0;
static volatile bool phun_suong_t = 0;
static volatile bool mai_che_t = 0;
/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  GPIO_PinModeSet(thong_gio, gpioModeInputPullFilter ,0);
  GPIO_PinModeSet(bom_nuoc, gpioModeInputPullFilter ,0);
  GPIO_PinModeSet(phun_suong, gpioModeInputPullFilter ,0);
  GPIO_PinModeSet(mai_che, gpioModeInputPullFilter ,0);
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{

}

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////

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
    //Đây là các sự kiện khi xảy ra
    case sl_bt_evt_system_boot_id:
      //Sự kiện khởi tạo bắt đầu quảng cáo
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      // Tạo một struct chứa thông tin quảng cáo mà thiết bị sẽ gửi đi
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      // Tạo dữ liệu quảng cáo cho một tập quảng cáo BLE
      // sl_bt_advertiser_general_discoverable nghĩa là thiết bị này có thể phát hiện được bới bất kỳ thiết bị nào muốn kết nối BLE
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
      app_log_info("Connection opened.\n");
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log_info("Connection closed.\n");
      sl_bt_connection_closed(evt->data.evt_connection_closed.reason,
                              evt->data.evt_connection_closed.connection);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      // taaoj tập quảng cáo cho ble chuẩn bị quảng cáo lại
      app_assert_status(sc);


      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      // Bắt đầu quảng cáo lại
      app_assert_status(sc);
      app_log("Advertising \n");
      break;

    // -------------------------------
    // This event indicates that the value of an attribute in the local GATT
    // database was changed by a remote GATT client.
    case sl_bt_evt_gatt_server_attribute_value_id:
      // Dấy là event xảy ra khi một giá trị attribute của cơ ssowr dữ liệu database bị thay đổi bới GATT client
      // The value of the gattdb_led_control characteristic was changed.
      if (gattdb_led_control == evt->data.evt_gatt_server_attribute_value.attribute) {
        //gattdb_led_control là ID của một thuộc tính GATT trong cơ sở dữ liệu GATT database
        //Trong trường hợp này gattdb_led_control chính là đại diện cho chương trình điều khiển led
        //evt là struct chứa các dữ liệu của sự kiện
        //evt->data.evt_gatt_server_attribute_value.attribute là id của thuộc tính mà giá trị của nó thay đổi trong trường hớp này nghĩa là giá trị attribute của thuộc tính điều khieenr led đã thay đổi
        // Ở đây có thề sử dụng các id của các thược tính khác nếu có nhiều hơn 1 thược tính
        uint8_t data_recv;
        size_t data_recv_len;

        // Read characteristic value.
        sc = sl_bt_gatt_server_read_attribute_value(gattdb_led_control,
                                                    0,
                                                    sizeof(data_recv),
                                                    &data_recv_len,
                                                    &data_recv);
        //Đọc giá trị attribute từ gatt database
        (void)data_recv_len;
        app_log_status_error(sc);

        if (sc != SL_STATUS_OK) {
          //  Đảm báo đọc được dữ liệu
          break;
        }

        // Toggle LED.
        // Đọc dữ liệu và bật tắt led
        app_log("Data_recieved: %d",data_recv);
        if (data_recv == 0x00) {
            GPIO_PinOutClear(bom_nuoc);
        } else if (data_recv == 0x01) {
            GPIO_PinOutSet(bom_nuoc);
        } else if(data_recv == 0x02){
            GPIO_PinOutClear(thong_gio);
        } else if(data_recv == 0x03){
            GPIO_PinOutSet(thong_gio);
        } else if(data_recv == 0x04){
            GPIO_PinOutClear(phun_suong);
        } else if(data_recv == 0x05){
            GPIO_PinOutSet(phun_suong);
        } else if(data_recv == 0x06){
            GPIO_PinOutClear(mai_che);
        } else if(data_recv == 0x07){
            GPIO_PinOutSet(mai_che);
        }else if(data_recv == 0x37){
            GPIO_PinOutSet(mai_che);
            GPIO_PinOutSet(thong_gio);
        }else if(data_recv == 0x26){
            GPIO_PinOutClear(mai_che);
            GPIO_PinOutClear(thong_gio);
        }else{
            app_log("Invalid attribute value: 0x%02x\n", (int)data_recv);
        }
      }
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
          if(gattdb_led_indicate == evt->data.evt_gatt_server_characteristic_status.characteristic){
              if(sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags){
                  sl_bt_sensor_indication_changed_cb(evt->data.evt_gatt_server_characteristic_status.connection,
                                                     (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags);
              }else if(sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags){
                  sl_bt_sensor_indication_confirmed_cb(evt->data.evt_gatt_server_characteristic_status.connection);
              }else{
                  app_assert(false,
                             "[E: 0x%04x] Unexpected status flag in evt_gatt_server_characteristic_status\n",
                             (int)evt->data.evt_gatt_server_characteristic_status.status_flags);
              }
          }

    // -------------------------------
    // This event occurs when the remote device enabled or disabled the
    // notification.

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
                           2500,
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
  uint8_t result = 0;

  thong_gio_t = GPIO_PinInGet(thong_gio);
  bom_nuoc_t = GPIO_PinInGet(bom_nuoc);
  phun_suong_t = GPIO_PinInGet(phun_suong);
  mai_che_t = GPIO_PinInGet(mai_che);

  result |= (bom_nuoc_t & 0x01) << 3;
  result |= (thong_gio_t & 0x01) << 2;
  result |= (phun_suong_t & 0x01) << 1;
  result |= (mai_che_t & 0x01);

  sc = sl_bt_gatt_server_send_indication(app_connection, gattdb_led_indicate, sizeof(result), &result);

}

void sl_bt_sensor_indication_confirmed_cb(uint8_t connection){
  (void) connection;
}



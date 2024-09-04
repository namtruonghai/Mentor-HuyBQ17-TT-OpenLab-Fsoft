from dataclasses import dataclass
import os.path
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))
from common.util import ArgumentParser, BluetoothApp, get_connector, find_service_in_advertisement
import st.status as status
import struct
import paho.mqtt.client as mqtt

#Hăng số của broker cần lưu để kết nói với broker
broker = "sparrow.rmq.cloudamqp.com"
port = 1883
username = "ydlziysx:ydlziysx"
password = "jtjD63G9r9UNSV7od5unLQWmZbxEl51b"

#Constant bên peripheral đọc
SENSOR_SERVICE = b"\xd0.-\x8a\x1f\xe3\x8d\xbe?F\x00\x07\xe4\xb6e\xfa"

#Constant bên peripheral viết
BLINKY_SERVICE = b'$\x12\xb5\xcb\xd4`\x80\x0c\x15\xc3\x9b\xa9\xacZ\x8a\xde'

#constant dữ liệu đầu vào min max có thể thay đổi phụ thuộc vào app
SENSOR_1_MIN = 0
SENSOR_1_MAX = 100
SENSOR_2_MIN = 0
SENSOR_2_MAX = 10
TEMP_MAX = 100
TEMP_MIN = 0
RH_MIN = 100
RH_MAX = 130

#Constant set up cho tự động hay không với các thiết bị
#0 là tắt 1 là bật 2 là tự động
STATUS_BOM = ""
STATUS_PHUN_SUONG = ""
STATUS_MAI_CHE = ""
STATUS_QUAT = ""

#Constant tạo riêng cho chế độ tự động của app mặc định là tự động
AUTOMATIC = ""

# Biến FLAG THÔNG BÁO Mỗi khi có dữ liệu mới từ app
FLAG = 0

#Constant hằng số khởi tạo việc quét
CONN_INTERVAL_MIN = 80
CONN_INTERVAL_MAX = 80
CONN_SLAVE_LATENCY = 0
CONN_TIMEOUT = 100
CONN_MIN_CE_LENGTH = 0
CONN_MAX_CE_MENGTH = 65535

SCAN_INTERVAL = 16
SCAN_WINDOW = 16
SCAN_PASSIVE = 0

# Khởi tạo kết nối với broker cho code 
client = mqtt.Client()
client.username_pw_set(username, password)

def on_message(client, userdata, msg):
    print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")
    
    global FLAG
    global SENSOR_1_MIN
    global SENSOR_1_MAX
    global SENSOR_2_MIN
    global SENSOR_2_MAX
    global TEMP_MAX
    global TEMP_MIN
    global RH_MIN
    global RH_MAX

    global STATUS_BOM
    global STATUS_PHUN_SUONG
    global STATUS_MAI_CHE
    global STATUS_QUAT
    global AUTOMATIC

    FLAG = 1
    if msg.topic == "control/mode":
        AUTOMATIC = msg.payload.decode()

    if msg.topic == "control/bomnuoc/status":
        STATUS_BOM = msg.payload.decode()

    if msg.topic == "control/thonggio/status":
        STATUS_QUAT = msg.payload.decode()

    if msg.topic == "control/phunsuong/status":
        STATUS_PHUN_SUONG = msg.payload.decode()

    if msg.topic == "control/maiche/status":
        STATUS_MAI_CHE = msg.payload.decode()

    if msg.topic == "control/bomnuoc/doammin":
        if msg.payload.decode() != "clear":
            SENSOR_1_MIN = msg.payload.decode()
            SENSOR_1_MIN = float(SENSOR_1_MIN)

    if msg.topic == "control/bomnuoc/doammax":
        if msg.payload.decode() != "clear":
            SENSOR_1_MAX = msg.payload.decode()
            SENSOR_1_MAX = float(SENSOR_1_MAX)

    if msg.topic == "control/thonggio/nhietdomin":
        if msg.payload.decode() != "clear":
            TEMP_MIN = msg.payload.decode()
            TEMP_MIN = float(TEMP_MIN)
    
    if msg.topic == "control/thonggio/nhietdomax":
        if msg.payload.decode() != "clear":
            TEMP_MAX = msg.payload.decode()
            TEMP_MAX = float(TEMP_MAX)
    
    if msg.topic == "control/phunsuong/doamkkmin":
        if msg.payload.decode() != "clear":
            RH_MIN = msg.payload.decode()
            RH_MIN = float(RH_MIN)
    
    if msg.topic == "control/phunsuong/doamkkmax":
        if msg.payload.decode() != "clear":
            RH_MAX = msg.payload.decode()
            RH_MAX = float(RH_MAX)
    
    if msg.topic == "control/maiche/chisouvmin":
        if msg.payload.decode() != "clear":
            SENSOR_2_MIN = msg.payload.decode()
            SENSOR_2_MIN = float(SENSOR_2_MIN)
    
    if msg.topic == "control/maiche/chisouvmax":
        if msg.payload.decode() != "clear":
            SENSOR_2_MAX = msg.payload.decode()
            SENSOR_2_MAX = float(SENSOR_2_MAX)

client.on_message = on_message

client.connect(broker, port, 60)
client.loop_start()

topic = "control/#"
client.subscribe(topic)


# DATA CLASS ĐẠI DIỆN CHO CONNECTION
class Connections:
    def __init__(self, address: str, address_type: int):
        self.address = address
        self.address_type = address_type


class App(BluetoothApp):
    def bt_evt_system_boot(self,evt):
        self.lib.bt.connection.set_default_parameters(
            CONN_INTERVAL_MIN,
            CONN_INTERVAL_MAX,
            CONN_SLAVE_LATENCY,
            CONN_TIMEOUT,
            CONN_MIN_CE_LENGTH,
            CONN_MAX_CE_MENGTH
        )
        self.lib.bt.scanner.start(
            self.lib.bt.scanner.SCAN_PHY_SCAN_PHY_1M,
            self.lib.bt.scanner.DISCOVER_MODE_DISCOVER_GENERIC
        )
        print("Scanning")
        self.conn_state = "scanning"
        self.connection = dict[int,Connections]()

    def bt_evt_scanner_legacy_advertisement_report(self,evt):
        if(evt.event_flags & self.lib.bt.scanner.EVENT_FLAG_EVENT_FLAG_CONNECTABLE and evt.event_flags & self.lib.bt.scanner.EVENT_FLAG_EVENT_FLAG_SCANNABLE):
            if(find_service_in_advertisement(evt.data,SENSOR_SERVICE)) and len(self.connection) == 0:
                print("Service 1 found")
                self.lib.bt.scanner.stop()
                self.lib.bt.connection.open(
                    evt.address,
                    evt.address_type,
                    self.lib.bt.gap.PHY_PHY_1M
                )
                self.conn_state = "opening_1"
                print("Opening 1")
            
            if(find_service_in_advertisement(evt.data,BLINKY_SERVICE)) and len(self.connection) == 1:
                print("Service 2 found")
                self.lib.bt.scanner.stop()
                self.lib.bt.connection.open(
                    evt.address,
                    evt.address_type,
                    self.lib.bt.gap.PHY_PHY_1M
                )
                self.conn_state = "opening_2"
                print("Opening 2")

    def bt_evt_connection_opened(self,evt):
        if self.conn_state == "opening_1":
            print(f"Connection opened to {evt.address}")
            self.connection[evt.connection] = Connections(evt.address,evt.address_type)
            self.lib.bt.scanner.start(
                self.lib.bt.scanner.SCAN_PHY_SCAN_PHY_1M,
                self.lib.bt.scanner.DISCOVER_MODE_DISCOVER_GENERIC
            )
            self.conn_state = "scanning"
            print("Scanning")

        if self.conn_state == "opening_2":
            print(f"Connection opened to {evt.address}")
            self.connection[evt.connection] = Connections(evt.address,evt.address_type)
            self.lib.bt.gatt.set_characteristic_notification(
                1,
                27,
                self.lib.bt.gatt.CLIENT_CONFIG_FLAG_INDICATION
            )
            self.conn_state = "enable_indication_1"
            print("Enabled indication 1")
    
    def bt_evt_gatt_procedure_completed(self,evt):
        if evt.result !=status.OK:
            address = self.connection[evt.connection].address
            print(f"GATT procedure for {address} completed with status {evt.result:#x}: {evt.result}")
            return
        
        if self.conn_state == "enable_indication_1":
            self.lib.bt.gatt.set_characteristic_notification(
                2,
                27,
                self.lib.bt.gatt.CLIENT_CONFIG_FLAG_INDICATION
            )
            self.conn_state = "enable_indication_2"
            print("Enabled indication 2")
        
        if self.conn_state == "enable_indication_2":
            self.conn_state == "running"
            print("Running")

    def bt_evt_connection_closed(self,evt):
        address = self.connection[evt.connection].address
        print(f"Connection to {address} closed with reason {evt.reason:#x}: '{evt.reason}'")
        del self.connection[evt.connection]
        if self.conn_state!="scanning" and len(self.connection) < 2:
            self.lib.bt.scanner.start(
                self.lib.bt.scanner.SCAN_PHY_SCAN_PHY_1M,
                self.lib.bt.scanner.DISCOVER_MODE_DISCOVER_GENERIC
            )
            self.conn_state = "scanning"
            print("Scanning")

    def bt_evt_gatt_server_indication_timeout(self,evt):
        del self.connection[evt.connection]
        if self.conn_state!="scanning" and len(self.connection) < 2:
            self.lib.bt.scanner.start(
                self.lib.bt.scanner.SCAN_PHY_SCAN_PHY_1M,
                self.lib.bt.scanner.DISCOVER_MODE_DISCOVER_GENERIC
            )
            self.conn_state = "scanning"
            print("Scanning")

    def send_data(self,data):
        time.sleep(0.4)
        print(f"Writing {data}")
        self.lib.bt.gatt.write_characteristic_value_without_response(
            2,
            30,
            data
        )

    def bt_evt_gatt_characteristic_value(self,evt):

        global FLAG
        global STATUS_BOM
        global STATUS_PHUN_SUONG
        global STATUS_MAI_CHE
        global STATUS_QUAT
        global AUTOMATIC
        global data_RHdat
        global data_UV
        global data_Temp
        global data_RH

        if evt.connection == 1:
            data_sensor_1, data_sensor_2, data_Temp, data_RH = struct.unpack('<2I2h', evt.value)
            data_RHdat = data_sensor_1/4096*100
            data_RHdat = round(data_RHdat)
            data_UV = data_sensor_2/4096*10
            data_UV = round(data_UV)
            client.publish("cambien/nhietdo", data_Temp/10)
            client.publish("cambien/doamkk", data_RH/10)
            client.publish("cambien/doamdat", data_RHdat)
            client.publish("cambien/uv", data_UV)
            self.lib.bt.gatt.send_characteristic_confirmation(1)
        
        if evt.connection == 2:
            status = evt.value[0] & 0x0F
            status = format(status, '04b')
            status = AUTOMATIC + status
            client.publish("status",status)
            self.lib.bt.gatt.send_characteristic_confirmation(2)
        
        if FLAG == 1:
            if STATUS_BOM == "1":
                self.send_data(b"\x01")
                STATUS_BOM = ""
            elif STATUS_BOM == "0":
                self.send_data(b"\x00")
                STATUS_BOM = ""
            
            if STATUS_MAI_CHE == "1":
                self.send_data(b"\x07")
                STATUS_MAI_CHE = ""
            elif STATUS_MAI_CHE == "0":
                self.send_data(b"\x06")
                STATUS_MAI_CHE = ""
            
            if STATUS_QUAT == "1":
                self.send_data(b"\x03")
                STATUS_QUAT = ""
            elif STATUS_QUAT == "0":
                self.send_data(b"\x02")
                STATUS_QUAT = ""

            if STATUS_PHUN_SUONG == "1":
                self.send_data(b"\x05")
                STATUS_PHUN_SUONG = ""
            elif STATUS_PHUN_SUONG == "0":
                self.send_data(b"\x04")
                STATUS_PHUN_SUONG = ""
            
            FLAG = 0

        if AUTOMATIC == "1":
            if data_RHdat < SENSOR_1_MIN:
                self.send_data(b"\x01")
            if data_RHdat > SENSOR_1_MAX:
                self.send_data(b"\x00")

            if data_UV > SENSOR_2_MAX:
                self.send_data(b"\x07")
            if data_UV < SENSOR_2_MIN:
                self.send_data(b"\x06")

            if data_Temp > TEMP_MAX:
                self.send_data(b"\x37")
            elif data_Temp < TEMP_MAX:
                self.send_data(b"\x26")
        
            if data_RH < RH_MIN:
                self.send_data(b"\x05")
            if data_RH > RH_MAX:
                self.send_data(b"\x04")

if __name__ == "__main__":
    parser=ArgumentParser(description=__doc__)
    args = parser.parse_args()
    connector = get_connector(args)
    app = App(connector)
    app.run()

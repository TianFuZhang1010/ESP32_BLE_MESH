/* PROJECT:  GSI-3 LED Controller project
 * FILE:  ble_mesh.h
 * DISC: BLE Mesh processing
 */

extern uint8_t cur_onoff_value;

void user_ble_mesh_init();
void send_RedValue(uint8_t value);
void send_BlueValue(uint8_t value);
void send_WhiteValue(uint8_t value);
void send_TurnOnOff(uint8_t value);
void resend_values();

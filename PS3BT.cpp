/* Copyright (C) 2017 Norbert Fekete. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Ported version for ESP32.

 Original contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "PS3BT.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "esp_log.h"

static const char *tag = "PS3BT";

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define pgm_read_byte(addr)     (*reinterpret_cast<const uint8_t*>(addr))
#define pgm_read_dword(addr) 		(*reinterpret_cast<const uint32_t*>(addr))

// To enable serial debugging see "settings.h"
//#define EXTRADEBUG // Uncomment to get even more debugging data
//#define PRINTREPORT // Uncomment to print the report send by the PS3 Controllers

PS3BT::PS3BT(BTD *p) :
BluetoothService(p) // Pointer to USB class instance - mandatory
{

        HIDBuffer[0] = 0x52; // HID BT Set_report (0x50) | Report Type (Output 0x02)
        HIDBuffer[1] = 0x01; // Report ID

        // Needed for PS3 Move Controller commands to work via bluetooth
        HIDMoveBuffer[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        HIDMoveBuffer[1] = 0x02; // Report ID

        /* Set device cid for the control and intterrupt channelse - LSB */
        control_dcid[0] = 0x40; // 0x0040
        control_dcid[1] = 0x00;
        interrupt_dcid[0] = 0x41; // 0x0041
        interrupt_dcid[1] = 0x00;

        Reset();
}

bool PS3BT::getButtonPress(ButtonEnum b) {
        return (ButtonState & pgm_read_dword(&PS3_BUTTONS[(uint8_t)b]));
}

bool PS3BT::getButtonClick(ButtonEnum b) {
        uint32_t button = pgm_read_dword(&PS3_BUTTONS[(uint8_t)b]);
        bool click = (ButtonClickState & button);
        ButtonClickState &= ~button; // Clear "click" event
        return click;
}

uint8_t PS3BT::getAnalogButton(ButtonEnum a) {
        return (uint8_t)(l2capinbuf[pgm_read_byte(&PS3_ANALOG_BUTTONS[(uint8_t)a])]);
}

uint8_t PS3BT::getAnalogHat(AnalogHatEnum a) {
        return (uint8_t)(l2capinbuf[(uint8_t)a + 15]);
}

int16_t PS3BT::getSensor(SensorEnum a) {
        if(PS3Connected) {
                if(a == aX || a == aY || a == aZ || a == gZ)
                        return ((l2capinbuf[(uint16_t)a] << 8) | l2capinbuf[(uint16_t)a + 1]);
                else
                        return 0;
        } else if(PS3MoveConnected) {
                if(a == mXmove || a == mYmove) // These are all 12-bits long
                        return (((l2capinbuf[(uint16_t)a] & 0x0F) << 8) | (l2capinbuf[(uint16_t)a + 1]));
                else if(a == mZmove || a == tempMove) // The tempearature is also 12 bits long
                        return ((l2capinbuf[(uint16_t)a] << 4) | ((l2capinbuf[(uint16_t)a + 1] & 0xF0) >> 4));
                else // aXmove, aYmove, aZmove, gXmove, gYmove and gZmove
                        return (l2capinbuf[(uint16_t)a] | (l2capinbuf[(uint16_t)a + 1] << 8));
        } else
                return 0;
}

float PS3BT::getAngle(AngleEnum a) {
        float accXval, accYval, accZval;

        if(PS3Connected) {
                // Data for the Kionix KXPC4 used in the DualShock 3
                const float zeroG = 511.5f; // 1.65/3.3*1023 (1.65V)
                accXval = -((float)getSensor(aX) - zeroG);
                accYval = -((float)getSensor(aY) - zeroG);
                accZval = -((float)getSensor(aZ) - zeroG);
        } else if(PS3MoveConnected) {
                // It's a Kionix KXSC4 inside the Motion controller
                const uint16_t zeroG = 0x8000;
                accXval = -(int16_t)(getSensor(aXmove) - zeroG);
                accYval = (int16_t)(getSensor(aYmove) - zeroG);
                accZval = (int16_t)(getSensor(aZmove) - zeroG);
        } else
                return 0;

        // Convert to 360 degrees resolution
        // atan2 outputs the value of -π to π (radians)
        // We are then converting it to 0 to 2π and then to degrees
        if(a == Pitch)
                return (atan2f(accYval, accZval) + PI) * RAD_TO_DEG;
        else
                return (atan2f(accXval, accZval) + PI) * RAD_TO_DEG;
}

float PS3BT::get9DOFValues(SensorEnum a) { // Thanks to Manfred Piendl
        if(!PS3MoveConnected)
                return 0;
        int16_t value = getSensor(a);
        if(a == mXmove || a == mYmove || a == mZmove) {
                if(value > 2047)
                        value -= 0x1000;
                return (float)value / 3.2f; // unit: muT = 10^(-6) Tesla
        } else if(a == aXmove || a == aYmove || a == aZmove) {
                if(value < 0)
                        value += 0x8000;
                else
                        value -= 0x8000;
                return (float)value / 442.0f; // unit: m/(s^2)
        } else if(a == gXmove || a == gYmove || a == gZmove) {
                if(value < 0)
                        value += 0x8000;
                else
                        value -= 0x8000;
                if(a == gXmove)
                        return (float)value / 11.6f; // unit: deg/s
                else if(a == gYmove)
                        return (float)value / 11.2f; // unit: deg/s
                else // gZmove
                        return (float)value / 9.6f; // unit: deg/s
        } else
                return 0;
}

void PS3BT::getTemperature(char *output) {

		strcpy(output, "Error");
        if(PS3MoveConnected) {
                int16_t input = getSensor(tempMove);

                char tmp[10];
                strcpy(output, itoa(input / 100, tmp, 10));
                strcat(output, ".");
                if(input % 100 < 10)
                		strcat(output, "0");
                strcat(output, itoa(input % 100, tmp, 10));
        }
}

bool PS3BT::getStatus(StatusEnum c) {
        return (l2capinbuf[(uint16_t)c >> 8] == ((uint8_t)c & 0xff));
}

void PS3BT::printStatusString() {
	    char statusOutput[256]; // Max string length plus null character
        if(PS3Connected || PS3NavigationConnected) {
                strcpy(statusOutput, "ConnectionStatus: ");

                if(getStatus(Plugged)) strcat(statusOutput, "Plugged");
                else if(getStatus(Unplugged)) strcat(statusOutput, "Unplugged");
                else strcat(statusOutput, "Error");

                strcat(statusOutput, " - PowerRating: ");

                if(getStatus(Charging)) strcat(statusOutput, "Charging");
                else if(getStatus(NotCharging)) strcat(statusOutput, "Not Charging");
                else if(getStatus(Shutdown)) strcat(statusOutput, "Shutdown");
                else if(getStatus(Dying)) strcat(statusOutput, "Dying");
                else if(getStatus(Low)) strcat(statusOutput, "Low");
                else if(getStatus(High)) strcat(statusOutput, "High");
                else if(getStatus(Full)) strcat(statusOutput, "Full");
                else strcat(statusOutput, "Error");

                strcat(statusOutput, " - WirelessStatus: ");

                if(getStatus(CableRumble)) strcat(statusOutput, "Cable - Rumble is on");
                else if(getStatus(Cable)) strcat(statusOutput, "Cable - Rumble is off");
                else if(getStatus(BluetoothRumble)) strcat(statusOutput, "Bluetooth - Rumble is on");
                else if(getStatus(Bluetooth)) strcat(statusOutput, "Bluetooth - Rumble is off");
                else strcat(statusOutput, "Error");
        } else if(PS3MoveConnected) {
        		strcpy(statusOutput, "PowerRating: ");

                if(getStatus(MoveCharging)) strcat(statusOutput, "Charging");
                else if(getStatus(MoveNotCharging)) strcat(statusOutput, "Not Charging");
                else if(getStatus(MoveShutdown)) strcat(statusOutput, "Shutdown");
                else if(getStatus(MoveDying)) strcat(statusOutput, "Dying");
                else if(getStatus(MoveLow)) strcat(statusOutput, "Low");
                else if(getStatus(MoveHigh)) strcat(statusOutput, "High");
                else if(getStatus(MoveFull)) strcat(statusOutput, "Full");
                else strcat(statusOutput, "Error");
        } else {
        		strcat(statusOutput, "Error");
        }

        ESP_LOGI(tag, "%s", statusOutput);
}

void PS3BT::Reset() {
        PS3Connected = false;
        PS3MoveConnected = false;
        PS3NavigationConnected = false;
        activeConnection = false;
        l2cap_event_flag = 0; // Reset flags
        l2cap_state = L2CAP_WAIT;

        // Needed for PS3 Dualshock Controller commands to work via Bluetooth
        for(uint8_t i = 0; i < PS3_REPORT_BUFFER_SIZE; i++)
                HIDBuffer[i + 2] = pgm_read_byte(&PS3_REPORT_BUFFER[i]); // First two bytes reserved for report type and ID

        ESP_LOGI(tag, "Reset done");
}

void PS3BT::disconnect() { // Use this void to disconnect any of the controllers
        // First the HID interrupt channel has to be disconnected, then the HID control channel and finally the HCI connection
        pBtd->l2cap_disconnection_request(hci_handle, ++identifier, interrupt_scid, interrupt_dcid);
        Reset();
        l2cap_state = L2CAP_INTERRUPT_DISCONNECT;
}

void PS3BT::ACLData(uint8_t* ACLData) {
        if(!pBtd->l2capConnectionClaimed && !PS3Connected && !PS3MoveConnected && !PS3NavigationConnected && !activeConnection && !pBtd->connectToWii && !pBtd->incomingWii && !pBtd->pairWithWii) {
                if(ACLData[8] == L2CAP_CMD_CONNECTION_REQUEST) {
                        if((ACLData[12] | (ACLData[13] << 8)) == HID_CTRL_PSM) {
                                pBtd->l2capConnectionClaimed = true; // Claim that the incoming connection belongs to this service
                                activeConnection = true;
                                hci_handle = pBtd->hci_handle; // Store the HCI Handle for the connection
                                l2cap_state = L2CAP_WAIT;
                                remote_name_first = pBtd->remote_name[0]; // Store the first letter in remote name for the connection
                                if(pBtd->hci_version < 3) { // Check the HCI Version of the Bluetooth dongle
                                        ESP_LOGE(tag, "Your dongle may not support reading the analog buttons, sensors and status\nYour HCI Version is: %d", pBtd->hci_version);
                                        ESP_LOGE(tag, "But should be at least 3\nThis means that it doesn't support Bluetooth Version 2.0+EDR");
                                }
                        }
                }
        }

        if(checkHciHandle(ACLData, hci_handle)) { // acl_handle_ok
                memcpy(l2capinbuf, ACLData, BULK_MAXPKTSIZE);
                if((l2capinbuf[6] | (l2capinbuf[7] << 8)) == 0x0001U) { // l2cap_control - Channel ID for ACL-U
                        if(l2capinbuf[8] == L2CAP_CMD_COMMAND_REJECT) {
                        		ESP_LOGE(tag, "L2CAP Command Rejected - Reason: 0x%x 0x%x Data: 0x%x 0x%x 0x%x 0x%x", l2capinbuf[13], l2capinbuf[12], l2capinbuf[17], l2capinbuf[16], l2capinbuf[15], l2capinbuf[14]);
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONNECTION_REQUEST) {
                        		ESP_LOGI(tag, "L2CAP Connection Request - PSM: 0x%x 0x%x SCID: 0x%x 0x%x Identifier: 0x%x", l2capinbuf[13], l2capinbuf[12], l2capinbuf[15], l2capinbuf[14], l2capinbuf[9]);
                                if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_CTRL_PSM) {
                                        identifier = l2capinbuf[9];
                                        control_scid[0] = l2capinbuf[14];
                                        control_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST);
                                } else if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_INTR_PSM) {
                                        identifier = l2capinbuf[9];
                                        interrupt_scid[0] = l2capinbuf[14];
                                        interrupt_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST);
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONFIG_RESPONSE) {
                                if((l2capinbuf[16] | (l2capinbuf[17] << 8)) == 0x0000) { // Success
                                        if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {
                                                //Notify(PSTR("\r\nHID Control Configuration Complete"), 0x80);
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS);
                                        } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {
                                                //Notify(PSTR("\r\nHID Interrupt Configuration Complete"), 0x80);
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS);
                                        }
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONFIG_REQUEST) {
                                if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {
                                        //Notify(PSTR("\r\nHID Control Configuration Request"), 0x80);
                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], control_scid);
                                } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {
                                        //Notify(PSTR("\r\nHID Interrupt Configuration Request"), 0x80);
                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], interrupt_scid);
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_DISCONNECT_REQUEST) {
                                if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {
                                        ESP_LOGI(tag, "Disconnect Request: Control Channel");
                                        identifier = l2capinbuf[9];
                                        pBtd->l2cap_disconnection_response(hci_handle, identifier, control_dcid, control_scid);
                                        Reset();
                                } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {
                                		ESP_LOGI(tag, "Disconnect Request: Interrupt Channel");
                                        identifier = l2capinbuf[9];
                                        pBtd->l2cap_disconnection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid);
                                        Reset();
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_DISCONNECT_RESPONSE) {
                                if(l2capinbuf[12] == control_scid[0] && l2capinbuf[13] == control_scid[1]) {
                                        //Notify(PSTR("\r\nDisconnect Response: Control Channel"), 0x80);
                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE);
                                } else if(l2capinbuf[12] == interrupt_scid[0] && l2capinbuf[13] == interrupt_scid[1]) {
                                        //Notify(PSTR("\r\nDisconnect Response: Interrupt Channel"), 0x80);
                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE);
                                }
                        }
                        else {
                        		ESP_LOGE(tag, "L2CAP Unknown Signaling Command: 0x%x", l2capinbuf[8]);
                        }
                } else if(l2capinbuf[6] == interrupt_dcid[0] && l2capinbuf[7] == interrupt_dcid[1]) { // l2cap_interrupt
                        ESP_LOGI(tag, "L2CAP Interrupt");
                        if(PS3Connected || PS3MoveConnected || PS3NavigationConnected) {
                                /* Read Report */
                                if(l2capinbuf[8] == 0xA1) { // HID_THDR_DATA_INPUT
                                        lastMessageTime = xTaskGetTickCount(); // Store the last message time

                                        if(PS3Connected || PS3NavigationConnected)
                                                ButtonState = (uint32_t)(l2capinbuf[11] | ((uint16_t)l2capinbuf[12] << 8) | ((uint32_t)l2capinbuf[13] << 16));
                                        else if(PS3MoveConnected)
                                                ButtonState = (uint32_t)(l2capinbuf[10] | ((uint16_t)l2capinbuf[11] << 8) | ((uint32_t)l2capinbuf[12] << 16));

                                        //Notify(PSTR("\r\nButtonState", 0x80);
                                        //PrintHex<uint32_t>(ButtonState, 0x80);

                                        if(ButtonState != OldButtonState) {
                                                ButtonClickState = ButtonState & ~OldButtonState; // Update click state variable
                                                OldButtonState = ButtonState;
                                        }

                                        ESP_LOGI(tag, "Report: ");
                                        for(uint8_t i = 10; i < 58; i++) {
                                                printf("0x%x ", l2capinbuf[i]);
                                        }
                                        printf("/n");
                                }
                        }
                }
                L2CAP_task();
        }
}

void PS3BT::L2CAP_task() {
        switch(l2cap_state) {
                case L2CAP_WAIT:
                        if(l2cap_check_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST)) {
                        		ESP_LOGI(tag, "HID Control Incoming Connection Request");
                                pBtd->l2cap_connection_response(hci_handle, identifier, control_dcid, control_scid, BTD_PENDING);
                                vTaskDelay(1 / portTICK_PERIOD_MS);
                                pBtd->l2cap_connection_response(hci_handle, identifier, control_dcid, control_scid, BTD_SUCCESSFUL);
                                identifier++;
                                vTaskDelay(1 / portTICK_PERIOD_MS);
                                pBtd->l2cap_config_request(hci_handle, identifier, control_scid);
                                l2cap_state = L2CAP_CONTROL_SUCCESS;
                        }
                        break;

                case L2CAP_CONTROL_SUCCESS:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS)) {
                        		ESP_LOGI(tag, "HID Control Successfully Configured");
                                l2cap_state = L2CAP_INTERRUPT_SETUP;
                        }
                        break;

                case L2CAP_INTERRUPT_SETUP:
                        if(l2cap_check_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST)) {
                        		ESP_LOGI(tag, "HID Interrupt Incoming Connection Request");
                                pBtd->l2cap_connection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid, BTD_PENDING);
                                vTaskDelay(1 / portTICK_PERIOD_MS);
                                pBtd->l2cap_connection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid, BTD_SUCCESSFUL);
                                identifier++;
                                vTaskDelay(1 / portTICK_PERIOD_MS);
                                pBtd->l2cap_config_request(hci_handle, identifier, interrupt_scid);

                                l2cap_state = L2CAP_INTERRUPT_CONFIG_REQUEST;
                        }
                        break;

                case L2CAP_INTERRUPT_CONFIG_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS)) { // Now the HID channels is established
                        		ESP_LOGI(tag, "HID Interrupt Successfully Configured");
                                if(remote_name_first == 'M') { // First letter in Motion Controller ('M')
                                        memset(l2capinbuf, 0, BULK_MAXPKTSIZE); // Reset l2cap in buffer as it sometimes read it as a button has been pressed
                                        l2cap_state = TURN_ON_LED;
                                } else
                                        l2cap_state = PS3_ENABLE_SIXAXIS;
                                timer = xTaskGetTickCount();
                        }
                        break;

                        /* These states are handled in Run() */

                case L2CAP_INTERRUPT_DISCONNECT:
                        if(l2cap_check_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE)) {
                        		ESP_LOGI(tag, "Disconnected Interrupt Channel");
                                identifier++;
                                pBtd->l2cap_disconnection_request(hci_handle, identifier, control_scid, control_dcid);
                                l2cap_state = L2CAP_CONTROL_DISCONNECT;
                        }
                        break;

                case L2CAP_CONTROL_DISCONNECT:
                        if(l2cap_check_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE)) {
                        		ESP_LOGI(tag, "Disconnected Control Channel");
                                pBtd->hci_disconnect(hci_handle);
                                hci_handle = -1; // Reset handle
                                l2cap_event_flag = 0; // Reset flags
                                l2cap_state = L2CAP_WAIT;
                        }
                        break;
        }
}

void PS3BT::Run() {
        switch(l2cap_state) {
                case PS3_ENABLE_SIXAXIS:
                        if(xTaskGetTickCount() - timer > 1000) { // loop 1 second before sending the command
                                memset(l2capinbuf, 0, BULK_MAXPKTSIZE); // Reset l2cap in buffer as it sometimes read it as a button has been pressed
                                for(uint8_t i = 15; i < 19; i++)
                                        l2capinbuf[i] = 0x7F; // Set the analog joystick values to center position
                                enable_sixaxis();
                                l2cap_state = TURN_ON_LED;
                                timer = xTaskGetTickCount();
                        }
                        break;

                case TURN_ON_LED:
                        if(xTaskGetTickCount() - timer > 1000) { // loop 1 second before sending the command
                                if(remote_name_first == 'P') { // First letter in PLAYSTATION(R)3 Controller ('P')
                                		ESP_LOGI(tag, "Dualshock 3 Controller Enabled");
                                        PS3Connected = true;
                                } else if(remote_name_first == 'N') { // First letter in Navigation Controller ('N')
                                		ESP_LOGI(tag, "Navigation Controller Enabled");
                                        PS3NavigationConnected = true;
                                } else if(remote_name_first == 'M') { // First letter in Motion Controller ('M')
                                        timer = xTaskGetTickCount();
                                        ESP_LOGI(tag, "Motion Controller Enabled");
                                        PS3MoveConnected = true;
                                }
                                ButtonState = 0; // Clear all values
                                OldButtonState = 0;
                                ButtonClickState = 0;

                                onInit(); // Turn on the LED on the controller
                                l2cap_state = L2CAP_DONE;
                        }
                        break;

                case L2CAP_DONE:
                        if(PS3MoveConnected) { // The Bulb and rumble values, has to be send at approximately every 5th second for it to stay on
                                if(xTaskGetTickCount() - timer > 4000) { // Send at least every 4th second
                                        HIDMove_Command(HIDMoveBuffer, HID_BUFFERSIZE); // The Bulb and rumble values, has to be written again and again, for it to stay turned on
                                        timer = xTaskGetTickCount();
                                }
                        }
                        break;
        }
}

/************************************************************/
/*                    HID Commands                          */
/************************************************************/

// Playstation Sixaxis Dualshock and Navigation Controller commands

void PS3BT::HID_Command(uint8_t* data, uint8_t nbytes) {
		if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
				uint16_t i;
				char buf[nbytes*3];
				for (i = 0; i < nbytes; i++) {
					sprintf((buf + i*3), "%02x", data[i]);
					buf[i*3 + 2] = ' ';
				}
				buf[(i - 1)*3 + 2] = '\0';
				ESP_LOGD(tag, "HID COMMAND ! %s", buf);
		}
        if(xTaskGetTickCount() - timerHID <= 150) // Check if is has been more than 150ms since last command
        		vTaskDelay((150 - (xTaskGetTickCount() - timerHID)) / portTICK_PERIOD_MS); // There have to be a delay between commands
        pBtd->L2CAP_Command(hci_handle, data, nbytes, control_scid[0], control_scid[1]); // Both the Navigation and Dualshock controller sends data via the control channel
        timerHID = xTaskGetTickCount();
}

void PS3BT::setAllOff() {
        HIDBuffer[3] = 0x00; // Rumble bytes
        HIDBuffer[4] = 0x00;
        HIDBuffer[5] = 0x00;
        HIDBuffer[6] = 0x00;

        HIDBuffer[11] = 0x00; // LED byte

        HID_Command(HIDBuffer, HID_BUFFERSIZE);
}

void PS3BT::setRumbleOff() {
        uint8_t rumbleBuf[HID_BUFFERSIZE];
        memcpy(rumbleBuf, HIDBuffer, HID_BUFFERSIZE);
        rumbleBuf[3] = 0x00;
        rumbleBuf[4] = 0x00;
        rumbleBuf[5] = 0x00;
        rumbleBuf[6] = 0x00;
        HID_Command(rumbleBuf, HID_BUFFERSIZE);
}

void PS3BT::setRumbleOn(RumbleEnum mode) {
        uint8_t power[2] = {0xff, 0x00}; // Defaults to RumbleLow
        if(mode == RumbleHigh) {
                power[0] = 0x00;
                power[1] = 0xff;
        }
        setRumbleOn(0xfe, power[0], 0xfe, power[1]);
}

void PS3BT::setRumbleOn(uint8_t rightDuration, uint8_t rightPower, uint8_t leftDuration, uint8_t leftPower) {
        uint8_t rumbleBuf[HID_BUFFERSIZE];
        memcpy(rumbleBuf, HIDBuffer, HID_BUFFERSIZE);
        rumbleBuf[3] = rightDuration;
        rumbleBuf[4] = rightPower;
        rumbleBuf[5] = leftDuration;
        rumbleBuf[6] = leftPower;
        HID_Command(rumbleBuf, HID_BUFFERSIZE);
}

void PS3BT::setLedRaw(uint8_t value) {
        HIDBuffer[11] = value << 1;
        HID_Command(HIDBuffer, HID_BUFFERSIZE);
}

void PS3BT::setLedOff(LEDEnum a) {
        HIDBuffer[11] &= ~((uint8_t)((pgm_read_byte(&PS3_LEDS[(uint8_t)a]) & 0x0f) << 1));
        HID_Command(HIDBuffer, HID_BUFFERSIZE);
}

void PS3BT::setLedOn(LEDEnum a) {
        if(a == OFF)
                setLedRaw(0);
        else {
                HIDBuffer[11] |= (uint8_t)((pgm_read_byte(&PS3_LEDS[(uint8_t)a]) & 0x0f) << 1);
                HID_Command(HIDBuffer, HID_BUFFERSIZE);
        }
}

void PS3BT::setLedToggle(LEDEnum a) {
        HIDBuffer[11] ^= (uint8_t)((pgm_read_byte(&PS3_LEDS[(uint8_t)a]) & 0x0f) << 1);
        HID_Command(HIDBuffer, HID_BUFFERSIZE);
}

void PS3BT::enable_sixaxis() { // Command used to enable the Dualshock 3 and Navigation controller to send data via Bluetooth
        uint8_t cmd_buf[6];
        cmd_buf[0] = 0x53; // HID BT Set_report (0x50) | Report Type (Feature 0x03)
        cmd_buf[1] = 0xF4; // Report ID
        cmd_buf[2] = 0x42; // Special PS3 Controller enable commands
        cmd_buf[3] = 0x03;
        cmd_buf[4] = 0x00;
        cmd_buf[5] = 0x00;

        HID_Command(cmd_buf, 6);
}

// Playstation Move Controller commands

void PS3BT::HIDMove_Command(uint8_t* data, uint8_t nbytes) {
        if(xTaskGetTickCount() - timerHID <= 150)// Check if is has been less than 150ms since last command
        		vTaskDelay((150 - (xTaskGetTickCount() - timerHID)) / portTICK_PERIOD_MS); // There have to be a delay between commands
        pBtd->L2CAP_Command(hci_handle, data, nbytes, interrupt_scid[0], interrupt_scid[1]); // The Move controller sends it's data via the intterrupt channel
        timerHID = xTaskGetTickCount();
}

void PS3BT::moveSetBulb(uint8_t r, uint8_t g, uint8_t b) { // Use this to set the Color using RGB values
        // Set the Bulb's values into the write buffer
        HIDMoveBuffer[3] = r;
        HIDMoveBuffer[4] = g;
        HIDMoveBuffer[5] = b;

        HIDMove_Command(HIDMoveBuffer, HID_BUFFERSIZE);
}

void PS3BT::moveSetBulb(ColorsEnum color) { // Use this to set the Color using the predefined colors in enum
        moveSetBulb((uint8_t)(color >> 16), (uint8_t)(color >> 8), (uint8_t)(color));
}

void PS3BT::moveSetRumble(uint8_t rumble) {
#ifdef DEBUG_USB_HOST
        if(rumble < 64 && rumble != 0) // The rumble value has to at least 64, or approximately 25% (64/255*100)
                Notify(PSTR("\r\nThe rumble value has to at least 64, or approximately 25%"), 0x80);
#endif
        // Set the rumble value into the write buffer
        HIDMoveBuffer[7] = rumble;

        HIDMove_Command(HIDMoveBuffer, HID_BUFFERSIZE);
}

void PS3BT::onInit() {
        if(pFuncOnInit)
                pFuncOnInit(); // Call the user function
        else {
                if(PS3MoveConnected)
                        moveSetBulb(Red);
                else // Dualshock 3 or Navigation controller
                        setLedOn(static_cast<LEDEnum>(LED1));
        }
}

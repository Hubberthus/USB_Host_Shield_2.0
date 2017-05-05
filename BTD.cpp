/* Copyright (C) 2017 Norbert Fekete. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Ported version for ESP32.
 USB dongle was virtualized.

 Original contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "BTD.h"
// To enable serial debugging see "settings.h"
//#define EXTRADEBUG // Uncomment to get even more debugging data

/*const uint8_t BTD::BTD_CONTROL_PIPE = 0;
const uint8_t BTD::BTD_EVENT_PIPE = 1;
const uint8_t BTD::BTD_DATAIN_PIPE = 2;
const uint8_t BTD::BTD_DATAOUT_PIPE = 3;*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include "esp_log.h"

static const char *tag = "BTD";

BTD *activeBTD;

static xQueueHandle acl_data_queue = NULL;

static void bt_controller_rcv_pkt_ready(void) {}

static int bt_host_rcv_pkt(uint8_t *data, uint16_t len);

static esp_vhci_host_callback_t vbtd_host_cb = {
	bt_controller_rcv_pkt_ready,
	bt_host_rcv_pkt
};

static void BTD_HCI_task(void* params);
static void BTD_ACL_task(void* params);
static void BTD_services_task(void* params);

BTD::BTD(/*USB *p*/) :
connectToWii(false),
pairWithWii(false),
connectToHIDDevice(false),
pairWithHIDDevice(false)/*,
pUsb(p), // Pointer to USB class instance - mandatory
bAddress(0), // Device address - mandatory
bNumEP(1), // If config descriptor needs to be parsed
qNextPollTime(0), // Reset NextPollTime
pollInterval(0),
bPollEnable(false) // Don't start polling before dongle is connected*/
{
        for(uint8_t i = 0; i < BTD_NUM_SERVICES; i++)
                btService[i] = NULL;

        Initialize(); // Set all variables, endpoint structs etc. to default values

/*        if(pUsb) // Register in USB subsystem
                pUsb->RegisterDeviceClass(this); // Set devConfig[] entry*/
}

/*uint8_t BTD::ConfigureDevice(uint8_t parent, uint8_t port, bool lowspeed) {
        const uint8_t constBufSize = sizeof (USB_DEVICE_DESCRIPTOR);
        uint8_t buf[constBufSize];
        USB_DEVICE_DESCRIPTOR * udd = reinterpret_cast<USB_DEVICE_DESCRIPTOR*>(buf);
        uint8_t rcode;
        UsbDevice *p = NULL;
        EpInfo *oldep_ptr = NULL;

        Initialize(); // Set all variables, endpoint structs etc. to default values

        AddressPool &addrPool = pUsb->GetAddressPool(); // Get memory address of USB device address pool
#ifdef EXTRADEBUG
        Notify(PSTR("\r\nBTD ConfigureDevice"), 0x80);
#endif

        if(bAddress) { // Check if address has already been assigned to an instance
#ifdef DEBUG_USB_HOST
                Notify(PSTR("\r\nAddress in use"), 0x80);
#endif
                return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
        }

        p = addrPool.GetUsbDevicePtr(0); // Get pointer to pseudo device with address 0 assigned
        if(!p) {
#ifdef DEBUG_USB_HOST
                Notify(PSTR("\r\nAddress not found"), 0x80);
#endif
                return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
        }

        if(!p->epinfo) {
#ifdef DEBUG_USB_HOST
                Notify(PSTR("\r\nepinfo is null"), 0x80);
#endif
                return USB_ERROR_EPINFO_IS_NULL;
        }

        oldep_ptr = p->epinfo; // Save old pointer to EP_RECORD of address 0
        p->epinfo = epInfo; // Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
        p->lowspeed = lowspeed;
        rcode = pUsb->getDevDescr(0, 0, constBufSize, (uint8_t*)buf); // Get device descriptor - addr, ep, nbytes, data

        p->epinfo = oldep_ptr; // Restore p->epinfo

        if(rcode)
                goto FailGetDevDescr;

        bAddress = addrPool.AllocAddress(parent, false, port); // Allocate new address according to device class

        if(!bAddress) {
#ifdef DEBUG_USB_HOST
                Notify(PSTR("\r\nOut of address space"), 0x80);
#endif
                return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;
        }

        if (udd->bDeviceClass == 0x09) // Some dongles have an USB hub inside
                goto FailHub;

        epInfo[0].maxPktSize = udd->bMaxPacketSize0; // Extract Max Packet Size from device descriptor
        epInfo[1].epAddr = udd->bNumConfigurations; // Steal and abuse from epInfo structure to save memory

        VID = udd->idVendor;
        PID = udd->idProduct;

        return USB_ERROR_CONFIG_REQUIRES_ADDITIONAL_RESET;

FailHub:
#ifdef DEBUG_USB_HOST
        Notify(PSTR("\r\nPlease create a hub instance in your code: \"USBHub Hub1(&Usb);\""), 0x80);
#endif
        pUsb->setAddr(bAddress, 0, 0); // Reset address
        rcode = USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;
        Release();
        return rcode;

FailGetDevDescr:
#ifdef DEBUG_USB_HOST
        NotifyFailGetDevDescr(rcode);
#endif
        if(rcode != hrJERR)
                rcode = USB_ERROR_FailGetDevDescr;
        Release();
        return rcode;
};*/

/*uint8_t BTD::Init(uint8_t parent, uint8_t port, bool lowspeed) {
        uint8_t rcode;
        uint8_t num_of_conf = epInfo[1].epAddr; // Number of configurations
        epInfo[1].epAddr = 0;

        AddressPool &addrPool = pUsb->GetAddressPool();
#ifdef EXTRADEBUG
        Notify(PSTR("\r\nBTD Init"), 0x80);
#endif
        UsbDevice *p = addrPool.GetUsbDevicePtr(bAddress); // Get pointer to assigned address record

        if(!p) {
#ifdef DEBUG_USB_HOST
                Notify(PSTR("\r\nAddress not found"), 0x80);
#endif
                return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
        }

        delay(300); // Assign new address to the device

        rcode = pUsb->setAddr(0, 0, bAddress); // Assign new address to the device
        if(rcode) {
#ifdef DEBUG_USB_HOST
                Notify(PSTR("\r\nsetAddr: "), 0x80);
                D_PrintHex<uint8_t > (rcode, 0x80);
#endif
                p->lowspeed = false;
                goto Fail;
        }
#ifdef EXTRADEBUG
        Notify(PSTR("\r\nAddr: "), 0x80);
        D_PrintHex<uint8_t > (bAddress, 0x80);
#endif

        p->lowspeed = false;

        p = addrPool.GetUsbDevicePtr(bAddress); // Get pointer to assigned address record
        if(!p) {
#ifdef DEBUG_USB_HOST
                Notify(PSTR("\r\nAddress not found"), 0x80);
#endif
                return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
        }

        p->lowspeed = lowspeed;

        rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo); // Assign epInfo to epinfo pointer - only EP0 is known
        if(rcode)
                goto FailSetDevTblEntry;

        if(VID == PS3_VID && (PID == PS3_PID || PID == PS3NAVIGATION_PID || PID == PS3MOVE_PID)) {
                delay(100);
                rcode = pUsb->setConf(bAddress, epInfo[ BTD_CONTROL_PIPE ].epAddr, 1); // We only need the Control endpoint, so we don't have to initialize the other endpoints of device
                if(rcode)
                        goto FailSetConfDescr;

#ifdef DEBUG_USB_HOST
                if(PID == PS3_PID || PID == PS3NAVIGATION_PID) {
                        if(PID == PS3_PID)
                                Notify(PSTR("\r\nDualshock 3 Controller Connected"), 0x80);
                        else // It must be a navigation controller
                                Notify(PSTR("\r\nNavigation Controller Connected"), 0x80);
                } else // It must be a Motion controller
                        Notify(PSTR("\r\nMotion Controller Connected"), 0x80);
#endif

                if(my_bdaddr[0] == 0x00 && my_bdaddr[1] == 0x00 && my_bdaddr[2] == 0x00 && my_bdaddr[3] == 0x00 && my_bdaddr[4] == 0x00 && my_bdaddr[5] == 0x00) {
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nPlease plug in the dongle before trying to pair with the PS3 Controller\r\nor set the Bluetooth address in the constructor of the PS3BT class"), 0x80);
#endif
                } else {
                        if(PID == PS3_PID || PID == PS3NAVIGATION_PID)
                                setBdaddr(my_bdaddr); // Set internal Bluetooth address
                        else
                                setMoveBdaddr(my_bdaddr); // Set internal Bluetooth address
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nBluetooth Address was set to: "), 0x80);
                        for(int8_t i = 5; i > 0; i--) {
                                D_PrintHex<uint8_t > (my_bdaddr[i], 0x80);
                                Notify(PSTR(":"), 0x80);
                        }
                        D_PrintHex<uint8_t > (my_bdaddr[0], 0x80);
#endif
                }

                pUsb->setConf(bAddress, epInfo[ BTD_CONTROL_PIPE ].epAddr, 0); // Reset configuration value
                pUsb->setAddr(bAddress, 0, 0); // Reset address
                Release(); // Release device
                return USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED; // Return
        } else {
                // Check if attached device is a Bluetooth dongle and fill endpoint data structure
                // First interface in the configuration must have Bluetooth assigned Class/Subclass/Protocol
                // And 3 endpoints - interrupt-IN, bulk-IN, bulk-OUT, not necessarily in this order
                for(uint8_t i = 0; i < num_of_conf; i++) {
                        if(VID == IOGEAR_GBU521_VID && PID == IOGEAR_GBU521_PID) {
                                ConfigDescParser<USB_CLASS_VENDOR_SPECIFIC, WI_SUBCLASS_RF, WI_PROTOCOL_BT, CP_MASK_COMPARE_ALL> confDescrParser(this); // Needed for the IOGEAR GBU521
                                rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);
                        } else {
                                ConfigDescParser<USB_CLASS_WIRELESS_CTRL, WI_SUBCLASS_RF, WI_PROTOCOL_BT, CP_MASK_COMPARE_ALL> confDescrParser(this);
                                rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);
                        }
                        if(rcode) // Check error code
                                goto FailGetConfDescr;
                        if(bNumEP >= BTD_MAX_ENDPOINTS) // All endpoints extracted
                                break;
                }

                if(bNumEP < BTD_MAX_ENDPOINTS)
                        goto FailUnknownDevice;

                // Assign epInfo to epinfo pointer - this time all 3 endpoins
                rcode = pUsb->setEpInfoEntry(bAddress, bNumEP, epInfo);
                if(rcode)
                        goto FailSetDevTblEntry;

                // Set Configuration Value
                rcode = pUsb->setConf(bAddress, epInfo[ BTD_CONTROL_PIPE ].epAddr, bConfNum);
                if(rcode)
                        goto FailSetConfDescr;

                hci_num_reset_loops = 100; // only loop 100 times before trying to send the hci reset command
                hci_counter = 0;
                hci_state = HCI_INIT_STATE;
                waitingForConnection = false;
                bPollEnable = true;

#ifdef DEBUG_USB_HOST
                Notify(PSTR("\r\nBluetooth Dongle Initialized"), 0x80);
#endif
        }
        return 0; // Successful configuration

        /-* Diagnostic messages *-/
FailSetDevTblEntry:
#ifdef DEBUG_USB_HOST
        NotifyFailSetDevTblEntry();
        goto Fail;
#endif

FailGetConfDescr:
#ifdef DEBUG_USB_HOST
        NotifyFailGetConfDescr();
        goto Fail;
#endif

FailSetConfDescr:
#ifdef DEBUG_USB_HOST
        NotifyFailSetConfDescr();
#endif
        goto Fail;

FailUnknownDevice:
#ifdef DEBUG_USB_HOST
        NotifyFailUnknownDevice(VID, PID);
#endif
        pUsb->setAddr(bAddress, 0, 0); // Reset address
        rcode = USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;
Fail:
#ifdef DEBUG_USB_HOST
        Notify(PSTR("\r\nBTD Init Failed, error code: "), 0x80);
        NotifyFail(rcode);
#endif
        Release();
        return rcode;
}*/

void test(void*) {
	printf("!\n");
}

void BTD::Initialize() {
        uint8_t i;
        /*for(i = 0; i < BTD_MAX_ENDPOINTS; i++) {
                epInfo[i].epAddr = 0;
                epInfo[i].maxPktSize = (i) ? 0 : 8;
                epInfo[i].bmSndToggle = 0;
                epInfo[i].bmRcvToggle = 0;
                epInfo[i].bmNakPower = (i) ? USB_NAK_NOWAIT : USB_NAK_MAX_POWER;
        }*/
        for(i = 0; i < BTD_NUM_SERVICES; i++) {
                if(btService[i])
                        btService[i]->Reset(); // Reset all Bluetooth services
        }

        connectToWii = false;
        incomingWii = false;
        connectToHIDDevice = false;
        incomingHIDDevice = false;
        incomingPS4 = false;
        /*bAddress = 0; // Clear device address
        bNumEP = 1; // Must have to be reset to 1
        qNextPollTime = 0; // Reset next poll time
        pollInterval = 0;
        bPollEnable = false; // Don't start polling before dongle is connected*/

        hci_num_reset_loops = 100; // only loop 100 times before trying to send the hci reset command
		hci_counter = 0;
		hci_state = HCI_INIT_STATE;
		waitingForConnection = false;

        ESP_LOGI(tag, "Starting BTD_HCI_task");
        activeBTD = this;

        acl_data_queue = xQueueCreate(10, sizeof(uint8_t));

        xTaskCreate(BTD_HCI_task, "BTD_HCI_task", 2048, NULL, 10, NULL);
        xTaskCreate(BTD_ACL_task, "BTD_ACL_task", 2048, NULL, 10, NULL);
        xTaskCreate(BTD_services_task, "BTD_services_task", 2048, NULL, 10, NULL);
}

/* Extracts interrupt-IN, bulk-IN, bulk-OUT endpoint information from config descriptor */
/*void BTD::EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto, const USB_ENDPOINT_DESCRIPTOR *pep) {
        //ErrorMessage<uint8_t>(PSTR("Conf.Val"),conf);
        //ErrorMessage<uint8_t>(PSTR("Iface Num"),iface);
        //ErrorMessage<uint8_t>(PSTR("Alt.Set"),alt);

        if(alt) // Wrong interface - by BT spec, no alt setting
                return;

        bConfNum = conf;
        uint8_t index;

        if((pep->bmAttributes & 0x03) == 3 && (pep->bEndpointAddress & 0x80) == 0x80) { // Interrupt In endpoint found
                index = BTD_EVENT_PIPE;
                epInfo[index].bmNakPower = USB_NAK_NOWAIT;
        } else {
                if((pep->bmAttributes & 0x02) == 2) // Bulk endpoint found
                        index = ((pep->bEndpointAddress & 0x80) == 0x80) ? BTD_DATAIN_PIPE : BTD_DATAOUT_PIPE;
                else
                        return;
        }

        // Fill the rest of endpoint data structure
        epInfo[index].epAddr = (pep->bEndpointAddress & 0x0F);
        epInfo[index].maxPktSize = (uint8_t)pep->wMaxPacketSize;
#ifdef EXTRADEBUG
        PrintEndpointDescriptor(pep);
#endif
        if(pollInterval < pep->bInterval) // Set the polling interval as the largest polling interval obtained from endpoints
                pollInterval = pep->bInterval;
        bNumEP++;
}*/

/*void BTD::PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr) {
#ifdef EXTRADEBUG
        Notify(PSTR("\r\nEndpoint descriptor:"), 0x80);
        Notify(PSTR("\r\nLength:\t\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bLength, 0x80);
        Notify(PSTR("\r\nType:\t\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bDescriptorType, 0x80);
        Notify(PSTR("\r\nAddress:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bEndpointAddress, 0x80);
        Notify(PSTR("\r\nAttributes:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bmAttributes, 0x80);
        Notify(PSTR("\r\nMaxPktSize:\t"), 0x80);
        D_PrintHex<uint16_t > (ep_ptr->wMaxPacketSize, 0x80);
        Notify(PSTR("\r\nPoll Intrv:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bInterval, 0x80);
#endif
}*/

/* Performs a cleanup after failed Init() attempt */
/*uint8_t BTD::Release() {
        Initialize(); // Set all variables, endpoint structs etc. to default values
        pUsb->GetAddressPool().FreeAddress(bAddress);
        return 0;
}*/

/*uint8_t BTD::Poll() {
        if(!bPollEnable)
                return 0;
        if((long)(millis() - qNextPollTime) >= 0L) { // Don't poll if shorter than polling interval
                qNextPollTime = millis() + pollInterval; // Set new poll time
                HCI_event_task(); // Poll the HCI event pipe
                HCI_task(); // HCI state machine
                ACL_event_task(); // Poll the ACL input pipe too
        }
        return 0;
}*/

void BTD::disconnect() {
        for(uint8_t i = 0; i < BTD_NUM_SERVICES; i++)
                if(btService[i])
                        btService[i]->disconnect();
};

static int bt_host_rcv_pkt(uint8_t *data, uint16_t len)
{
	if (len == 0) {
		return 0;
	}

	if (data[0] == 0x04) { // HCI EVENT

		for (uint16_t i = 0; i < len - 1; i++) {
				activeBTD->hcibuf[i] = data[i+1];
		}

		activeBTD->HCI_event_task(len-1);
	} else if (data[0] == 0x02) { // HCI DATA

		for (uint16_t i = 0; i < len - 1; i++) {
			activeBTD->l2capinbuf[i] = data[i+1];
		}

		activeBTD->ACL_event_task(len-1);
	} else {
		ESP_LOGE(tag, "Unknown packet type: 0x%0xx", data[0]);
	}

	return 0;
}

void BTD::HCI_event_task(uint16_t length) {
		if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
				uint16_t i;
				char buf[length*3];
				for (i = 0; i < length; i++) {
					sprintf((buf + i*3), "%02x", hcibuf[i]);
					buf[i*3 + 2] = ' ';
				}
				buf[(i - 1)*3 + 2] = '\0';
				ESP_LOGD(tag, "HCI EVENT < %s", buf);
		}
		switch(hcibuf[0]) { // Switch on event type
				case EV_COMMAND_COMPLETE:
						if(!hcibuf[5]) { // Check if command succeeded
								hci_set_flag(HCI_FLAG_CMD_COMPLETE); // Set command complete flag
								if((hcibuf[3] == 0x01) && (hcibuf[4] == 0x10)) { // Parameters from read local version information
										hci_version = hcibuf[6]; // Used to check if it supports 2.0+EDR - see http://www.bluetooth.org/Technical/AssignedNumbers/hci.htm
										hci_set_flag(HCI_FLAG_READ_VERSION);
								} else if((hcibuf[3] == 0x09) && (hcibuf[4] == 0x10)) { // Parameters from read local bluetooth address
										for(uint8_t i = 0; i < 6; i++)
												my_bdaddr[i] = hcibuf[6 + i];
										hci_set_flag(HCI_FLAG_READ_BDADDR);
								}
						}
						break;

				case EV_COMMAND_STATUS:
						if(hcibuf[2]) { // Show status on serial if not OK
								ESP_LOGI(tag, "HCI Command Failed: 0x%x", hcibuf[2]);
						}
						break;

				case EV_INQUIRY_COMPLETE:
						if(inquiry_counter >= 5 && (pairWithWii || pairWithHIDDevice)) {
								inquiry_counter = 0;
								if(pairWithWii) {
										ESP_LOGI(tag, "Couldn't find Wiimote");
								} else {
										ESP_LOGI(tag, "Couldn't find HID device");
								}
								connectToWii = false;
								pairWithWii = false;
								connectToHIDDevice = false;
								pairWithHIDDevice = false;
								hci_state = HCI_SCANNING_STATE;
						}
						inquiry_counter++;
						break;

				case EV_INQUIRY_RESULT:
						if(hcibuf[2]) { // Check that there is more than zero responses
								ESP_LOGI(tag, "Number of responses: %d", hcibuf[2]);
								for(uint8_t i = 0; i < hcibuf[2]; i++) {
										uint8_t offset = 8 * hcibuf[2] + 3 * i;

										for(uint8_t j = 0; j < 3; j++)
												classOfDevice[j] = hcibuf[j + 4 + offset];

										ESP_LOGI(tag, "Class of device: 0x%x 0x%x 0x%x", classOfDevice[2], classOfDevice[1], classOfDevice[0]);

										if(pairWithWii && classOfDevice[2] == 0x00 && (classOfDevice[1] & 0x05) && (classOfDevice[0] & 0x0C)) { // See http://wiibrew.org/wiki/Wiimote#SDP_information
												checkRemoteName = true; // Check remote name to distinguish between the different controllers

												for(uint8_t j = 0; j < 6; j++)
														disc_bdaddr[j] = hcibuf[j + 3 + 6 * i];

												hci_set_flag(HCI_FLAG_DEVICE_FOUND);
												break;
										} else if(pairWithHIDDevice && (classOfDevice[1] & 0x05) && (classOfDevice[0] & 0xC8)) { // Check if it is a mouse, keyboard or a gamepad - see: http://bluetooth-pentest.narod.ru/software/bluetooth_class_of_device-service_generator.html

												if(classOfDevice[0] & 0x80)
														ESP_LOGI(tag, "Mouse found");
												if(classOfDevice[0] & 0x40)
														ESP_LOGI(tag, "Keyboard found");
												if(classOfDevice[0] & 0x08)
														ESP_LOGI(tag, "Gamepad found");

												for(uint8_t j = 0; j < 6; j++)
														disc_bdaddr[j] = hcibuf[j + 3 + 6 * i];

												hci_set_flag(HCI_FLAG_DEVICE_FOUND);
												break;
										}
								}
						}
						break;

				case EV_CONNECT_COMPLETE:
						hci_set_flag(HCI_FLAG_CONNECT_EVENT);
						if(!hcibuf[2]) { // Check if connected OK
								ESP_LOGI(tag, "Connection established");
								hci_handle = hcibuf[3] | ((hcibuf[4] & 0x0F) << 8); // Store the handle for the ACL connection
								//hci_write_automatic_flush_timeout();
								hci_set_flag(HCI_FLAG_CONNECT_COMPLETE); // Set connection complete flag
						} else {
								hci_state = HCI_CHECK_DEVICE_SERVICE;
								ESP_LOGI(tag, "Connection Failed: 0x%x", hcibuf[2]);
						}
						break;

				case EV_DISCONNECT_COMPLETE:
						if(!hcibuf[2]) { // Check if disconnected OK
								hci_set_flag(HCI_FLAG_DISCONNECT_COMPLETE); // Set disconnect command complete flag
								hci_clear_flag(HCI_FLAG_CONNECT_COMPLETE); // Clear connection complete flag
						}
						break;

				case EV_REMOTE_NAME_COMPLETE:
						if(!hcibuf[2]) { // Check if reading is OK
								uint8_t i;
								size_t name_length = (sizeof (remote_name) < sizeof (hcibuf) - 9 ? sizeof (remote_name) : sizeof (hcibuf) - 9);
								for(i = 0; i < name_length; i++) {
										remote_name[i] = hcibuf[9 + i];
										if(remote_name[i] == '\0') // End of string
												break;
								}
								remote_name[i] = '\0'; // Make sure remote name is always null terminated
								hci_set_flag(HCI_FLAG_REMOTE_NAME_COMPLETE);
						}
						break;

				case EV_INCOMING_CONNECT:
						for(uint8_t i = 0; i < 6; i++)
								disc_bdaddr[i] = hcibuf[i + 2];

						for(uint8_t i = 0; i < 3; i++)
								classOfDevice[i] = hcibuf[i + 8];

						if((classOfDevice[1] & 0x05) && (classOfDevice[0] & 0xC8)) { // Check if it is a mouse, keyboard or a gamepad
								if(classOfDevice[0] & 0x80)
										ESP_LOGI(tag, "Mouse is connecting");
								if(classOfDevice[0] & 0x40)
										ESP_LOGI(tag, "Keyboard is connecting");
								if(classOfDevice[0] & 0x08)
										ESP_LOGI(tag, "Gamepad is connecting");
								incomingHIDDevice = true;
						}

						ESP_LOGI(tag, "Class of device: 0x%x 0x%x 0x%x", classOfDevice[2], classOfDevice[1], classOfDevice[0]);
						hci_set_flag(HCI_FLAG_INCOMING_REQUEST);
						break;

				case EV_PIN_CODE_REQUEST:
						if(pairWithWii) {
								ESP_LOGI(tag, "Pairing with Wiimote");
								hci_pin_code_request_reply();
						} else if(btdPin != NULL) {
								ESP_LOGI(tag, "Bluetooth pin is set too: %s", btdPin);
								hci_pin_code_request_reply();
						} else {
								ESP_LOGI(tag, "No pin was set");
								hci_pin_code_negative_request_reply();
						}
						break;

				case EV_LINK_KEY_REQUEST:
						ESP_LOGI(tag, "Received Key Request");
						hci_link_key_request_negative_reply();
						break;

				case EV_AUTHENTICATION_COMPLETE:
						if(!hcibuf[2]) { // Check if pairing was successful
								if(pairWithWii && !connectToWii) {
										ESP_LOGI(tag, "Pairing successful with Wiimote");
										connectToWii = true; // Used to indicate to the Wii service, that it should connect to this device
								} else if(pairWithHIDDevice && !connectToHIDDevice) {
										ESP_LOGI(tag, "Pairing successful with HID device");
										connectToHIDDevice = true; // Used to indicate to the BTHID service, that it should connect to this device
								}
						} else {
								ESP_LOGI(tag, "Pairing Failed: 0x%x", hcibuf[2]);
								hci_disconnect(hci_handle);
								hci_state = HCI_DISCONNECT_STATE;
						}
						break;
						/* We will just ignore the following events */
				case EV_READ_REMOTE_SUPPORT_FEATURES:
						if(!hcibuf[2]) { // Check if command was successful
								uint16_t read_handle = hcibuf[3] | ((hcibuf[4] & 0x0F) << 8);
								if (read_handle == hci_handle) { // Check if handle is correct
									ESP_LOGI(tag, "Remote supported features read");
								} else {
									ESP_LOGI(tag, "Invalid HCI handle: 0x%x", read_handle);
								}
						}
						break;
				case EV_NUM_COMPLETE_PKT:
				case EV_ROLE_CHANGED:
				case EV_PAGE_SCAN_REP_MODE:
				case EV_LOOPBACK_COMMAND:
				case EV_DATA_BUFFER_OVERFLOW:
				case EV_CHANGE_CONNECTION_LINK:
				case EV_MAX_SLOTS_CHANGE:
				case EV_QOS_SETUP_COMPLETE:
				case EV_LINK_KEY_NOTIFICATION:
				case EV_ENCRYPTION_CHANGE:
				case EV_READ_REMOTE_VERSION_INFORMATION_COMPLETE:
						break;
				default:
						if(hcibuf[0] != 0x00) {
								ESP_LOGI(tag, "Unmanaged HCI Event: 0x%x", hcibuf[0]);
						}
						break;
		}
}

static void BTD_HCI_task(void* params) {

	esp_vhci_host_register_callback(&vbtd_host_cb);

	ESP_LOGI(tag, "Callbacks registered, starting up state-machine task");

	for(;;) {

		activeBTD->HCI_task();

        vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

/* Poll Bluetooth and print result */
void BTD::HCI_task() {
        switch(hci_state) {
                case HCI_INIT_STATE:
                        hci_counter++;
                        if(hci_counter > hci_num_reset_loops) { // wait until we have looped x times to clear any old events
                                hci_reset();
                                hci_state = HCI_RESET_STATE;
                                hci_counter = 0;
                        }
                        break;

                case HCI_RESET_STATE:
                        hci_counter++;
                        if(hci_check_flag(HCI_FLAG_CMD_COMPLETE)) {
                                hci_counter = 0;
                                ESP_LOGI(tag, "HCI Reset complete");
                                hci_state = HCI_CLASS_STATE;
                                hci_write_class_of_device();
                        } else if(hci_counter > hci_num_reset_loops) {
                                hci_num_reset_loops *= 10;
                                if(hci_num_reset_loops > 2000)
                                        hci_num_reset_loops = 2000;
                                ESP_LOGI(tag, "No response to HCI Reset");
                                hci_state = HCI_INIT_STATE;
                                hci_counter = 0;
                        }
                        break;

                case HCI_CLASS_STATE:
                        if(hci_check_flag(HCI_FLAG_CMD_COMPLETE)) {
                        		ESP_LOGI(tag, "Write class of device");
                                hci_state = HCI_BDADDR_STATE;
                                hci_read_bdaddr();
                        }
                        break;

                case HCI_BDADDR_STATE:
                        if(hci_check_flag(HCI_FLAG_READ_BDADDR)) {
                        		ESP_LOGI(tag, "Local Bluetooth Address: %x:%x:%x:%x:%x:%x", my_bdaddr[5], my_bdaddr[4], my_bdaddr[3], my_bdaddr[2], my_bdaddr[1], my_bdaddr[0]);
                                hci_read_local_version_information();
                                hci_state = HCI_LOCAL_VERSION_STATE;
                        }
                        break;

                case HCI_LOCAL_VERSION_STATE: // The local version is used by the PS3BT class
                        if(hci_check_flag(HCI_FLAG_READ_VERSION)) {
                                if(btdName != NULL) {
                                        hci_set_local_name(btdName);
                                        hci_state = HCI_SET_NAME_STATE;
                                } else
                                        hci_state = HCI_CHECK_DEVICE_SERVICE;
                        }
                        break;

                case HCI_SET_NAME_STATE:
                        if(hci_check_flag(HCI_FLAG_CMD_COMPLETE)) {
                        		ESP_LOGI(tag, "The name is set to: %s", btdName);
                                hci_state = HCI_CHECK_DEVICE_SERVICE;
                        }
                        break;

                case HCI_CHECK_DEVICE_SERVICE:
                        if(pairWithHIDDevice || pairWithWii) { // Check if it should try to connect to a Wiimote
                                if(pairWithWii) {
                                		ESP_LOGI(tag, "Starting inquiry\nPress 1 & 2 on the Wiimote\nOr press the SYNC button if you are using a Wii U Pro Controller or a Wii Balance Board");
                                } else {
                                		ESP_LOGI(tag, "Please enable discovery of your device");
                                }
                                hci_inquiry();
                                hci_state = HCI_INQUIRY_STATE;
                        } else {
                                hci_state = HCI_SCANNING_STATE; // Don't try to connect to a Wiimote
                        }
                        break;

                case HCI_INQUIRY_STATE:
                        if(hci_check_flag(HCI_FLAG_DEVICE_FOUND)) {
                                hci_inquiry_cancel(); // Stop inquiry
                                if(pairWithWii) {
                                		ESP_LOGI(tag, "Wiimote found");
                                } else {
                                		ESP_LOGI(tag, "HID device found");
                                }
                                ESP_LOGI(tag, "Now just create the instance like so:");
                                if(pairWithWii) {
                                		ESP_LOGI(tag, "WII Wii(&Btd);");
                                } else {
                                		ESP_LOGI(tag, "BTHID bthid(&Btd);");
                                }

                                ESP_LOGI(tag, "And then press any button on the %s", (pairWithWii ? "Wiimote" : "device"));
                                if(checkRemoteName) {
                                        hci_remote_name(); // We need to know the name to distinguish between the Wiimote, the new Wiimote with Motion Plus inside, a Wii U Pro Controller and a Wii Balance Board
                                        hci_state = HCI_REMOTE_NAME_STATE;
                                } else
                                        hci_state = HCI_CONNECT_DEVICE_STATE;
                        }
                        break;

                case HCI_CONNECT_DEVICE_STATE:
                        if(hci_check_flag(HCI_FLAG_CMD_COMPLETE)) {
                                if(pairWithWii) {
                                		ESP_LOGI(tag, "Connecting to Wiimote");
                                } else {
                                		ESP_LOGI(tag, "Connecting to HID device");
                                }
                                checkRemoteName = false;
                                hci_connect();
                                hci_state = HCI_CONNECTED_DEVICE_STATE;
                        }
                        break;

                case HCI_CONNECTED_DEVICE_STATE:
                        if(hci_check_flag(HCI_FLAG_CONNECT_EVENT)) {
                                if(hci_check_flag(HCI_FLAG_CONNECT_COMPLETE)) {
                                        if(pairWithWii) {
                                        		ESP_LOGI(tag, "Connected to Wiimote");
                                        } else {
                                        		ESP_LOGI(tag, "Connected to HID device");
                                        }
                                        hci_authentication_request(); // This will start the pairing with the Wiimote
                                        hci_state = HCI_SCANNING_STATE;
                                } else {
                                		ESP_LOGI(tag, "Trying to connect one more time...");
                                        hci_connect(); // Try to connect one more time
                                }
                        }
                        break;

                case HCI_SCANNING_STATE:
                        if(!connectToWii && !pairWithWii && !connectToHIDDevice && !pairWithHIDDevice) {
                        		ESP_LOGI(tag, "Wait For Incoming Connection Request");
                                hci_write_scan_enable();
                                waitingForConnection = true;
                                hci_state = HCI_CONNECT_IN_STATE;
                        }
                        break;

                case HCI_CONNECT_IN_STATE:
                		if(hci_check_flag(HCI_FLAG_INCOMING_REQUEST/*HCI_FLAG_REMOTE_NAME_COMPLETE*/)) {
                				ESP_LOGI(tag, "Incoming Connection Request");
								if(pairWithWii && checkRemoteName)
										hci_state = HCI_CONNECT_DEVICE_STATE;
								else {
										hci_accept_connection();
										hci_state = HCI_REMOTE_NAME_STATE/*HCI_CONNECTED_STATE*/;
								}
						}
						break;

                case HCI_REMOTE_NAME_STATE:
                		if(hci_check_flag(HCI_FLAG_CONNECT_COMPLETE/*HCI_FLAG_INCOMING_REQUEST*/)) {
								waitingForConnection = false;
								ESP_LOGI(tag, "Connection Request Accepted");
								hci_remote_name();
								hci_state = HCI_CONNECTED_STATE/*HCI_REMOTE_NAME_STATE*/;
						} else if(hci_check_flag(HCI_FLAG_DISCONNECT_COMPLETE))
								hci_state = HCI_DISCONNECT_STATE;
						break;

                case HCI_CONNECTED_STATE:
                        if(hci_check_flag(HCI_FLAG_REMOTE_NAME_COMPLETE/*HCI_FLAG_CONNECT_COMPLETE*/)) {
								ESP_LOGI(tag, "Remote Name: %s", remote_name);
								if(strncmp((const char*)remote_name, "Nintendo", 8) == 0) {
										incomingWii = true;
										motionPlusInside = false;
										wiiUProController = false;
										pairWiiUsingSync = false;
										ESP_LOGI(tag, "Wiimote is connecting");
										if(strncmp((const char*)remote_name, "Nintendo RVL-CNT-01-TR", 22) == 0) {
												ESP_LOGI(tag, " with Motion Plus Inside");
												motionPlusInside = true;
										} else if(strncmp((const char*)remote_name, "Nintendo RVL-CNT-01-UC", 22) == 0) {
												ESP_LOGI(tag, " - Wii U Pro Controller");
												wiiUProController = motionPlusInside = pairWiiUsingSync = true;
										} else if(strncmp((const char*)remote_name, "Nintendo RVL-WBC-01", 19) == 0) {
												ESP_LOGI(tag, " - Wii Balance Board");
												pairWiiUsingSync = true;
										}
								}
								if(classOfDevice[2] == 0 && classOfDevice[1] == 0x25 && classOfDevice[0] == 0x08 && strncmp((const char*)remote_name, "Wireless Controller", 19) == 0) {
										ESP_LOGI(tag, "PS4 controller is connecting");
										incomingPS4 = true;
								}
                        		ESP_LOGI(tag, "Connected to Device: %x:%x:%x:%x:%x:%x", disc_bdaddr[5], disc_bdaddr[4], disc_bdaddr[3], disc_bdaddr[2], disc_bdaddr[1], disc_bdaddr[0]);
                                if(incomingPS4)
                                        connectToHIDDevice = true; // We should always connect to the PS4 controller

                                // Clear these flags for a new connection
                                l2capConnectionClaimed = false;
                                sdpConnectionClaimed = false;
                                rfcommConnectionClaimed = false;

                                hci_event_flag = 0;
                                hci_read_remote_supported_features();
                                hci_state = HCI_DONE_STATE;
                        }
                        break;

                case HCI_DONE_STATE:
                        hci_counter++;
                        if(hci_counter > 1000) { // Wait until we have looped 1000 times to make sure that the L2CAP connection has been started
                                hci_counter = 0;
                                hci_state = HCI_SCANNING_STATE;
                        }
                        break;

                case HCI_DISCONNECT_STATE:
                        if(hci_check_flag(HCI_FLAG_DISCONNECT_COMPLETE)) {
                        		ESP_LOGI(tag, "HCI Disconnected from Device");
                                hci_event_flag = 0; // Clear all flags

                                // Reset all buffers
                                memset(hcibuf, 0, BULK_MAXPKTSIZE);
                                memset(l2capinbuf, 0, BULK_MAXPKTSIZE);

                                connectToWii = incomingWii = pairWithWii = false;
                                connectToHIDDevice = incomingHIDDevice = pairWithHIDDevice = checkRemoteName = false;
                                incomingPS4 = false;

                                hci_state = HCI_SCANNING_STATE;
                        }
                        break;
                default:
                        break;
        }
}

static void BTD_ACL_task(void* params) {

	ESP_LOGI(tag, "Starting up ACL handler task");

	uint8_t l2capinbuf_incoming = 0;

	for(;;) {

		if(xQueueReceive(acl_data_queue, &l2capinbuf_incoming, portMAX_DELAY)) {
				for(uint8_t i = 0; i < BTD_NUM_SERVICES; i++) {
						if(activeBTD->btService[i]) {
								activeBTD->btService[i]->ACLData(activeBTD->l2capinbuf_ring[l2capinbuf_incoming]);
						}
				}
		}
	}
}
static void BTD_services_task(void* params) {

		ESP_LOGI(tag, "Starting up services task");

		for(;;) {

			for(uint8_t i = 0; i < BTD_NUM_SERVICES; i++)
					if(activeBTD->btService[i])
							activeBTD->btService[i]->Run();

			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
}

void BTD::ACL_event_task(uint16_t length) {
		if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
				uint16_t i;
				char buf[length*3];
				for (i = 0; i < length; i++) {
					sprintf((buf + i*3), "%02x", l2capinbuf[i]);
					buf[i*3 + 2] = ' ';
				}
				buf[(i - 1)*3 + 2] = '\0';
				ESP_LOGD(tag, "ACL DATA < %s", buf);
		}

		xQueueSend(acl_data_queue, &l2capinbuf_current, NULL);

		l2capinbuf_current++;
		if (l2capinbuf_current >= MAX_RINGBUFFER) {
			l2capinbuf_current = 0;
		}

		l2capinbuf = l2capinbuf_ring[l2capinbuf_current];
}

/************************************************************/
/*                    HCI Commands                          */
/************************************************************/
void BTD::HCI_Command(uint8_t* data, uint16_t nbytes) {
        hci_clear_flag(HCI_FLAG_CMD_COMPLETE);

        uint8_t buf[1 + nbytes];
		buf[0] = (uint8_t)(0x01); // HCI COMMAND

		for(uint16_t i = 0; i < nbytes; i++)
				buf[1 + i] = data[i];

		while(! esp_vhci_host_check_send_available()) {
				vTaskDelay(10 / portTICK_PERIOD_MS);
		}

		if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
				uint16_t i;
				char log_buf[nbytes*3];
				for (i = 0; i < nbytes; i++) {
					sprintf((log_buf + i*3), "%02x", buf[i + 1]);
					log_buf[i*3 + 2] = ' ';
				}
				log_buf[(i - 1)*3 + 2] = '\0';
				ESP_LOGD(tag, "HCI COMMAND > %s", log_buf);
		}

		esp_vhci_host_send_packet(buf, nbytes + 1);
}

void BTD::hci_reset() {
        hci_event_flag = 0; // Clear all the flags
        hcibuf[0] = 0x03; // HCI OCF = 3
        hcibuf[1] = 0x03 << 2; // HCI OGF = 3
        hcibuf[2] = 0x00;

        HCI_Command(hcibuf, 3);
}

void BTD::hci_write_scan_enable() {
        hci_clear_flag(HCI_FLAG_INCOMING_REQUEST);
        hcibuf[0] = 0x1A; // HCI OCF = 1A
        hcibuf[1] = 0x03 << 2; // HCI OGF = 3
        hcibuf[2] = 0x01; // parameter length = 1
        if(btdName != NULL)
                hcibuf[3] = 0x03; // Inquiry Scan enabled. Page Scan enabled.
        else
                hcibuf[3] = 0x02; // Inquiry Scan disabled. Page Scan enabled.

        HCI_Command(hcibuf, 4);
}

void BTD::hci_write_scan_disable() {
        hcibuf[0] = 0x1A; // HCI OCF = 1A
        hcibuf[1] = 0x03 << 2; // HCI OGF = 3
        hcibuf[2] = 0x01; // parameter length = 1
        hcibuf[3] = 0x00; // Inquiry Scan disabled. Page Scan disabled.

        HCI_Command(hcibuf, 4);
}

void BTD::hci_write_automatic_flush_timeout() {
		hcibuf[0] = 0x28; // HCI OCF = 28
		hcibuf[1] = 0x03 << 2; // HCI OGF = 3
		hcibuf[2] = 0x04; // parameter length = 4
		hcibuf[3] = (uint8_t)(hci_handle & 0xFF); //connection handle - low byte
		hcibuf[4] = (uint8_t)((hci_handle >> 8) & 0x0F); //connection handle - high byte
		hcibuf[5] = 0xFF; // Flush timeout N * 0.625 msec. N=0x00FF
		hcibuf[6] = 0x00;

		HCI_Command(hcibuf, 7);
}

void BTD::hci_read_bdaddr() {
        hci_clear_flag(HCI_FLAG_READ_BDADDR);
        hcibuf[0] = 0x09; // HCI OCF = 9
        hcibuf[1] = 0x04 << 2; // HCI OGF = 4
        hcibuf[2] = 0x00;

        HCI_Command(hcibuf, 3);
}

void BTD::hci_read_local_version_information() {
        hci_clear_flag(HCI_FLAG_READ_VERSION);
        hcibuf[0] = 0x01; // HCI OCF = 1
        hcibuf[1] = 0x04 << 2; // HCI OGF = 4
        hcibuf[2] = 0x00;

        HCI_Command(hcibuf, 3);
}

void BTD::hci_accept_connection() {
        hci_clear_flag(HCI_FLAG_CONNECT_COMPLETE);
        hcibuf[0] = 0x09; // HCI OCF = 9
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x07; // parameter length 7
        hcibuf[3] = disc_bdaddr[0]; // 6 octet bdaddr
        hcibuf[4] = disc_bdaddr[1];
        hcibuf[5] = disc_bdaddr[2];
        hcibuf[6] = disc_bdaddr[3];
        hcibuf[7] = disc_bdaddr[4];
        hcibuf[8] = disc_bdaddr[5];
        hcibuf[9] = 0x01; // Switch role to slave

        HCI_Command(hcibuf, 10);
}

void BTD::hci_remote_name() {
        hci_clear_flag(HCI_FLAG_REMOTE_NAME_COMPLETE);
        hcibuf[0] = 0x19; // HCI OCF = 19
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x0A; // parameter length = 10
        hcibuf[3] = disc_bdaddr[0]; // 6 octet bdaddr
        hcibuf[4] = disc_bdaddr[1];
        hcibuf[5] = disc_bdaddr[2];
        hcibuf[6] = disc_bdaddr[3];
        hcibuf[7] = disc_bdaddr[4];
        hcibuf[8] = disc_bdaddr[5];
        hcibuf[9] = 0x02; // Page Scan Repetition Mode
        hcibuf[10] = 0x00; // Reserved
        hcibuf[11] = 0x00; // Clock offset - low byte
        hcibuf[12] = 0x00; // Clock offset - high byte

        HCI_Command(hcibuf, 13);
}

void BTD::hci_read_remote_supported_features() {
        hcibuf[0] = 0x1b; // HCI OCF = 1b
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x02; // parameter length = 2
        hcibuf[3] = (uint8_t)(hci_handle & 0xFF); //connection handle - low byte
        hcibuf[4] = (uint8_t)((hci_handle >> 8) & 0x0F); //connection handle - high byte

        HCI_Command(hcibuf, 5);
}

void BTD::hci_set_local_name(const char* name) {
        hcibuf[0] = 0x13; // HCI OCF = 13
        hcibuf[1] = 0x03 << 2; // HCI OGF = 3
        hcibuf[2] = 248;

        uint8_t i, j;
        for(i = 0; i < strlen(name); i++)
				hcibuf[i + 3] = name[i];

        for(j = i; j < 3 + 248; j++)
        		hcibuf[j] = 0;

        HCI_Command(hcibuf, 3 + 248);
}

void BTD::hci_inquiry() {
        hci_clear_flag(HCI_FLAG_DEVICE_FOUND);
        hcibuf[0] = 0x01;
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x05; // Parameter Total Length = 5
        hcibuf[3] = 0x33; // LAP: Genera/Unlimited Inquiry Access Code (GIAC = 0x9E8B33) - see https://www.bluetooth.org/Technical/AssignedNumbers/baseband.htm
        hcibuf[4] = 0x8B;
        hcibuf[5] = 0x9E;
        hcibuf[6] = 0x30; // Inquiry time = 61.44 sec (maximum)
        hcibuf[7] = 0x0A; // 10 number of responses

        HCI_Command(hcibuf, 8);
}

void BTD::hci_inquiry_cancel() {
        hcibuf[0] = 0x02;
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x00; // Parameter Total Length = 0

        HCI_Command(hcibuf, 3);
}

void BTD::hci_connect() {
        hci_connect(disc_bdaddr); // Use last discovered device
}

void BTD::hci_connect(uint8_t *bdaddr) {
        hci_clear_flag(HCI_FLAG_CONNECT_COMPLETE | HCI_FLAG_CONNECT_EVENT);
        hcibuf[0] = 0x05;
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x0D; // parameter Total Length = 13
        hcibuf[3] = bdaddr[0]; // 6 octet bdaddr (LSB)
        hcibuf[4] = bdaddr[1];
        hcibuf[5] = bdaddr[2];
        hcibuf[6] = bdaddr[3];
        hcibuf[7] = bdaddr[4];
        hcibuf[8] = bdaddr[5];
        hcibuf[9] = 0x18; // DM1 or DH1 may be used
        hcibuf[10] = 0xCC; // DM3, DH3, DM5, DH5 may be used
        hcibuf[11] = 0x01; // Page repetition mode R1
        hcibuf[12] = 0x00; // Reserved
        hcibuf[13] = 0x00; // Clock offset
        hcibuf[14] = 0x00; // Invalid clock offset
        hcibuf[15] = 0x00; // Do not allow role switch

        HCI_Command(hcibuf, 16);
}

void BTD::hci_pin_code_request_reply() {
        hcibuf[0] = 0x0D; // HCI OCF = 0D
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x17; // parameter length 23
        hcibuf[3] = disc_bdaddr[0]; // 6 octet bdaddr
        hcibuf[4] = disc_bdaddr[1];
        hcibuf[5] = disc_bdaddr[2];
        hcibuf[6] = disc_bdaddr[3];
        hcibuf[7] = disc_bdaddr[4];
        hcibuf[8] = disc_bdaddr[5];
        if(pairWithWii) {
                hcibuf[9] = 6; // Pin length is the length of the Bluetooth address
                if(pairWiiUsingSync) {
                		ESP_LOGI(tag, "Pairing with Wii controller via SYNC");
                        for(uint8_t i = 0; i < 6; i++)
                                hcibuf[10 + i] = my_bdaddr[i]; // The pin is the Bluetooth dongles Bluetooth address backwards
                } else {
                        for(uint8_t i = 0; i < 6; i++)
                                hcibuf[10 + i] = disc_bdaddr[i]; // The pin is the Wiimote's Bluetooth address backwards
                }
                for(uint8_t i = 16; i < 26; i++)
                        hcibuf[i] = 0x00; // The rest should be 0
        } else {
                hcibuf[9] = strlen(btdPin); // Length of pin
                uint8_t i;
                for(i = 0; i < strlen(btdPin); i++) // The maximum size of the pin is 16
                        hcibuf[i + 10] = btdPin[i];
                for(; i < 16; i++)
                        hcibuf[i + 10] = 0x00; // The rest should be 0
        }

        HCI_Command(hcibuf, 26);
}

void BTD::hci_pin_code_negative_request_reply() {
        hcibuf[0] = 0x0E; // HCI OCF = 0E
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x06; // parameter length 6
        hcibuf[3] = disc_bdaddr[0]; // 6 octet bdaddr
        hcibuf[4] = disc_bdaddr[1];
        hcibuf[5] = disc_bdaddr[2];
        hcibuf[6] = disc_bdaddr[3];
        hcibuf[7] = disc_bdaddr[4];
        hcibuf[8] = disc_bdaddr[5];

        HCI_Command(hcibuf, 9);
}

void BTD::hci_link_key_request_negative_reply() {
        hcibuf[0] = 0x0C; // HCI OCF = 0C
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x06; // parameter length 6
        hcibuf[3] = disc_bdaddr[0]; // 6 octet bdaddr
        hcibuf[4] = disc_bdaddr[1];
        hcibuf[5] = disc_bdaddr[2];
        hcibuf[6] = disc_bdaddr[3];
        hcibuf[7] = disc_bdaddr[4];
        hcibuf[8] = disc_bdaddr[5];

        HCI_Command(hcibuf, 9);
}

void BTD::hci_authentication_request() {
        hcibuf[0] = 0x11; // HCI OCF = 11
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x02; // parameter length = 2
        hcibuf[3] = (uint8_t)(hci_handle & 0xFF); //connection handle - low byte
        hcibuf[4] = (uint8_t)((hci_handle >> 8) & 0x0F); //connection handle - high byte

        HCI_Command(hcibuf, 5);
}

void BTD::hci_disconnect(uint16_t handle) { // This is called by the different services
        hci_clear_flag(HCI_FLAG_DISCONNECT_COMPLETE);
        hcibuf[0] = 0x06; // HCI OCF = 6
        hcibuf[1] = 0x01 << 2; // HCI OGF = 1
        hcibuf[2] = 0x03; // parameter length = 3
        hcibuf[3] = (uint8_t)(handle & 0xFF); //connection handle - low byte
        hcibuf[4] = (uint8_t)((handle >> 8) & 0x0F); //connection handle - high byte
        hcibuf[5] = 0x13; // reason

        HCI_Command(hcibuf, 6);
}

void BTD::hci_write_class_of_device() { // See http://bluetooth-pentest.narod.ru/software/bluetooth_class_of_device-service_generator.html
        hcibuf[0] = 0x24; // HCI OCF = 24
        hcibuf[1] = 0x03 << 2; // HCI OGF = 3
        hcibuf[2] = 0x03; // parameter length = 3
        hcibuf[3] = 0x04; // Robot
        hcibuf[4] = 0x08; // Toy
        hcibuf[5] = 0x00;

        HCI_Command(hcibuf, 6);
}
/*******************************************************************
 *                                                                 *
 *                        HCI ACL Data Packet                      *
 *                                                                 *
 *   buf[0]          buf[1]          buf[2]          buf[3]
 *   0       4       8    11 12      16              24            31 MSB
 *  .-+-+-+-+-+-+-+-|-+-+-+-|-+-|-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *  |      HCI Handle       |PB |BC |       Data Total Length       |   HCI ACL Data Packet
 *  .-+-+-+-+-+-+-+-|-+-+-+-|-+-|-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *
 *   buf[4]          buf[5]          buf[6]          buf[7]
 *   0               8               16                            31 MSB
 *  .-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *  |            Length             |          Channel ID           |   Basic L2CAP header
 *  .-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *
 *   buf[8]          buf[9]          buf[10]         buf[11]
 *   0               8               16                            31 MSB
 *  .-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.
 *  |     Code      |  Identifier   |            Length             |   Control frame (C-frame)
 *  .-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-.   (signaling packet format)
 */
/************************************************************/
/*                    L2CAP Commands                        */
/************************************************************/
void BTD::L2CAP_Command(uint16_t handle, uint8_t* data, uint16_t nbytes, uint8_t channelLow, uint8_t channelHigh) {
        uint8_t buf[9 + nbytes];
        buf[0] = (uint8_t)(0x02); // HCI ACL DATA
        buf[1] = (uint8_t)(handle & 0xff); // HCI handle with PB,BC flag
        buf[2] = (uint8_t)(((handle >> 8) & 0x0f) | 0x20);
        buf[3] = (uint8_t)((4 + nbytes) & 0xff); // HCI ACL total data length
        buf[4] = (uint8_t)((4 + nbytes) >> 8);
        buf[5] = (uint8_t)(nbytes & 0xff); // L2CAP header: Length
        buf[6] = (uint8_t)(nbytes >> 8);
        buf[7] = channelLow;
        buf[8] = channelHigh;

        for(uint16_t i = 0; i < nbytes; i++) // L2CAP C-frame
                buf[9 + i] = data[i];

		while(! esp_vhci_host_check_send_available()) {
				vTaskDelay(10 / portTICK_PERIOD_MS);
		}

		if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
				uint16_t i;
				char log_buf[(8 + nbytes)*3];
				for (i = 0; i < 8 + nbytes; i++) {
					sprintf((log_buf + i*3), "%02x", buf[i + 1]);
					log_buf[i*3 + 2] = ' ';
				}
				log_buf[(i - 1)*3 + 2] = '\0';
				ESP_LOGD(tag, "L2CAP COMMAND > %s", log_buf);
		}

		esp_vhci_host_send_packet(buf, nbytes + 9);
}

void BTD::l2cap_connection_request(uint16_t handle, uint8_t rxid, uint8_t* scid, uint16_t psm) {
        l2capoutbuf[0] = L2CAP_CMD_CONNECTION_REQUEST; // Code
        l2capoutbuf[1] = rxid; // Identifier
        l2capoutbuf[2] = 0x04; // Length
        l2capoutbuf[3] = 0x00;
        l2capoutbuf[4] = (uint8_t)(psm & 0xff); // PSM
        l2capoutbuf[5] = (uint8_t)(psm >> 8);
        l2capoutbuf[6] = scid[0]; // Source CID
        l2capoutbuf[7] = scid[1];

        L2CAP_Command(handle, l2capoutbuf, 8);
}

void BTD::l2cap_connection_response(uint16_t handle, uint8_t rxid, uint8_t* dcid, uint8_t* scid, uint8_t result) {
        l2capoutbuf[0] = L2CAP_CMD_CONNECTION_RESPONSE; // Code
        l2capoutbuf[1] = rxid; // Identifier
        l2capoutbuf[2] = 0x08; // Length
        l2capoutbuf[3] = 0x00;
        l2capoutbuf[4] = dcid[0]; // Destination CID
        l2capoutbuf[5] = dcid[1];
        l2capoutbuf[6] = scid[0]; // Source CID
        l2capoutbuf[7] = scid[1];
        l2capoutbuf[8] = result; // Result: Pending or Success
        l2capoutbuf[9] = 0x00;
        l2capoutbuf[10] = 0x00; // No further information
        l2capoutbuf[11] = 0x00;

        L2CAP_Command(handle, l2capoutbuf, 12);
}

void BTD::l2cap_config_request(uint16_t handle, uint8_t rxid, uint8_t* dcid) {
        l2capoutbuf[0] = L2CAP_CMD_CONFIG_REQUEST; // Code
        l2capoutbuf[1] = rxid; // Identifier
        l2capoutbuf[2] = 0x04; // Length
        l2capoutbuf[3] = 0x00;
        l2capoutbuf[4] = dcid[0]; // Destination CID
        l2capoutbuf[5] = dcid[1];
        l2capoutbuf[6] = 0x00; // Flags
        l2capoutbuf[7] = 0x00;
        l2capoutbuf[8] = 0x01; // Config Opt: type = MTU (Maximum Transmission Unit) - Hint
        l2capoutbuf[9] = 0x02; // Config Opt: length
        l2capoutbuf[10] = 0xFF; // MTU
        l2capoutbuf[11] = 0xFF;

        L2CAP_Command(handle, l2capoutbuf, 12);
}

void BTD::l2cap_config_response(uint16_t handle, uint8_t rxid, uint8_t* scid) {
        l2capoutbuf[0] = L2CAP_CMD_CONFIG_RESPONSE; // Code
        l2capoutbuf[1] = rxid; // Identifier
        l2capoutbuf[2] = 0x0A; // Length
        l2capoutbuf[3] = 0x00;
        l2capoutbuf[4] = scid[0]; // Source CID
        l2capoutbuf[5] = scid[1];
        l2capoutbuf[6] = 0x00; // Flag
        l2capoutbuf[7] = 0x00;
        l2capoutbuf[8] = 0x00; // Result
        l2capoutbuf[9] = 0x00;
        l2capoutbuf[10] = 0x01; // Config
        l2capoutbuf[11] = 0x02;
        l2capoutbuf[12] = 0xA0;
        l2capoutbuf[13] = 0x02;

        L2CAP_Command(handle, l2capoutbuf, 14);
}

void BTD::l2cap_disconnection_request(uint16_t handle, uint8_t rxid, uint8_t* dcid, uint8_t* scid) {
        l2capoutbuf[0] = L2CAP_CMD_DISCONNECT_REQUEST; // Code
        l2capoutbuf[1] = rxid; // Identifier
        l2capoutbuf[2] = 0x04; // Length
        l2capoutbuf[3] = 0x00;
        l2capoutbuf[4] = dcid[0];
        l2capoutbuf[5] = dcid[1];
        l2capoutbuf[6] = scid[0];
        l2capoutbuf[7] = scid[1];

        L2CAP_Command(handle, l2capoutbuf, 8);
}

void BTD::l2cap_disconnection_response(uint16_t handle, uint8_t rxid, uint8_t* dcid, uint8_t* scid) {
        l2capoutbuf[0] = L2CAP_CMD_DISCONNECT_RESPONSE; // Code
        l2capoutbuf[1] = rxid; // Identifier
        l2capoutbuf[2] = 0x04; // Length
        l2capoutbuf[3] = 0x00;
        l2capoutbuf[4] = dcid[0];
        l2capoutbuf[5] = dcid[1];
        l2capoutbuf[6] = scid[0];
        l2capoutbuf[7] = scid[1];

        L2CAP_Command(handle, l2capoutbuf, 8);
}

void BTD::l2cap_information_response(uint16_t handle, uint8_t rxid, uint8_t infoTypeLow, uint8_t infoTypeHigh) {
        l2capoutbuf[0] = L2CAP_CMD_INFORMATION_RESPONSE; // Code
        l2capoutbuf[1] = rxid; // Identifier
        l2capoutbuf[2] = 0x08; // Length
        l2capoutbuf[3] = 0x00;
        l2capoutbuf[4] = infoTypeLow;
        l2capoutbuf[5] = infoTypeHigh;
        l2capoutbuf[6] = 0x00; // Result = success
        l2capoutbuf[7] = 0x00; // Result = success
        l2capoutbuf[8] = 0x00;
        l2capoutbuf[9] = 0x00;
        l2capoutbuf[10] = 0x00;
        l2capoutbuf[11] = 0x00;

        L2CAP_Command(handle, l2capoutbuf, 12);
}

/* PS3 Commands - only set Bluetooth address is implemented in this library */
void BTD::setBdaddr(uint8_t* bdaddr) {
        /* Set the internal Bluetooth address */
        uint8_t buf[8];
        buf[0] = 0x01;
        buf[1] = 0x00;

        for(uint8_t i = 0; i < 6; i++)
                buf[i + 2] = bdaddr[5 - i]; // Copy into buffer, has to be written reversed, so it is MSB first

        // bmRequest = Host to device (0x00) | Class (0x20) | Interface (0x01) = 0x21, bRequest = Set Report (0x09), Report ID (0xF5), Report Type (Feature 0x03), interface (0x00), datalength, datalength, data
        //pUsb->ctrlReq(bAddress, epInfo[BTD_CONTROL_PIPE].epAddr, bmREQ_HID_OUT, HID_REQUEST_SET_REPORT, 0xF5, 0x03, 0x00, 8, 8, buf, NULL);
}

void BTD::setMoveBdaddr(uint8_t* bdaddr) {
        /* Set the internal Bluetooth address */
        uint8_t buf[11];
        buf[0] = 0x05;
        buf[7] = 0x10;
        buf[8] = 0x01;
        buf[9] = 0x02;
        buf[10] = 0x12;

        for(uint8_t i = 0; i < 6; i++)
                buf[i + 1] = bdaddr[i];

        // bmRequest = Host to device (0x00) | Class (0x20) | Interface (0x01) = 0x21, bRequest = Set Report (0x09), Report ID (0x05), Report Type (Feature 0x03), interface (0x00), datalength, datalength, data
        //pUsb->ctrlReq(bAddress, epInfo[BTD_CONTROL_PIPE].epAddr, bmREQ_HID_OUT, HID_REQUEST_SET_REPORT, 0x05, 0x03, 0x00, 11, 11, buf, NULL);
}

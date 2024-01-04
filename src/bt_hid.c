/*
 * Derived from the btstack hid_host_demo:
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Modifications Copyright (C) 2021-2023 Brian Starkey <stark3y@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


#include "pico/async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "bt_hid.h"
#include "btstack.h"
#include "btstack_config.h"
#include "btstack_run_loop.h"
#include "classic/sdp_server.h"
#include "utils.h"
#include "pinout.h"

#define MAX_ATTRIBUTE_VALUE_SIZE 512

// SN30 Pro
// static const char * remote_addr_string = "E4:17:D8:EE:73:0E";
// Real DS4
// static const char * remote_addr_string = "00:22:68:DB:D3:66";
// Knockoff DS4
// static const char * remote_addr_string = "A5:15:66:8E:91:3B";
// Brian C Knockoff DS4
// static const char * remote_addr_string = "8C:41:F2:D0:32:43";
// Blakes B03 DS4 ADDRESS??
// static const char * remote_addr_string = "00:01:6C:A6:B4:DA";
// My broke ass controller
static const char *remote_addr_string = "00:1F:E2:B0:08:EE";
static bd_addr_t remote_addr;
static bd_addr_t connected_addr;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// SDP
static uint8_t hid_descriptor_storage[MAX_ATTRIBUTE_VALUE_SIZE];

static uint16_t hid_host_cid = 0;
static bool hid_host_descriptor_available = false;
static hid_protocol_mode_t hid_host_report_mode = HID_PROTOCOL_MODE_REPORT;

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void hid_host_setup(void) {
    // Initialize L2CAP
    l2cap_init();

    sdp_init();

    // Initialize HID Host
    hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
    hid_host_register_packet_handler(packet_handler);

    // Allow sniff mode requests by HID device and support role switch
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

    // try to become master on incoming connections
    hci_set_master_slave_policy(HCI_ROLE_MASTER);

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
}

const struct bt_hid_state default_state = {
    .buttons = 0,
    .lx = 0x80,
    .ly = 0x80,
    .rx = 0x80,
    .ry = 0x80,
    .l2 = 0x80,
    .r2 = 0x80,
    .hat = 0x8,
};

struct bt_hid_state latest;

struct __attribute__((packed)) input_report_17 {
    uint8_t report_id;
    uint8_t pad[2];

    uint8_t lx, ly;
    uint8_t rx, ry;
    uint8_t buttons[3];
    uint8_t l2, r2;

    uint16_t timestamp;
    uint16_t temperature;
    uint16_t gyro[3];
    uint16_t accel[3];
    uint8_t pad2[5];
    uint8_t status[2];
    uint8_t pad3;
};

static void hid_host_handle_interrupt_report(const uint8_t *packet, uint16_t packet_len) {
    static struct bt_hid_state last_state = {0};

    // Only interested in report_id 0x11
    if (packet_len < sizeof(struct input_report_17) + 1) {
        return;
    }

    if ((packet[0] != 0xa1) || (packet[1] != 0x11)) {
        return;
    }

    // printf_hexdump(packet, packet_len);

    struct input_report_17 *report = (struct input_report_17 *)&packet[1];

    // if we are using the inputs we don't want to change the mem
    if (!g_bt_packet_gaurd) {
        latest = (struct bt_hid_state){
            .buttons = ((report->buttons[0] & 0xf0) << 8) | ((report->buttons[2] & 0x3) << 8) | (report->buttons[1]),
            .lx = report->lx,
            .ly = report->ly,
            .rx = report->rx,
            .ry = report->ry,
            .l2 = report->l2,
            .r2 = report->r2,
            .hat = (report->buttons[0] & 0xf),
        };
    }

}

void bt_hid_get_latest(struct bt_hid_state *dst) {
    async_context_t *context = cyw43_arch_async_context();
    async_context_acquire_lock_blocking(context);
    memcpy(dst, &latest, sizeof(*dst));
    async_context_release_lock(context);
}

struct bt_hid_state bt_hid_get_latest_lazy() {
    return latest;
}

static void bt_hid_disconnected(bd_addr_t addr) {
    hid_host_cid = 0;
    hid_host_descriptor_available = false;

    memcpy(&latest, &default_state, sizeof(latest));
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    uint8_t event;
    uint8_t hid_event;
    bd_addr_t event_addr;
    uint8_t status;
    uint8_t reason;

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    event = hci_event_packet_get_type(packet);
    switch (event) {
    case BTSTACK_EVENT_STATE:
        // On boot, we try a manual connection
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            sprintf(g_print_buf, "Starting hid_host_connect (%s)\n", bd_addr_to_str(remote_addr));
            vGuardedPrint(g_print_buf);
            status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
            if (status != ERROR_CODE_SUCCESS) {
                sprintf(g_print_buf, "hid_host_connect command failed: 0x%02x\n", status);
                vGuardedPrint(g_print_buf);
            }
        }
        break;
    case HCI_EVENT_CONNECTION_COMPLETE:
        status = hci_event_connection_complete_get_status(packet);
        sprintf(g_print_buf, "Connection complete: %x\n", status);
        vGuardedPrint(g_print_buf);
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        status = hci_event_disconnection_complete_get_status(packet);
        reason = hci_event_disconnection_complete_get_reason(packet);
        sprintf(g_print_buf, "Disconnection complete: status: %x, reason: %x\n", status, reason);
        vGuardedPrint(g_print_buf);
        break;
    case HCI_EVENT_MAX_SLOTS_CHANGED:
        status = hci_event_max_slots_changed_get_lmp_max_slots(packet);
        sprintf(g_print_buf, "Max slots changed: %x\n", status);
        vGuardedPrint(g_print_buf);
        break;
    case HCI_EVENT_PIN_CODE_REQUEST:
        sprintf(g_print_buf, "Pin code request. Responding '0000'\n");
        vGuardedPrint(g_print_buf);
        hci_event_pin_code_request_get_bd_addr(packet, event_addr);
        gap_pin_code_response(event_addr, "0000");
        break;
    case HCI_EVENT_USER_CONFIRMATION_REQUEST:
        sprintf(g_print_buf, "SSP User Confirmation Request: %d\n", little_endian_read_32(packet, 8));
        vGuardedPrint(g_print_buf);
        break;
    case HCI_EVENT_HID_META:
        hid_event = hci_event_hid_meta_get_subevent_code(packet);
        switch (hid_event) {
        case HID_SUBEVENT_INCOMING_CONNECTION:
            hid_subevent_incoming_connection_get_address(packet, event_addr);
            sprintf(g_print_buf, "Accepting connection from %s\n", bd_addr_to_str(event_addr));
            vGuardedPrint(g_print_buf);
            hid_host_accept_connection(hid_subevent_incoming_connection_get_hid_cid(packet), hid_host_report_mode);
            break;
        case HID_SUBEVENT_CONNECTION_OPENED:
            status = hid_subevent_connection_opened_get_status(packet);
            hid_subevent_connection_opened_get_bd_addr(packet, event_addr);
            if (status != ERROR_CODE_SUCCESS) {
                sprintf(g_print_buf, "Connection to %s failed: 0x%02x\n", bd_addr_to_str(event_addr), status);
                vGuardedPrint(g_print_buf);
                bt_hid_disconnected(event_addr);
                return;
            }
            hid_host_descriptor_available = false;
            hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
            sprintf(g_print_buf, "Connected to %s\n", bd_addr_to_str(event_addr));
            vGuardedPrint(g_print_buf);
            bd_addr_copy(connected_addr, event_addr);
            // Turn blue led on and beep
            gpio_put(STAT_LED_0, HIGH);
            beep(2, 50);
            // set bt connected
            is_bt_connected = true;

            break;
        case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
            status = hid_subevent_descriptor_available_get_status(packet);
            if (status == ERROR_CODE_SUCCESS) {
                hid_host_descriptor_available = true;

                uint16_t dlen = hid_descriptor_storage_get_descriptor_len(hid_host_cid);
                sprintf(g_print_buf, "HID descriptor available. Len: %d\n", dlen);
                vGuardedPrint(g_print_buf);

                // Send FEATURE 0x05, to switch the controller to "full" report mode
                hid_host_send_get_report(hid_host_cid, HID_REPORT_TYPE_FEATURE, 0x05);
            } else {
                sprintf(g_print_buf, "Couldn't process HID Descriptor, status: %d\n", status);
                vGuardedPrint(g_print_buf);
            }
            break;
        case HID_SUBEVENT_REPORT:
            if (hid_host_descriptor_available) {
                hid_host_handle_interrupt_report(hid_subevent_report_get_report(packet),
                                                 hid_subevent_report_get_report_len(packet));
            } else {
                // this gets pretty annoying and causes issues
                // sprintf(g_print_buf, "No hid host descriptor available\n");
                // vGuardedPrint(g_print_buf);
                // printf_hexdump(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
            }
            break;
        case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
            status = hid_subevent_set_protocol_response_get_handshake_status(packet);
            if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL) {
                sprintf(g_print_buf, "Protocol handshake error: 0x%02x\n", status);
                vGuardedPrint(g_print_buf);
                break;
            }
            hid_protocol_mode_t proto = hid_subevent_set_protocol_response_get_protocol_mode(packet);
            switch (proto) {
            case HID_PROTOCOL_MODE_BOOT:
                sprintf(g_print_buf, "Negotiated protocol: BOOT\n");
                vGuardedPrint(g_print_buf);
                break;
            case HID_PROTOCOL_MODE_REPORT:
                sprintf(g_print_buf, "Negotiated protocol: REPORT\n");
                vGuardedPrint(g_print_buf);
                break;
            default:
                sprintf(g_print_buf, "Negotiated unknown protocol: 0x%x\n", proto);
                vGuardedPrint(g_print_buf);
                break;
            }
            break;
        case HID_SUBEVENT_CONNECTION_CLOSED:
            sprintf(g_print_buf, "HID connection closed: %s\n", bd_addr_to_str(connected_addr));
            vGuardedPrint(g_print_buf);
            bt_hid_disconnected(connected_addr);
            // turn BLUE LED OFF AND BEEP
            gpio_put(STAT_LED_0, LOW);
            beep(2, 50);
            // ? set disconected maybe shut down sequence
            is_bt_connected = false;
            break;
        case HID_SUBEVENT_GET_REPORT_RESPONSE: {
            status = hid_subevent_get_report_response_get_handshake_status(packet);
            uint16_t dlen = hid_subevent_get_report_response_get_report_len(packet);
            sprintf(g_print_buf, "GET_REPORT response. status: %d, len: %d\n", status, dlen);
            vGuardedPrint(g_print_buf);
        } break;
        default:
            sprintf(g_print_buf, "Unknown HID subevent: 0x%x\n", hid_event);
            vGuardedPrint(g_print_buf);
            break;
        }
        break;
    default:
        // sprintf(g_print_buf, "Unknown HCI event: 0x%x\n", event);
        // vGuardedPrint(g_print_buf);
        break;
    }
}

void bt_hid_init() {
    gap_set_security_level(LEVEL_2);
    hid_host_setup();
    sscanf_bd_addr(remote_addr_string, remote_addr);
    bt_hid_disconnected(remote_addr);
    hci_power_control(HCI_POWER_ON);
}

void bt_main(void) {
    btstack_run_loop_execute();
}

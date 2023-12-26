// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#ifndef BT_HID_H
#define BT_HID_H

struct bt_hid_state {
    uint16_t buttons;
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t l2;
    uint8_t r2;
    uint8_t hat;
    uint8_t pad;
};

void bt_hid_get_latest(struct bt_hid_state *dst);
struct bt_hid_state bt_hid_get_latest_lazy();
void bt_hid_init();
void bt_main();

#endif
/*
 * drivers/misc/bluedroid_pm.h
 *
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef _BLUEDROID_PM_H
#define _BLUEDROID_PM_H
void bluedroid_pm_set_ext_state(bool blocked);
void bt_wlan_lock(void);
void bt_wlan_unlock(void);
#endif /* _BLUEDROID_PM_H */

/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef T234_HWPM_IP_DISPLAY_H
#define T234_HWPM_IP_DISPLAY_H

#if defined(CONFIG_T234_HWPM_IP_DISPLAY)
#define T234_HWPM_ACTIVE_IP_DISPLAY		T234_HWPM_IP_DISPLAY,

/* This data should ideally be available in HW headers */
#define T234_HWPM_IP_DISPLAY_NUM_INSTANCES		1U
#define T234_HWPM_IP_DISPLAY_NUM_CORE_ELEMENT_PER_INST	1U
#define T234_HWPM_IP_DISPLAY_NUM_PERFMON_PER_INST	1U
#define T234_HWPM_IP_DISPLAY_NUM_PERFMUX_PER_INST	1U
#define T234_HWPM_IP_DISPLAY_NUM_BROADCAST_PER_INST	0U

extern struct hwpm_ip t234_hwpm_ip_display;

#else
#define T234_HWPM_ACTIVE_IP_DISPLAY
#endif

#endif /* T234_HWPM_IP_DISPLAY_H */

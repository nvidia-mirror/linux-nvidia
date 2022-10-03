// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/string.h>
#include "eventlib.h"

void eventlib_copy_to_buffer(struct eventlib_write_buffer *buffer, void *src, uint32_t size)
{
	uint32_t to_write = buffer->size < size ? buffer->size : size;

	memcpy(buffer->destination, src, to_write);
	buffer->size -= to_write;
	buffer->destination += to_write;
}

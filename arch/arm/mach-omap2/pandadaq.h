/*
 * Driver for PandaDAQ data acquisition platform
 *
 * Copyright (C) 2010 Ben Gamari
 *
 * Author: Ben Gamari <bgamari.foss@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PANDADAQ_H
#define __PANDADAQ_H

struct pandadaq_platform_data {
	unsigned long phys_base;
};

int pandadaq_init(struct pandadaq_platform_data *pdata);

#endif

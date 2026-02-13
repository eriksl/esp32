#pragma once

enum
{
	IO_RAMDISK_GET_USED,
	IO_RAMDISK_SET_SIZE,
	IO_RAMDISK_GET_SIZE,
	IO_RAMDISK_WIPE,
};

void ramdisk_init(unsigned int size);

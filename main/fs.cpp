#include <stdint.h>
#include <stdbool.h>
#include <string.h> // for strerror

#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "info.h"
#include "ramdisk.h"
#include "encryption.h"
#include "fs.h"

#include <esp_littlefs.h>

#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <string>
#include <format>

static bool inited = false;

void fs_init(void)
{
	esp_vfs_littlefs_conf_t littlefs_parameters =
	{
		.base_path = "/littlefs",
		.partition_label = "littlefs",
		.partition = nullptr,
		.format_if_mount_failed = true,
		.read_only = 0,
		.dont_mount = 0,
		.grow_on_mount = 0,
	};

	assert(!inited);

	util_abort_on_esp_err("esp_vfs_littlefs_register", esp_vfs_littlefs_register(&littlefs_parameters));

	inited = true;
}

void fs_command_info(cli_command_call_t *call)
{
	size_t total, used, avail, usedpct;
	int fd;

	assert(inited);
	assert(call->parameter_count == 0);

	util_abort_on_esp_err("esp_littlefs_info", esp_littlefs_info("littlefs", &total, &used));
	avail = total - used;
	usedpct = (100 * used) / total;

	call->result = "LITTLEFS";

	if(esp_littlefs_mounted("littlefs"))
		call->result += std::format(" mounted at /littlefs:\n- total size: {:d} kB\n- used: {:d} kB\n- available {:d} kB, {:d}% used",
				total / 1024, used / 1024, avail / 1024, usedpct);
	else
		call->result += " not mounted";

	if((fd = open("/ramdisk", O_RDONLY | O_DIRECTORY)) >= 0)
	{
		ioctl(fd, IO_RAMDISK_GET_SIZE, &total);
		ioctl(fd, IO_RAMDISK_GET_USED, &used);
		close(fd);

		avail = total - used;
		usedpct = (100 * used) / total;

		call->result += std::format("\nRAMDISK mounted at /ramdisk:\n- total size: {:d} kB\n- used: {:d} kB\n- available {:d} kB, {:d}% used",
				total / 1024, used / 1024, avail / 1024, usedpct);
	}
}

void fs_command_list(cli_command_call_t *call)
{
	DIR *dir;
	struct dirent *dirent;
	struct stat statb;
	std::string filename;
	bool option_long;
	std::string ctime;
	std::string mtime;
	int inode, length, allocated;

	assert(inited);
	assert((call->parameter_count > 0) && (call->parameter_count < 3));

	if(!(dir = opendir(call->parameters[0].str.c_str())))
	{
		call->result = std::format("opendir of {} failed", call->parameters[0].str);
		return;
	}

	if(call->parameter_count == 2)
	{
		if(call->parameters[1].str == "-l")
			option_long = true;
		else
		{
			call->result = std::format("fs-list: unknown option: {}\n", call->parameters[1].str);
			return;
		}
	}
	else
		option_long = false;

	call->result = std::format("DIRECTORY {}", call->parameters[0].str);

	while((dirent = readdir(dir)))
	{
		filename = std::format("{}/{}", call->parameters[0].str.c_str(), dirent->d_name);

		if(stat(filename.c_str(), &statb))
		{
			inode = -1;
			length = -1;
			allocated = -1;
			mtime.clear();
			ctime.clear();
		}
		else
		{
			inode = statb.st_ino;
			length = statb.st_size;
			allocated = (statb.st_blocks * 512UL) / 1024UL;
			ctime = util_time_to_string(statb.st_ctim.tv_sec);
			mtime = util_time_to_string(statb.st_mtim.tv_sec);
		}

		if(option_long)
			call->result += std::format("\n{:20} {:7d} {:4d}k {:19} {:19} {:11d}",
					dirent->d_name, length, allocated, ctime, mtime, inode);
		else
			call->result += std::format("\n{:3d}k {}", length / 1024UL, dirent->d_name);
	}

	closedir(dir);
}

void fs_command_format(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 1);

	if(esp_littlefs_format(call->parameters[0].str.c_str()))
	{
		call->result = std::format("format of {} failed", call->parameters[0].str);
		return;
	}

	call->result = "format complete";
}

void fs_command_read(cli_command_call_t *call)
{
	int fd, length;

	assert(inited);
	assert(call->parameter_count == 3);

	if((fd = open(call->parameters[2].str.c_str(), O_RDONLY, 0)) < 0)
	{
		call->result = std::format("ERROR: cannot open file {}: {}", call->parameters[2].str, strerror(errno));
		return;
	}

	if(lseek(fd, call->parameters[1].unsigned_int, SEEK_SET) == -1)
		length = 0;
	else
	{
		call->result_oob.resize(call->parameters[0].unsigned_int);

		if((length = ::read(fd, call->result_oob.data(), call->parameters[0].unsigned_int)) == 0)
		{
			call->result = "ERROR: read failed";
			close(fd);
			return;
		}

		call->result_oob.resize(length);
	}

	close(fd);

	call->result = std::format("OK chunk read: {:d}", length);
}

void fs_command_write(cli_command_call_t *call)
{
	int fd, length;
	struct stat statb;
	int open_mode;

	assert(inited);
	assert(call->parameter_count == 3);

	open_mode = O_WRONLY | O_CREAT | (call->parameters[0].unsigned_int ? O_APPEND : O_TRUNC);

	if(call->parameters[1].unsigned_int != call->oob.length())
	{
		call->result = std::format("ERROR: length [{:d}] != oob data length [{:d}]", call->parameters[1].unsigned_int, call->oob.length());
		return;
	}

	if((fd = open(call->parameters[2].str.c_str(), open_mode, 0)) < 0)
	{
		call->result = std::format("ERROR: cannot open file {}: {}", call->parameters[2].str, strerror(errno));
		return;
	}

	if((length = write(fd, call->oob.data(), call->oob.length())) != call->parameters[1].unsigned_int)
	{
		call->result = "ERROR: write failed";
		close(fd);
		return;
	}

	close(fd);

	if(stat(call->parameters[2].str.c_str(), &statb))
		length = -1;
	else
		length = statb.st_size;

	call->result = std::format("OK file length: {:d}", length);
}

void fs_command_erase(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 1);

	if(unlink(call->parameters[0].str.c_str()))
		call->result = "file erase failed";
	else
		call->result = "OK file erased";
}

void fs_command_rename(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 2);

	if(rename(call->parameters[0].str.c_str(), call->parameters[1].str.c_str()))
		call->result = "file rename failed";
	else
		call->result = "OK file renamed";
}

void fs_command_checksum(cli_command_call_t *call)
{
	int length, fd;
	Encryption encryption;
	std::string hash;
	std::string hash_text;
	std::string block;

	if((fd = open(call->parameters[0].str.c_str(), O_RDONLY, 0)) < 0)
	{
		call->result = std::format("ERROR: cannot open file: {}", strerror(errno));
		return;
	}

	encryption.sha256_init();

	block.resize(4096);

	while((length = ::read(fd, block.data(), block.size())) > 0)
	{
		block.resize(length);
		encryption.sha256_update(block);
		block.resize(4096);
	}

	close(fd);

	hash = encryption.sha256_finish();
	hash_text = util_hash_to_string(hash);

	call->result = std::format("OK checksum: {}", hash_text);
}

void fs_command_truncate(cli_command_call_t *call)
{
	assert(call->parameter_count == 2);

	if(truncate(call->parameters[0].str.c_str(), call->parameters[1].unsigned_int))
	{
		call->result = std::format("ERROR: cannot truncate file: {}", strerror(errno));
		return;
	}

	call->result = "OK truncated";
}

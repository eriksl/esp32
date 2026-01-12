#include <stdint.h>
#include <stdbool.h>
#include <string.h> // for strerror

#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "info.h"
#include "ramdisk.h"
#include "fs.h"

#include <esp_littlefs.h>

#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <mbedtls/sha256.h>

#include <string>
#include <boost/format.hpp>

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
		call->result += (boost::format(" mounted at /littlefs:\n- total size: %u kB\n- used: %u kB\n- available %u kB, %u%% used") % (total / 1024) % (used / 1024) % (avail / 1024) % usedpct).str();
	else
		call->result += " not mounted";

	if((fd = open("/ramdisk", O_RDONLY | O_DIRECTORY)) >= 0)
	{
		ioctl(fd, IO_RAMDISK_GET_SIZE, &total);
		ioctl(fd, IO_RAMDISK_GET_USED, &used);
		close(fd);

		avail = total - used;
		usedpct = (100 * used) / total;

		call->result += (boost::format("\nRAMDISK mounted at /ramdisk:\n- total size: %u kB\n- used: %u kB\n- available %u kB, %u%% used") % (total / 1024) % (used / 1024) % (avail / 1024) % usedpct).str();
	}
}

void fs_command_list(cli_command_call_t *call)
{
	DIR *dir;
	struct dirent *dirent;
	struct stat statb;
	string_auto(filename, 64);
	bool option_long;
	string_auto(ctime, 32);
	string_auto(mtime, 32);
	int inode, length, allocated;

	assert(inited);
	assert((call->parameter_count > 0) && (call->parameter_count < 3));

	if(!(dir = opendir(call->parameters[0].str.c_str())))
	{
		call->result = (boost::format("opendir of %s failed") % call->parameters[0].str).str();
		return;
	}

	if(call->parameter_count == 2)
	{
		if(call->parameters[1].str == "-l")
			option_long = true;
		else
		{
			call->result = (boost::format("fs-list: unknown option: %s\n") % call->parameters[1].str).str();
			return;
		}
	}
	else
		option_long = false;

	call->result = (boost::format("DIRECTORY %s") % call->parameters[0].str).str();

	while((dirent = readdir(dir)))
	{
		string_format(filename, "%s/%s", call->parameters[0].str.c_str(), dirent->d_name);

		if(stat(string_cstr(filename), &statb))
		{
			inode = -1;
			length = -1;
			allocated = -1;
			string_clear(ctime);
			string_clear(mtime);
		}
		else
		{
			inode = statb.st_ino;
			length = statb.st_size;
			allocated = (statb.st_blocks * 512UL) / 1024UL;
			util_time_to_string(ctime, &statb.st_ctim.tv_sec);
			util_time_to_string(mtime, &statb.st_mtim.tv_sec);
		}

		if(option_long)
			call->result += (boost::format("\n%-20s %7d %4dk %19s %19s %11d") %
					dirent->d_name % length % allocated % string_cstr(ctime) % string_cstr(mtime) % inode).str();
		else
			call->result += (boost::format("\n%3luk %-20s") % (length / 1024UL) % dirent->d_name).str();
	}

	closedir(dir);
}

void fs_command_format(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 1);

	if(esp_littlefs_format(call->parameters[0].str.c_str()))
	{
		call->result = (boost::format("format of %s failed") % call->parameters[0].str).str();
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
		call->result = (boost::format("ERROR: cannot open file %s: %s") % call->parameters[2].str % strerror(errno)).str();
		return;
	}

	if(lseek(fd, call->parameters[1].unsigned_int, SEEK_SET) == -1)
		length = 0;
	else
	{
		call->result_oob.reserve(call->parameters[0].unsigned_int);

		if((length = ::read(fd, call->result_oob.data(), call->parameters[0].unsigned_int)) == 0)
		{
			call->result = "ERROR: read failed";
			close(fd);
			return;
		}

		call->result_oob.resize(length);
	}

	close(fd);

	call->result = (boost::format("OK chunk read: %d") % length).str();
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
		call->result = (boost::format("ERROR: length [%u] != oob data length [%u]") % call->parameters[1].unsigned_int % call->oob.length()).str();
		return;
	}

	if((fd = open(call->parameters[2].str.c_str(), open_mode, 0)) < 0)
	{
		call->result = (boost::format("ERROR: cannot open file %s: %s") % call->parameters[2].str % strerror(errno)).str();
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

	call->result = (boost::format("OK file length: %d") % length).str();
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
	mbedtls_sha256_context hash_context;
	unsigned char hash[32];
	string_auto(hash_text, (sizeof(hash) * 2) + 1);

	assert(call->parameter_count == 1);

	mbedtls_sha256_init(&hash_context);
	mbedtls_sha256_starts(&hash_context, /* no SHA-224 */ 0);

	if((fd = open(call->parameters[0].str.c_str(), O_RDONLY, 0)) < 0)
	{
		call->result = (boost::format("ERROR: cannot open file: %s") %strerror(errno)).str();
		return;
	}

	call->result_oob.reserve(4096);

	while((length = ::read(fd, call->result_oob.data(), 4096)) > 0)
		mbedtls_sha256_update(&hash_context, reinterpret_cast<const unsigned char *>(call->result_oob.data()), length);

	close(fd);

	call->result_oob.clear();

	mbedtls_sha256_finish(&hash_context, hash);
	mbedtls_sha256_free(&hash_context);

	util_hash_to_string(hash_text, sizeof(hash), hash);

	call->result = (boost::format("OK checksum: %s") % string_cstr(hash_text)).str();
}

void fs_command_truncate(cli_command_call_t *call)
{
	assert(call->parameter_count == 2);

	if(truncate(call->parameters[0].str.c_str(), call->parameters[1].unsigned_int))
	{
		call->result = (boost::format("ERROR: cannot truncate file: %s") % strerror(errno)).str();
		return;
	}

	call->result = "OK truncated";
}

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // for strerror

#include "string.h"
#include "fs.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "info.h"
#include "ramdisk.h"

#include <esp_littlefs.h>

#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <mbedtls/sha256.h>

static bool inited = false;

void fs_init(void)
{
	esp_vfs_littlefs_conf_t littlefs_parameters =
	{
		.base_path = "/littlefs",
		.partition_label = "littlefs",
		.format_if_mount_failed = true,
		.dont_mount = false,
		.grow_on_mount = true,
	};

	assert(!inited);

	util_abort_on_esp_err("esp_vfs_littlefs_register", esp_vfs_littlefs_register(&littlefs_parameters));

	inited = true;
}

void fs_command_info(cli_command_call_t *call)
{
	int total, used, avail, usedpct, fd;

	assert(inited);
	assert(call->parameter_count == 0);

	util_abort_on_esp_err("esp_littlefs_info", esp_littlefs_info("littlefs", &total, &used));
	avail = total - used;
	usedpct = (100 * used) / total;

	string_assign_cstr(call->result, "LITTLEFS");

	if(esp_littlefs_mounted("littlefs"))
		string_format_append(call->result, " mounted at /littlefs:\n- total size: %d kB\n- used: %d kB\n- available %d kB, %d%% used", total / 1024, used / 1024, avail / 1024, usedpct);
	else
		string_append_cstr(call->result, " not mounted");

	if((fd = open("/ramdisk", O_RDONLY | O_DIRECTORY)) >= 0)
	{
		ioctl(fd, IO_RAMDISK_GET_SIZE, &total);
		ioctl(fd, IO_RAMDISK_GET_USED, &used);
		close(fd);

		avail = total - used;
		usedpct = (100 * used) / total;

		string_format_append(call->result, "\nRAMDISK mounted at /ramdisk:\n- total size: %d kB\n- used: %d kB\n- available %d kB, %d%% used", total / 1024, used / 1024, avail / 1024, usedpct);
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

	if(!(dir = opendir(string_cstr(call->parameters[0].string))))
	{
		string_format(call->result, "opendir of %s failed", string_cstr(call->parameters[0].string));
		return;
	}

	if(call->parameter_count == 2)
	{
		if(string_equal_cstr(call->parameters[1].string, "-l"))
			option_long = true;
		else
		{
			string_format_append(call->result, "fs-list: unknown option: %s\n", string_cstr(call->parameters[1].string));
			return;
		}
	}
	else
		option_long = false;

	string_format(call->result, "DIRECTORY %s", string_cstr(call->parameters[0].string));

	while((dirent = readdir(dir)))
	{
		string_format(filename, "%s/%s", string_cstr(call->parameters[0].string), dirent->d_name);

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
			string_format_append(call->result, "\n%-20s %7d %3dk %19s %19s %11d",
					dirent->d_name, length, allocated, string_cstr(ctime), string_cstr(mtime), inode);
		else
			string_format_append(call->result, "\n%3luk %-20s",
					length / 1024UL, dirent->d_name);
	}

	closedir(dir);
}

void fs_command_format(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 1);

	if(esp_littlefs_format(string_cstr(call->parameters[0].string)))
	{
		string_format(call->result, "format of %s failed", string_cstr(call->parameters[0].string));
		return;
	}

	string_assign_cstr(call->result, "format complete");
}

void fs_command_read(cli_command_call_t *call)
{
	int fd, length;

	assert(inited);
	assert(call->parameter_count == 3);

	if(call->parameters[0].unsigned_int > (string_size(call->result_oob) - 1))
	{
		string_format(call->result, "ERROR: length [%u] too large for buffer length [%u]", call->parameters[0].unsigned_int, string_size(call->result_oob) - 1);
		return;
	}

	if((fd = open(string_cstr(call->parameters[2].string), O_RDONLY, 0)) < 0)
	{
		string_format(call->result, "ERROR: cannot open file %s: %s", string_cstr(call->parameters[2].string), strerror(errno));
		return;
	}

	if(lseek(fd, call->parameters[1].unsigned_int, SEEK_SET) == -1)
	{
		string_assign_cstr(call->result, "ERROR: lseek failed");
		close(fd);
		return;
	}

	if((length = string_read_fd(call->result_oob, fd, call->parameters[0].unsigned_int)) < 0)
	{
		string_assign_cstr(call->result, "ERROR: read failed");
		close(fd);
		return;
	}

	close(fd);

	string_format(call->result, "OK chunk read: %d", length);
}

void fs_command_write(cli_command_call_t *call)
{
	int fd, length;
	struct stat statb;
	int open_mode;

	assert(inited);
	assert(call->parameter_count == 3);

	open_mode = O_WRONLY | O_CREAT | (call->parameters[0].unsigned_int ? O_APPEND : O_TRUNC);

	if(call->parameters[1].unsigned_int != string_length(call->oob))
	{
		string_format(call->result, "ERROR: length [%u] != oob data length [%u]", call->parameters[1].unsigned_int, string_length(call->oob));
		return;
	}

	if((fd = open(string_cstr(call->parameters[2].string), open_mode, 0)) < 0)
	{
		string_format(call->result, "ERROR: cannot open file %s: %s", string_cstr(call->parameters[2].string), strerror(errno));
		return;
	}

	if((length = write(fd, string_data(call->oob), string_length(call->oob))) != call->parameters[1].unsigned_int)
	{
		string_assign_cstr(call->result, "ERROR: write failed");
		close(fd);
		return;
	}

	close(fd);

	if(stat(string_cstr(call->parameters[2].string), &statb))
		length = -1;
	else
		length = statb.st_size;

	string_format(call->result, "OK file length: %d", length);
}

void fs_command_erase(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 1);

	if(unlink(string_cstr(call->parameters[0].string)))
		string_assign_cstr(call->result, "file erase failed");
	else
		string_assign_cstr(call->result, "OK file erased");
}

void fs_command_rename(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 2);

	if(rename(string_cstr(call->parameters[0].string), string_cstr(call->parameters[1].string)))
		string_assign_cstr(call->result, "file rename failed");
	else
		string_assign_cstr(call->result, "OK file renamed");
}

void fs_command_checksum(cli_command_call_t *call)
{
	int fd;
	mbedtls_sha256_context hash_context;
	unsigned char hash[32];
	string_auto(hash_text, (sizeof(hash) * 2) + 1);

	assert(call->parameter_count == 1);
	assert(string_size(call->result_oob) > 4096);

	mbedtls_sha256_init(&hash_context);
	mbedtls_sha256_starts(&hash_context, /* no SHA-224 */ 0);

	if((fd = open(string_cstr(call->parameters[0].string), O_RDONLY, 0)) < 0)
	{
		string_assign_cstr(call->result, "ERROR: cannot open file: ");
		string_append_cstr(call->result, strerror(errno));
		return;
	}

	while(string_read_fd(call->result_oob, fd, 4096) > 0)
		mbedtls_sha256_update(&hash_context, string_data(call->result_oob), string_length(call->result_oob));

	close(fd);

	string_clear(call->result_oob);

	mbedtls_sha256_finish(&hash_context, hash);
	mbedtls_sha256_free(&hash_context);

	util_hash_to_string(hash_text, sizeof(hash), hash);

	string_format(call->result, "OK checksum: %s", string_cstr(hash_text));
}

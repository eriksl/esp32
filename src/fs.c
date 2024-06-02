#include <stdint.h>
#include <stdbool.h>
#include <string.h> // for strerror

#include "string.h"
#include "cli-command.h"
#include "fs.h"
#include "log.h"
#include "util.h"

#include <esp_littlefs.h>

#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
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

void command_fs_info(cli_command_call_t *call)
{
	unsigned int total, used;

	assert(inited);
	assert(call->parameter_count == 0);

	util_abort_on_esp_err("esp_littlefs_info", esp_littlefs_info("littlefs", &total, &used));

	string_format(call->result, "FS\nsizes:\n- total size: %u kB\n- used: %u kB\n- available %u kB", total / 1024, used / 1024, (total - used) / 1024);
	string_format_append(call->result, "\nmounted: %s", esp_littlefs_mounted("littlefs") ? "yes" : "no");
}

void command_fs_ls(cli_command_call_t *call)
{
	DIR *dir;
	struct dirent *dirent;
	struct stat statb;
	int length;
	string_auto(filename, 64);

	assert(inited);
	assert(call->parameter_count == 0);

	if(!(dir = opendir("/littlefs")))
		util_abort("littlefs opendir failed");

	string_assign_cstr(call->result, "DIRECTORY /littlefs");

	while((dirent = readdir(dir)))
	{
		errno = 0;

		string_assign_cstr(filename, "/littlefs/");
		string_append_cstr(filename, dirent->d_name);

		if(stat(string_cstr(filename), &statb))
			length = -1;
		else
			length = statb.st_size / 1024;

		string_format_append(call->result, "\n%3d %s", length, dirent->d_name);
	}

	closedir(dir);
}

void command_fs_format(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 0);

	if(esp_littlefs_format("littlefs"))
	{
		string_assign_cstr(call->result, "format failed");
		return;
	}

	string_assign_cstr(call->result, "format complete");
}

void command_fs_read(cli_command_call_t *call)
{
	string_auto(filename, 64);
	int fd, length;

	assert(inited);
	assert(call->parameter_count == 3);

	if(call->parameters[0].unsigned_int > (string_size(call->result_oob) - 1))
	{
		string_format(call->result, "ERROR: length [%u] too large for buffer length [%u]", call->parameters[0].unsigned_int, string_size(call->result_oob) - 1);
		return;
	}

	string_assign_cstr(filename, "/littlefs/");
	string_append_string(filename, call->parameters[2].string);

	if((fd = open(string_cstr(filename), O_RDONLY, 0)) < 0)
	{
		string_assign_cstr(call->result, "ERROR: cannot open file: ");
		string_append_cstr(call->result, strerror(errno));
		return;
	}

	if(lseek(fd, call->parameters[1].unsigned_int, SEEK_SET) == -1)
	{
		string_assign_cstr(call->result, "ERROR: lseek failed");
		close(fd);
		return;
	}

	if((length = read(fd, string_data_nonconst(call->result_oob), call->parameters[0].unsigned_int)) < 0)
	{
		string_assign_cstr(call->result, "ERROR: read failed");
		close(fd);
		return;
	}

	close(fd);

	string_set_length(call->result_oob, length);

	string_format(call->result, "OK chunk read: %d", length);
}

void command_fs_append(cli_command_call_t *call)
{
	string_auto(filename, 64);
	int fd, length;
	off_t offset;

	assert(inited);
	assert(call->parameter_count == 2);

	if(call->parameters[0].unsigned_int != string_length(call->oob))
	{
		string_format(call->result, "ERROR: length [%u] != oob data length [%u]", call->parameters[0].unsigned_int, string_length(call->oob));
		return;
	}

	string_assign_cstr(filename, "/littlefs/");
	string_append_string(filename, call->parameters[1].string);

	if((fd = open(string_cstr(filename), O_WRONLY | O_APPEND | O_CREAT, 0)) < 0)
	{
		string_assign_cstr(call->result, "ERROR: cannot open file: ");
		string_append_cstr(call->result, strerror(errno));
		return;
	}

	if((length = write(fd, string_data(call->oob), string_length(call->oob))) != call->parameters[0].unsigned_int)
	{
		string_assign_cstr(call->result, "ERROR: write failed");
		close(fd);
		return;
	}

	if((offset = lseek(fd, 0, SEEK_END)) == -1)
	{
		string_assign_cstr(call->result, "ERROR: lseek failed");
		close(fd);
		return;
	}

	close(fd);

	string_format(call->result, "OK file length: %ld", offset);
}
 
void command_fs_erase(cli_command_call_t *call)
{
	string_auto(filename, 64);

	assert(inited);
	assert(call->parameter_count == 1);

	string_assign_cstr(filename, "/littlefs/");
	string_append_string(filename, call->parameters[0].string);

	unlink(string_cstr(filename));

	string_assign_cstr(call->result, "OK file erased");
}

void command_fs_checksum(cli_command_call_t *call)
{
	int fd, length;
	string_auto(filename, 64);
	mbedtls_sha256_context hash_context;
	unsigned char hash[32];
	string_auto(hash_text, (sizeof(hash) * 2) + 1);
	uint8_t *buffer = string_data_nonconst(call->result_oob);

	assert(call->parameter_count == 1);
	assert(string_size(call->result_oob) > 4096);

	string_assign_cstr(filename, "/littlefs/");
	string_append_string(filename, call->parameters[0].string);

	mbedtls_sha256_init(&hash_context);
	mbedtls_sha256_starts(&hash_context, /* no SHA-224 */ 0);

	if((fd = open(string_cstr(filename), O_RDONLY, 0)) < 0)
	{
		string_assign_cstr(call->result, "ERROR: cannot open file: ");
		string_append_cstr(call->result, strerror(errno));
		return;
	}

	while((length = read(fd, buffer, 4096)) > 0)
		mbedtls_sha256_update(&hash_context, buffer, length);

	close(fd);

	string_clear(call->result_oob);

	mbedtls_sha256_finish(&hash_context, hash);
	mbedtls_sha256_free(&hash_context);

	util_hash_to_string(hash_text, sizeof(hash), hash);

	string_format(call->result, "OK checksum: %s", string_cstr(hash_text));
}

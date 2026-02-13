#include "fs.h"

#include "ramdisk.h"
#include "crypt.h"
#include "exception.h"

#include <esp_littlefs.h>

#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <format>

FS *FS::singleton = nullptr;

FS::FS(Log &log_in) : log(log_in)
{
	esp_err_t rv;

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

	if(this->singleton)
		throw(hard_exception("FS: already active"));

	if((rv = esp_vfs_littlefs_register(&littlefs_parameters)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "FS::eps_vfs_littlefs_register: ")));

	this->singleton = this;
}

FS& FS::get()
{
	if(!FS::singleton)
		throw(hard_exception("FS::get: not active"));

	return(*FS::singleton);
}

void FS::list(std::string &out, const std::string &directory, bool option_long)
{
	DIR *dir;
	struct dirent *dirent;
	struct stat statb;
	std::string filename;
	std::string ctime;
	std::string mtime;
	int inode, length, allocated;

	if(!(dir = ::opendir(directory.c_str())))
		throw(transient_exception(std::format("opendir of {} failed", directory)));

	while((dirent = ::readdir(dir)))
	{
		filename = std::format("{}/{}", directory, dirent->d_name);

		if(::stat(filename.c_str(), &statb))
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
			ctime = Util::get().time_to_string(statb.st_ctim.tv_sec);
			mtime = Util::get().time_to_string(statb.st_mtim.tv_sec);
		}

		if(option_long)
			out += std::format("\n{:20} {:7d} {:4d}k {:19} {:19} {:11d}", dirent->d_name, length, allocated, ctime, mtime, inode);
		else
			out += std::format("\n{:3d}k {}", length / 1024UL, dirent->d_name);
	}

	::closedir(dir);
}

void FS::format(const std::string &mount)
{
	int fd;

	if(mount == "/littlefs")
	{
		if(esp_littlefs_format(mount.c_str()))
			throw(transient_exception(std::format("FS::format: littleFS format of {} failed", mount)));
	}
	else
	{
		if(mount == "/ramdisk")
		{
			if((fd = open("/ramdisk", O_RDONLY | O_DIRECTORY, 0)) < 0)
				throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::format: cannot open filesystem {}", mount))));

			if(ioctl(fd, IO_RAMDISK_WIPE, nullptr))
				throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::format: cannot format filesystem {}", mount))));

			close(fd);
		}
		else
			throw(transient_exception(std::format("FS::format: mountpount {} doesn't exist", mount)));
	}
}

int FS::read(std::string &out, const std::string &file, int position, int size)
{
	int fd, length;

	if((fd = ::open(file.c_str(), O_RDONLY, 0)) < 0)
		throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::read: cannot open file {}", file))));

	if(::lseek(fd, position, SEEK_SET) == -1)
		length = 0;
	else
	{
		out.resize(size);

		if((length = ::read(fd, out.data(), size)) == 0)
		{
			close(fd);
			throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::read: read from {} failed", file))));
		}

		out.resize(length);
	}

	close(fd);

	return(length);
}

int FS::write(const std::string &in, const std::string &file, bool append, int length)
{
	int fd;
	struct stat statb;
	int open_mode;

	open_mode = O_WRONLY | O_CREAT | (append ? O_APPEND : O_TRUNC);

	if(length != in.size())
		throw(hard_exception(std::format("FS::write: length parameter [{:d}] != data length [{:d}]", length, in.size())));

	if((fd = ::open(file.c_str(), open_mode, 0)) < 0)
		throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::write: cannot open file {}", file))));

	if(::write(fd, in.data(), in.size()) != length)
	{
		close(fd);
		throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::write: write to {} failed", file))));
	}

	close(fd);

	if(::stat(file.c_str(), &statb))
		length = -1;
	else
		length = statb.st_size;

	return(length);
}

void FS::erase(const std::string &file)
{
	if(::unlink(file.c_str()))
		throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::erase: unlink of {} failed", file))));
}

void FS::rename(const std::string &from, const std::string &to)
{
	if(::rename(from.c_str(), to.c_str()))
		throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::rename: rename of {} to {} failed", from, to))));
}

void FS::truncate(const std::string &file, int position)
{
	if(::truncate(file.c_str(), position))
		throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::truncate: truncate of {} failed", file))));
}

std::string FS::checksum(const std::string &file)
{
	int length, fd;
	Crypt::SHA256 md;
	std::string hash_text;
	std::string block;

	if((fd = open(file.c_str(), O_RDONLY, 0)) < 0)
		throw(transient_exception(this->log.errno_string_error(errno, std::format("FS::checksum: open {} failed", file))));

	md.init();

	block.resize(4096);

	while((length = ::read(fd, block.data(), block.size())) > 0)
	{
		block.resize(length);
		md.update(block);
		block.resize(4096);
	}

	close(fd);

	hash_text = Crypt::hash_to_text(md.finish());

	return(hash_text);
}

void FS::info(std::string &out)
{
	esp_err_t rv;
	size_t total, used, avail, usedpct;
	int fd;

	if((rv = esp_littlefs_info("littlefs", &total, &used)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "FS::info: esp_littlefs_info: ")));

	avail = total - used;
	usedpct = (100 * used) / total;

	out += "LITTLEFS";

	if(esp_littlefs_mounted("littlefs"))
		out += std::format(" mounted at /littlefs:\n- total size: {:d} kB\n- used: {:d} kB\n- available {:d} kB, {:d}% used",
				total / 1024, used / 1024, avail / 1024, usedpct);
	else
		out += " not mounted";

	if((fd = open("/ramdisk", O_RDONLY | O_DIRECTORY)) >= 0)
	{
		ioctl(fd, IO_RAMDISK_GET_SIZE, &total);
		ioctl(fd, IO_RAMDISK_GET_USED, &used);
		close(fd);

		avail = total - used;
		usedpct = (100 * used) / total;

		out += std::format("\nRAMDISK mounted at /ramdisk:\n- total size: {:d} kB\n- used: {:d} kB\n- available {:d} kB, {:d}% used",
				total / 1024, used / 1024, avail / 1024, usedpct);
	}
}

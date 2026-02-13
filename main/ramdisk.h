#pragma once

#include "log.h"

#include <time.h>
#include <dirent.h>

#include <freertos/FreeRTOS.h> // for SemaphoreHandle

#include <string>
#include <map>

namespace Ramdisk
{
	enum
	{
		IO_RAMDISK_GET_USED,
		IO_RAMDISK_SET_SIZE,
		IO_RAMDISK_GET_SIZE,
		IO_RAMDISK_WIPE,
	};

	class Mutex final
	{
		public:

			explicit Mutex() = delete;
			explicit Mutex(SemaphoreHandle_t *mutex);
			~Mutex();

		private:

			SemaphoreHandle_t *mutex;
	};

	class File final
	{
		public:

			std::string get_filename() const;
			unsigned int get_fileno() const;
			unsigned int get_length() const;
			unsigned int get_allocated() const;
			struct timespec get_ctime() const;
			struct timespec get_mtime() const;

			void time_update(bool update_ctime = false);
			int read(unsigned int offset, unsigned int size, uint8_t *data) const;
			int write(unsigned int offset, unsigned int length, const uint8_t *data);
			int truncate(unsigned int length);
			int rename(const std::string &filename);

			explicit File() = delete;
			explicit File(const std::string &filename, unsigned int fileno);

		private:

			typedef std::basic_string<uint8_t> Data;

			std::string filename;
			unsigned int fileno;
			struct timespec c_time;
			struct timespec m_time;
			Data contents;
	};

	class Dirent final
	{
		public:

			void set(int next_fileno, unsigned int fileno = 0, const std::string &filename = "");
			int get_next_fileno() const;
			DIR *get_DIR();
			struct dirent *get_dirent();

			explicit Dirent(int next_fileno = -1);

		private:

			struct dirent dirent;
			DIR dir;
			int next_fileno;
	};

	class Directory final
	{
		public:

			File *get_file_by_fileno(unsigned int fileno);
			const File *get_file_by_fileno_const(unsigned int fileno) const;
			File *get_file_by_name(const std::string &filename);
			const File *get_file_by_name_const(const std::string &filename) const;
			unsigned int get_used() const;

			int opendir(const std::string &path) const;
			int readdir(Dirent *dirent) const;
			int closedir(const Dirent *dirent) const;

			int open(const std::string &filename, int fcntl_flags, unsigned int new_fileno);
			int close(int fd) const;
			int read(unsigned int fileno, unsigned int offset, unsigned int size, uint8_t *data) const;
			int write(unsigned int fileno, unsigned int offset, unsigned int length, const uint8_t *data);

			int unlink(const std::string &path);
			int rename(const std::string &from, const std::string &to);

			int clear();

			explicit Directory() = delete;
			explicit Directory(const std::string &path);

		private:

			typedef std::map<unsigned int /*fileno*/, File> FileMap;

			std::string path;
			FileMap files;
	};

	class FileDescriptor final
	{
		public:

			unsigned int get_fd() const;
			unsigned int get_fileno() const;
			unsigned int get_fcntl_flags() const;
			unsigned int get_offset() const;
			bool is_fs() const;

			void set_offset(unsigned int offset);

			explicit FileDescriptor() = delete;
			explicit FileDescriptor(unsigned int fd, unsigned int fileno, unsigned int fcntl_flags, unsigned int offset, bool fs);

		private:

			unsigned int fd;
			unsigned int fileno;
			unsigned int fcntl_flags;
			unsigned int offset;
			bool fs;
	};

	class Root final
	{
		public:

			explicit Root() = delete;
			explicit Root(const Root &) = delete;
			explicit Root(Log &, const std::string &mountpoint, unsigned int size);

			static Root &get();

		private:

			static constexpr unsigned int fd_max = 8;

			typedef std::map<unsigned int /* file descriptor index */, FileDescriptor> FileDescriptorTable;
			typedef std::map<DIR *, Dirent *> DirentTable;

			static Root *singleton;

			Log &log;
			std::string mountpoint;
			unsigned int size;
			Directory root;
			unsigned int last_fileno;

			FileDescriptorTable fd_table;
			DirentTable dirent_table;
			SemaphoreHandle_t mutex;

			DIR *opendir(const std::string &name);
			struct dirent *readdir(DIR *pdir);
			int closedir(DIR *pdir);
			int stat(const std::string &path, struct stat *st);
			int fstat(int fd, struct stat *st);
			int ioctl(int fd, int op, int *intp);
			int open(const std::string &path, int fcntl_flags);
			int close(int fd);
			int read(int fd, unsigned int size, uint8_t *data);
			int write(int fd, unsigned int length, const uint8_t *data);
			int lseek(int fd, unsigned int mode, int offset);
			int truncate(const std::string &path, unsigned int length);
			int ftruncate(int fd, unsigned int length);
			int unlink(const std::string &path);
			int rename(const std::string &from, const std::string &to);

			static DIR *static_opendir(void *context, const char *name);
			static struct dirent *static_readdir(void *context, DIR *pdir);
			static int static_closedir(void *context, DIR *pdir);
			static int static_stat(void *context, const char *path, struct stat *st);
			static int static_fstat(void *context, int fd, struct stat *st);
			static int static_ioctl(void *context, int fd, int op, va_list);
			static int static_open(void *context, const char *path, int fcntl_flags, int file_access_mode);
			static int static_close(void *context, int fd);
			static int static_read(void *context, int fd, void *data, size_t size);
			static int static_write(void *context, int fd, const void *data, size_t size);
			static off_t static_lseek(void *context, int fd, off_t size, int mode);
			static int static_truncate(void *context, const char *path, off_t length);
			static int static_ftruncate(void *context, int fd, off_t length);
			static int static_unlink(void *context, const char *path);
			static int static_rename(void *context, const char *from, const char *to);

			bool file_in_use(const std::string &filename, unsigned int fcntl_flags) const;
			void all_stat(const File *fp, struct stat *st) const;
	};
};

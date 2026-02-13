#pragma once

#include "log.h"

#include <string>

class FS final
{
	public:

		explicit FS() = delete;
		explicit FS(const FS &) = delete;
		explicit FS(Log &);

		static FS& get();

		void list(std::string &out, const std::string &directory, bool option_long);
		void format(const std::string &mount);
		int read(std::string &out, const std::string &file, int position, int size);
		int write(const std::string &in, const std::string &file, bool append, int length);
		void erase(const std::string &file);
		void rename(const std::string &from, const std::string &to);
		void truncate(const std::string &file, int position);
		std::string checksum(const std::string &file);
		void info(std::string &out);

	private:

		static FS *singleton;
		Log &log;
};

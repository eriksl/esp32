#pragma once

#include "log.h"

#include <string>
#include <map>

#include "driver/sdm.h"

#include "magic_enum/magic_enum.hpp"
#include "magic_enum/magic_enum_containers.hpp"

class PDM final
{
	public:

		static constexpr int channels_size = 4;

		enum class Channel : unsigned int
		{
			channel_8bit_150khz_0 = 0,
			channel_8bit_150khz_1,
			channel_8bit_150khz_2,
			channel_8bit_150khz_3,
		};

		explicit PDM() = delete;
		explicit PDM(Log &);
		explicit PDM(const PDM &) = delete;

		static PDM& get();

		void open(Channel channel, std::string_view owner);
		void set(Channel channel, int density);
		int get(Channel channel);
		void info(std::string &);

	private:

		static constexpr int pdm_sample_frequency = 80000000 / 256;

		struct handle_t
		{
			std::string owner;
			int density;
			sdm_channel_handle_t handle;
			struct
			{
				unsigned int available:1;
				unsigned int open:1;
			};
		};

		Log &log;

		static PDM *singleton;

		static const std::map<Channel, int> channel_to_gpio;
		std::map<Channel, handle_t> handles;
};

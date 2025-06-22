#include <string>
#include <array>
#include <vector>
#include <deque>
#include <fstream>
#include <thread>

extern "C"
{
#include "string.h"
#include "cli-command.h"
#include "log.h"
#include "script.h"
}

#include <stdint.h>
#include <string.h>

extern "C"
{
#include "util.h"
}

#include <esp_pthread.h>

static constexpr unsigned int parameter_size = 4;

class ThreadState
{
	public:

		std::string script;
		std::ifstream file;
		std::array<std::string, parameter_size> parameter;
};

typedef std::deque<ThreadState *> ThreadStates;

static void script_run(ThreadState *initial_thread_state)
{
	try
	{
		ThreadStates thread_states;
		ThreadState *thread_state;

		cli_buffer_t cli_buffer;
		std::string line, command, expanded_line;
		unsigned int ix, parameter_index;
		size_t pos, start, end;

		initial_thread_state->file.open((std::string("/ramdisk/") + initial_thread_state->script).c_str());

		if(initial_thread_state->file.fail())
		{
			initial_thread_state->file.open((std::string("/littlefs/") + initial_thread_state->script).c_str());

			if(initial_thread_state->file.fail())
				throw(std::string("script ") + initial_thread_state->script + " not found");
		}

		thread_states.push_front(initial_thread_state);

		while(thread_states.size() > 0)
		{
			thread_state = thread_states.front();
			thread_states.pop_front();

			while(std::getline(thread_state->file, line))
			{
				if(line.back() == '\n')
					line.pop_back();

				expanded_line.clear();

				for(ix = 0; ix < line.length(); ix++)
				{
					if(line.at(ix) != '$') 
					{
						expanded_line.append(1, line.at(ix));
						continue;
					}

					if((ix + 1) >= line.length())
						continue;

					if((line.at(ix + 1) >= '0') && (line.at(ix + 1) <= '9'))
					{
						parameter_index = (unsigned int)line.at(ix + 1) - '0';
						ix++;

						if(parameter_index >= parameter_size)
							continue;

						expanded_line.append(thread_state->parameter.at(parameter_index));
					}
				}

				if((pos = expanded_line.find(' ')) == std::string::npos)
					command = expanded_line;
				else
					command = expanded_line.substr(0, pos);

				if(command == "stop")
				{
					log_format("%s: STOP", thread_state->script.c_str());
					break;
				}

				if(command == "call")
				{
					thread_states.push_front(thread_state);

					thread_state = new ThreadState;

					for(start = expanded_line.find(' '); (start != std::string::npos) && (start < expanded_line.length()) && (expanded_line.at(start) == ' '); start++)
						(void)0;

					end = expanded_line.find(' ', start);

					if(end == std::string::npos)
					{
						thread_state->script = expanded_line.substr(start);
						start = std::string::npos;
					}
					else
					{
						thread_state->script = expanded_line.substr(start, end - start);
						start = end + 1;
					}

					for(parameter_index = 0; (parameter_index < parameter_size) && (start != std::string::npos) && (start < expanded_line.length()); parameter_index++)
					{
						end = expanded_line.find(' ', start);

						if(end == std::string::npos)
						{
							thread_state->parameter[parameter_index] = expanded_line.substr(start);
							start = std::string::npos;
						}
						else
						{
							thread_state->parameter[parameter_index] = expanded_line.substr(start, end - start);
							start = end + 1;
						}
					}

					thread_state->file.open((std::string("/ramdisk/") + thread_state->script).c_str());

					if(thread_state->file.fail())
					{
						thread_state->file.open((std::string("/littlefs/") + thread_state->script).c_str());

						if(thread_state->file.fail())
							throw(std::string("script ") + initial_thread_state->script + " in call statement not found");
					}

					continue;
				}

				if(command == "pause")
				{
					unsigned int sleep_msec;

					if(pos == std::string::npos)
						sleep_msec = 1000;
					else
					{
						try
						{
							sleep_msec = static_cast<unsigned int>(std::stof(expanded_line.substr(pos)) * 1000.0f);
						}
						catch(...)
						{
							sleep_msec = 1000;
						}
					}

					if(sleep_msec >= 10)
						util_sleep(sleep_msec);

					continue;
				}

				if(command == "repeat")
				{
					thread_state->file.seekg(0);
					util_sleep(100);
					continue;
				}

				cli_buffer.source = cli_source_script;
				cli_buffer.mtu = 0;
				cli_buffer.packetised = 0;
				cli_buffer.data = string_new(expanded_line.length());
				string_assign_cstr(cli_buffer.data, expanded_line.c_str());
				strlcpy(cli_buffer.script.name, thread_state->script.c_str(), sizeof(cli_buffer.script.name));
				cli_buffer.script.task = xTaskGetCurrentTaskHandle();

				cli_receive_queue_push(&cli_buffer);

				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}

			delete thread_state;
		}
	}
	catch(const std::string &cause)
	{
		log_format("run script: %s", cause.c_str());
	}
	catch(...)
	{
		log_format("run script: unknown exeception");
	}
}

void command_run(cli_command_call_t *call)
{
	auto *thread_state = new ThreadState;
	unsigned int ix;
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	assert((call->parameter_count > 0) && (call->parameter_count <= (parameter_size + 1)));

	thread_state->script = string_cstr(call->parameters[0].string);

	for(ix = 0; (ix + 1) < call->parameter_count; ix++)
		thread_state->parameter[ix] = string_cstr(call->parameters[ix + 1].string);

	thread_config.thread_name = string_cstr(call->parameters[0].string);
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 3 * 1024;
	thread_config.prio = 1;
	esp_pthread_set_cfg(&thread_config);

	std::thread new_thread(script_run, thread_state);

	new_thread.detach();
}

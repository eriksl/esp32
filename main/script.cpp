#include <string>
#include <array>
#include <vector>
#include <deque>
#include <fstream>
#include <thread>

#include "string.h"
#include "cli-command.h"
#include "log.h"
#include "script.h"
#include <stdint.h>
#include <string.h>
#include "util.h"

#include <esp_pthread.h>

static constexpr unsigned int parameter_size = 4;

class ThreadState
{
	public:

		std::string script;
		std::ifstream file;
		struct
		{
			bool active;
			unsigned int target;
			unsigned int current;
		} repeat;
		std::array<std::string, parameter_size> parameter;
};

typedef std::deque<ThreadState *> ThreadStates;

static void script_run(ThreadState *initial_thread_state)
{
	try
	{
		ThreadStates thread_states;
		ThreadState *thread_state;

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
					{
						expanded_line.append(1, '$');
						continue;
					}

					if((line.at(ix + 1) >= '0') && (line.at(ix + 1) <= '3'))
					{
						parameter_index = (unsigned int)line.at(ix + 1) - '0';

						if(parameter_index >= parameter_size)
							continue;

						expanded_line.append(thread_state->parameter.at(parameter_index));
						ix++;
						continue;
					}

					if(line.at(ix + 1) == 'r')
					{
						expanded_line.append(std::to_string(thread_state->repeat.current));
						ix++;
						continue;
					}

					if(line.at(ix + 1) == 'R')
					{
						expanded_line.append(std::to_string(thread_state->repeat.target));
						ix++;
						continue;
					}

					expanded_line.append(1, line.at(ix));
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

					thread_state = new ThreadState();

					thread_state->repeat.active = false;
					thread_state->repeat.target = 0;
					thread_state->repeat.current = 0;

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
					if(thread_state->repeat.active)
					{
						if((thread_state->repeat.target != 0) && (thread_state->repeat.current++ >= thread_state->repeat.target))
						{
							thread_state->repeat.target = 0;
							thread_state->repeat.current = 0;
							thread_state->repeat.active = false;
						}
						else
						{
							thread_state->file.seekg(0);
							util_sleep(100);
						}
					}
					else
					{
						unsigned int target = 0;

						if(pos != std::string::npos)
						{
							try
							{
								target = static_cast<unsigned int>(std::stoul(expanded_line.substr(pos)));
							}
							catch(...)
							{
								target = 0;
							}
						}

						thread_state->repeat.target = target;
						thread_state->repeat.current = 1;
						thread_state->repeat.active = true;
						thread_state->file.seekg(0);
						util_sleep(100);
					}

					continue;
				}

				command_response_t *command_response = new command_response_t;

				command_response->source = cli_source_script;
				command_response->mtu = 0;
				command_response->packetised = 0;
				command_response->packet = expanded_line;
				strlcpy(command_response->script.name, thread_state->script.c_str(), sizeof(command_response->script.name)); // FIXME
				command_response->script.task = xTaskGetCurrentTaskHandle();

				cli_receive_queue_push(command_response);

				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

				command_response = nullptr;
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
	auto *thread_state = new ThreadState();
	unsigned int ix;
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	assert((call->parameter_count > 0) && (call->parameter_count <= (parameter_size + 1)));

	thread_state->repeat.active = false;
	thread_state->repeat.target = 0;
	thread_state->repeat.current = 0;
	thread_state->script = call->parameters[0].str.c_str();

	for(ix = 0; (ix + 1) < call->parameter_count; ix++)
		thread_state->parameter[ix] = call->parameters[ix + 1].str.c_str();

	thread_config.thread_name = call->parameters[0].str.c_str();
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 4 * 1024;
	thread_config.prio = 1;
	thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM;
	esp_pthread_set_cfg(&thread_config);

	std::thread new_thread(script_run, thread_state);

	new_thread.detach();
}

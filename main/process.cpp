#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <esp_log.h>
#include <esp_system.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "process.h"

#include <string>
#include <boost/format.hpp>

enum
{
	task_id_size = 48,
};

typedef struct
{
	unsigned int task_id;
	int64_t previous_runtime;
} task_info_cache_t;

static task_info_cache_t *task_info_cache;
static bool inited = false;

void command_process_list(cli_command_call_t *call)
{
	unsigned int ix, processes, current_task_id, done;
	int requested_core;
	uint64_t runtime;
	TaskStatus_t *process_info;
	task_info_cache_t *task_info_cache_ptr;
	const TaskStatus_t *pip;
	const char *name;
	const char *state;
	const char *core_string;
	int64_t delta, total_delta_idle, total_delta_active;
	bool idle_task;

	assert(inited);
	assert(call->parameter_count < 2);

	if(call->parameter_count == 1)
		requested_core = (int)call->parameters[0].unsigned_int;
	else
		requested_core = -1;

	processes = uxTaskGetNumberOfTasks();
	process_info = (TaskStatus_t *)util_memory_alloc_spiram(sizeof(*process_info) * processes);
	assert(uxTaskGetSystemState(process_info, processes, &runtime) == processes);

	total_delta_idle = 0;
	total_delta_active = 0;

	for(ix = 0; ix < processes; ix++)
	{
		pip = &process_info[ix];

		assert(pip->xTaskNumber < task_id_size);

		task_info_cache_ptr = &task_info_cache[pip->xTaskNumber];

		assert(task_info_cache_ptr->task_id == pip->xTaskNumber);

		if(strncmp(pip->pcTaskName, "IDLE", 4))
			total_delta_active += pip->ulRunTimeCounter - task_info_cache_ptr->previous_runtime;
		else
			total_delta_idle   += pip->ulRunTimeCounter - task_info_cache_ptr->previous_runtime;
	}

	call->result += (boost::format("threads: %u, active: %lld%%, idle: %lld%%\n") %
			processes %
			(total_delta_active * 100 / (total_delta_active + total_delta_idle)) %
			(total_delta_idle   * 100 / (total_delta_active + total_delta_idle))).str();;

	call->result += (boost::format("  %2s  %-14s %4s %-10s %4s %5s %10s %10s %8s\n") % "#" % "name" % "core" % "state" % "prio" % "stack" % "runtime" % "delta" % "active_%").str();;

	done = 0;

	for(current_task_id = 0; (current_task_id < task_id_size) && (done < processes); current_task_id++)
	{
		for(ix = 0; ix < processes; ix++)
		{
			pip = &process_info[ix];

			assert(pip->xTaskNumber < task_id_size);

			if(pip->xTaskNumber != current_task_id)
				continue;

			done++;

			if((requested_core >= 0) && ((pip->xCoreID == 0) || (pip->xCoreID == 1)) && (pip->xCoreID != requested_core))
				continue;

			task_info_cache_ptr = &task_info_cache[current_task_id];

			assert(task_info_cache_ptr->task_id == current_task_id);

			name = pip->pcTaskName ? pip->pcTaskName : "(null)";

			switch(pip->xCoreID)
			{
				case(0): { core_string = "0"; break; };
				case(1): { core_string = "1"; break; };
				default: { core_string = "both"; break; };
			}

			switch(pip->eCurrentState)
			{
				case(eRunning):
				{
					state = "running";
					break;
				}

				case(eReady):
				{
					state = "ready";
					break;
				}

				case(eBlocked):
				{
					state = "blocked";
					break;
				}

				case(eSuspended):
				{
					state = "suspended";
					break;
				}

				case(eDeleted):
				{
					state = "deleted";
					break;
				}

				default:
				{
					state = "invalid";
					break;
				}
			}

			idle_task = !strncmp(pip->pcTaskName, "IDLE", 4);

			delta = pip->ulRunTimeCounter - task_info_cache_ptr->previous_runtime;

			call->result += (boost::format("  %2u: %-14s %4s %-10s %4u %5lu %10llu %10lld %8llu\n") %
					current_task_id %
					name %
					core_string %
					state %
					pip->uxCurrentPriority %
					pip->usStackHighWaterMark %
					pip->ulRunTimeCounter %
					delta %
					(idle_task ? 0 : ((delta * 100ULL) / total_delta_active))).str();

			task_info_cache_ptr->previous_runtime = pip->ulRunTimeCounter;
		}
	}

	free(process_info);
}

void command_process_kill(cli_command_call_t *call)
{
	unsigned int ix, processes, target_task_id;
	TaskStatus_t *process_info;
	uint64_t runtime;
	const TaskStatus_t *pip;
	bool found;
	char taskname[16];

	assert(inited);
	assert(call->parameter_count == 1);

	try
	{
		target_task_id = std::stoul(call->parameters[0].str);
	}
	catch(...)
	{
		target_task_id = ~0UL;
	}

	processes = uxTaskGetNumberOfTasks();
	process_info = (TaskStatus_t *)util_memory_alloc_spiram(sizeof(*process_info) * processes);
	assert(uxTaskGetSystemState(process_info, processes, &runtime) == processes);

	for(ix = 0, found = false; ix < processes; ix++)
	{
		pip = &process_info[ix];
		assert(pip->xTaskNumber < task_id_size);

		if(target_task_id == ~0UL)
		{
			if(call->parameters[0].str == pip->pcTaskName)
			{
				found = true;
				break;
			}
		}
		else
		{
			if(pip->xTaskNumber == target_task_id)
			{
				found = true;
				break;
			}
		}
	}

	if(found && (ix < processes))
	{
		target_task_id = pip->xTaskNumber;
		strlcpy(taskname, pip->pcTaskName, sizeof(taskname));

		vTaskDelete(pip->xHandle);
		log_format("process #%u: \"%s\" killed", target_task_id, taskname);
	}
	else
	{
		if(target_task_id == ~0UL)
			log_format("process \"%s\" not found", call->parameters[0].str.c_str());
		else
			log_format("process #%u not found", target_task_id);
	}

	free(process_info);
}

void process_init(void)
{
	unsigned int ix;
	task_info_cache_t *task_info_cache_ptr;

	assert(!inited);

	task_info_cache = (task_info_cache_t *)util_memory_alloc_spiram(sizeof(task_info_cache_t[task_id_size]));
	assert(task_info_cache);

	for(ix = 0; ix < task_id_size; ix++)
	{
		task_info_cache_ptr = &task_info_cache[ix];
		task_info_cache_ptr->task_id = ix;
		task_info_cache_ptr->previous_runtime = 0;
	}

	inited = true;
}

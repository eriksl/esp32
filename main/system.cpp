#include "system.h"

#include "log.h"
#include "display.h"
#include "exception.h"
#include "crypt.h"

#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <esp_app_desc.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <format>

#include <netinet/in.h>
#include <sys/socket.h>

System *System::singleton = nullptr;

const System::ipv6_address_type_map_t System::ipv6_address_type_strings =
{
	{ IPV6AddressType::loopback,		"loopback"		},
	{ IPV6AddressType::link_local,		"link local"	},
	{ IPV6AddressType::multicast,		"multicast"		},
	{ IPV6AddressType::site_local,		"site local"	},
	{ IPV6AddressType::ipv4_mapped,		"ipv4 mapped"	},
	{ IPV6AddressType::unspecified,		"unspecified"	},
	{ IPV6AddressType::global_slaac,	"slaac"			},
	{ IPV6AddressType::global_static,	"static"		},
	{ IPV6AddressType::other,			"other"			},
};

System::System(Log &log_in) : log(log_in)
{
	int ix;

	if(this->singleton)
		throw(hard_exception("System: already active"));

	this->initial_free_heap = esp_get_free_heap_size();
	this->initial_free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
	this->initial_free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
	this->initial_free_total = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
	this->initial_free_rtcram = heap_caps_get_free_size(MALLOC_CAP_RTCRAM);

	for(ix = 0; ix < this->task_info_cache.size(); ix++)
	{
		this->task_info_cache[ix].task_id = ix;
		this->task_info_cache[ix].previous_runtime = 0;
	}

	this->singleton = this;
}

System& System::get()
{
	if(!System::singleton)
		throw(hard_exception("System: not active"));

	return(*System::singleton);
}

void System::identify(std::string &out, int mtu)
{
	out += std::format("firmware date: {} {}, ", __DATE__, __TIME__);
	out += std::format("transport mtu: {:d}, ", mtu);
    out += std::format("display area: {:d}x{:d}", display_image_x_size(), display_image_y_size());
}

void System::info(std::string &out)
{
	const esp_app_desc_t *desc;
	esp_chip_info_t chip_info;
	uint32_t flash_size;

	if(!(desc = esp_app_get_description()))
		throw(hard_exception("System::info: esp_app_get_description failed"));

	esp_chip_info(&chip_info);

	out += std::format("SoC: {} with {:d} cores\nRF: {}{}{}{}",
		CONFIG_IDF_TARGET,
		chip_info.cores,
		((chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : ""),
		((chip_info.features & CHIP_FEATURE_BT) ? "BT" : ""),
		((chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : ""),
		((chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : ""));

	int major_rev = chip_info.revision / 100;
	int minor_rev = chip_info.revision % 100;
	out += std::format("\nRevision: {:d}.{:d}", major_rev, minor_rev);

	esp_flash_get_size(nullptr, &flash_size);

	out += std::format("\nFlash: {:d} MB {}",
		flash_size / (1024 * 1024),
		(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	out += std::format("\nBSP:\n- board name: {}\n- flash size: {:d} MB\n- SPI RAM size: {:d} MB",
			CONFIG_BSP_BOARD_TYPE_NAME,
			CONFIG_BSP_FLASH_SIZE / 1024,
			CONFIG_BSP_SPIRAM_SIZE / 1024);
#if (CONFIG_BSP_LEDPIXEL0 >= 0)
	out += std::format("\n- LEDpixel at GPIO {:d}", CONFIG_BSP_LEDPIXEL0);
#else
	out += "\n- no LEDpixel";
#endif
#if (CONFIG_BSP_LEDPWM0 >= 0)
	out += std::format("\n- status LED at GPIO {:d}", CONFIG_BSP_LEDPWM0);
#else
	out += "\n- no status LED";
#endif
	out += std::format("\nfirmware\n- date: {} {}\n- build start: {} {}\n",
			__DATE__, __TIME__, desc->date, desc->time);
}

void System::memory(std::string &out)
{
	unsigned int free_total;

	free_total = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

	out += "MEMORY";
	out += "\namount kB:";
	out += std::format("\n- {:<29s} {:5d} / {:5d} kB",		"free heap total",				esp_get_free_heap_size() / 1024,						this->initial_free_heap / 1024);
	out += std::format("\n- {:<29s} {:5d} kB",				"minimum free heap",			esp_get_minimum_free_heap_size() / 1024);
	out += std::format("\n- {:<29s} {:5d} kB",				"heap executable",				heap_caps_get_free_size(MALLOC_CAP_EXEC) / 1024);
	out += std::format("\n- {:<29s} {:5d} kB",				"heap 32 bit addressable",		heap_caps_get_free_size(MALLOC_CAP_32BIT) / 1024);
	out += std::format("\n- {:<29s} {:5d} kB",				"heap 8 bit addressable",		heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024);
	out += std::format("\n- {:<29s} {:5d} kB",				"heap DMA adressable",			heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024);
	out += std::format("\n- {:<29s} {:5d} / {:5d} kB",		"heap SPI RAM",					heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024, 		this->initial_free_spiram / 1024);
	out += std::format("\n- {:<29s} {:5d} / {:5d} kB",		"heap internal RAM",			heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,	this->initial_free_internal / 1024);
	out += std::format("\n- {:<29s} {:5d} / {:5d} kB",		"heap default",					free_total / 1024,										this->initial_free_total / 1024);
	out += std::format("\n- {:<29s} {:5d} kB",				"heap IRAM 8 bit adressable",	heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT) / 1024);
	out += std::format("\n- {:<29s} {:5d} kB",				"heap retention",				heap_caps_get_free_size(MALLOC_CAP_RETENTION) / 1024);
	out += std::format("\n- {:<29s} {:5d} / {:5d} kB",		"heap RTC RAM",					heap_caps_get_free_size(MALLOC_CAP_RTCRAM) / 1024,		this->initial_free_rtcram / 1024);
	out += std::format("\n- {:<29s} {:5d} kB",				"heap TCM",						heap_caps_get_free_size(MALLOC_CAP_TCM) / 1024);
	out += "\nusage:";
	out += std::format("\n- total: {:6.3f} MB",				this->initial_free_total / (1024.0 * 1024.0));
	out += std::format("\n- used:  {:6.3f} MB {:4.1f}%",	(this->initial_free_total - free_total) / (1024.0 * 1024.0), (100.0 * (this->initial_free_total - free_total)) / this->initial_free_total);
	out += std::format("\n- free:  {:6.3f} MB {:4.1f}%",	free_total / (1024.0 * 1024.0), (100.0 * free_total) / this->initial_free_total);
}

void System::partitions(std::string &out, int match_partition)
{
	int rv;
	esp_partition_iterator_t partition_iterator;
	const esp_partition_t *partition, *boot_partition, *running_partition;
	std::string type;
	std::string subtype;
	unsigned int index;
	esp_ota_img_states_t ota_state;
	const char *ota_state_text;
	std::string sha256_hash;
	std::string sha256_hash_text;

	if(!(boot_partition = esp_ota_get_boot_partition()))
		throw(hard_exception("System::partitions: esp_ota_get_boot_partition failed"));

	if(!(running_partition = esp_ota_get_running_partition()))
		throw(hard_exception("System::partitions: esp_ota_get_running_partition failed"));

	if(!(partition_iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, (const char *)0)))
		throw(hard_exception("System::partitions: esp_partition_find failed"));

	out += "Partitions:";

	for(index = 0; partition_iterator; index++, partition_iterator = esp_partition_next(partition_iterator))
	{
		if((match_partition >= 0) && (match_partition != index))
			continue;

		if(!(partition = esp_partition_get(partition_iterator)))
			throw(hard_exception("System::partitions: esp_partition_get failed"));

		switch(partition->type)
		{
			case(ESP_PARTITION_TYPE_APP):
			{
				type = "app";
				break;
			}

			case(ESP_PARTITION_TYPE_DATA):
			{
				type = "data";
				break;
			}

			default:
			{
				type = "unknown";
				break;
			}
		}

		switch(partition->subtype)
		{
			case(ESP_PARTITION_SUBTYPE_APP_FACTORY):
			//case(ESP_PARTITION_SUBTYPE_DATA_OTA): // these are using the same value
			{
				if(partition->type == ESP_PARTITION_TYPE_APP)
					subtype = "factory";
				else
					subtype = "ota data";

				break;
			}

			case(ESP_PARTITION_SUBTYPE_APP_OTA_0):
			{
				subtype = "ota 0";
				break;
			}

			case(ESP_PARTITION_SUBTYPE_APP_OTA_1):
			{
				subtype = "ota 1";
				break;
			}

			case(ESP_PARTITION_SUBTYPE_DATA_NVS):
			{
				subtype = "nvs";
				break;
			}

			default:
			{
				subtype = "unknown";
				break;
			}
		}

		ota_state_text = "";

		if((partition->type == ESP_PARTITION_TYPE_APP) &&
			((partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) || (partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)))
		{
			if(!esp_ota_get_state_partition(partition, &ota_state))
			{
				switch(ota_state)
				{
					case(ESP_OTA_IMG_NEW):				{ ota_state_text = "N"; break; }
					case(ESP_OTA_IMG_PENDING_VERIFY):	{ ota_state_text = "P"; break; }
					case(ESP_OTA_IMG_VALID):			{ ota_state_text = "V"; break; }
					case(ESP_OTA_IMG_INVALID):			{ ota_state_text = "I"; break; }
					case(ESP_OTA_IMG_ABORTED):			{ ota_state_text = "A"; break; }
					case(ESP_OTA_IMG_UNDEFINED):		{ ota_state_text = "U"; break; }
					default:							{ ota_state_text = "?"; break; }
				}
			}
			else
				ota_state_text = "X";
		}

		sha256_hash.resize(32);

		if((rv = esp_partition_get_sha256(partition, reinterpret_cast<uint8_t *>(sha256_hash.data()))))
			sha256_hash_text = "<invalid>";
		else
			sha256_hash_text = Crypt::hash_to_text(sha256_hash);

		out += std::format("\n  {:2d} {:1s}{}{} {:<8s} {:06x} {:4d} {:<7s} {:<8s} {}",
				index,
				ota_state_text,
				(partition->address == boot_partition->address) ? "b" : " ",
				(partition->address == running_partition->address) ? "r" : " ",
				partition->label,
				partition->address,
				partition->size / 1024,
				type,
				subtype,
				sha256_hash_text);
	}

	esp_partition_iterator_release(partition_iterator);
}

void System::process_list(std::string &out, int requested_core)
{
	unsigned int ix, processes, reported_processes, current_task_id, done;
	std::uint64_t runtime;
	TaskStatus_t *process_info;
	task_info_cache_t *task_info_cache_ptr;
	const TaskStatus_t *pip;
	std::string process_name;
	std::string process_state;
	std::string core_string;
	std::int64_t delta, total_delta_idle, total_delta_active;
	bool idle_task;

	processes = uxTaskGetNumberOfTasks();
	process_info = new TaskStatus_t[processes];
	reported_processes = uxTaskGetSystemState(process_info, processes, &runtime);

	try
	{
		if(reported_processes != processes)
			throw(hard_exception("uxTaskGetSystemState failed"));

		total_delta_idle = 0;
		total_delta_active = 0;

		for(ix = 0; ix < processes; ix++)
		{
			pip = &process_info[ix];

			if(pip->xTaskNumber >= this->task_id_size)
				throw(hard_exception("task number > task size"));

			task_info_cache_ptr = &this->task_info_cache[pip->xTaskNumber];

			if(task_info_cache_ptr->task_id != pip->xTaskNumber)
				throw(hard_exception("task number != task id"));

			if((pip->pcTaskName[0] == 'I') &&
					(pip->pcTaskName[1] == 'D') &&
					(pip->pcTaskName[2] == 'L') &&
					(pip->pcTaskName[3] == 'E'))
				total_delta_active += pip->ulRunTimeCounter - task_info_cache_ptr->previous_runtime;
			else
				total_delta_idle += pip->ulRunTimeCounter - task_info_cache_ptr->previous_runtime;
		}

		out += std::format("processes: {:d}, active: {:d}%%, idle: {:d}%\n",
				processes,
				total_delta_active * 100 / (total_delta_active + total_delta_idle),
				total_delta_idle   * 100 / (total_delta_active + total_delta_idle));

		out += std::format("  {:2s}: {:<14s} {:2s} {:<10s} {:4s} {:5s} {:>10s} {:>10s} {:>8s}\n",
				"##",
				"name",
				"co",
				"state",
				"prio",
				"stack",
				"runtime",
				"delta",
				"active_%");

		done = 0;

		for(current_task_id = 0; (current_task_id < this->task_id_size) && (done < processes); current_task_id++)
		{
			for(ix = 0; ix < processes; ix++)
			{
				pip = &process_info[ix];

				if(pip->xTaskNumber >= this->task_id_size)
					throw(hard_exception("pip->xTaskNumber >= this->task_id_size"));

				if(pip->xTaskNumber != current_task_id)
					continue;

				done++;

				if((requested_core >= 0) && ((pip->xCoreID == 0) || (pip->xCoreID == 1)) && (pip->xCoreID != requested_core))
					continue;

				task_info_cache_ptr = &this->task_info_cache[current_task_id];

				if(task_info_cache_ptr->task_id != current_task_id)
					throw(hard_exception("task_info_cache_ptr->task_id != current_task_id"));

				process_name = pip->pcTaskName ? pip->pcTaskName : "(null)";

				switch(pip->xCoreID)
				{
					case(0): { core_string = "0 "; break; };
					case(1): { core_string = " 1"; break; };
					default: { core_string = "01"; break; };
				}

				switch(pip->eCurrentState)
				{
					case(eRunning):
					{
						process_state = "running";
						break;
					}

					case(eReady):
					{
						process_state = "ready";
						break;
					}

					case(eBlocked):
					{
						process_state = "blocked";
						break;
					}

					case(eSuspended):
					{
						process_state = "suspended";
						break;
					}

					case(eDeleted):
					{
						process_state = "deleted";
						break;
					}

					default:
					{
						process_state = "invalid";
						break;
					}
				}

				idle_task = !strncmp(pip->pcTaskName, "IDLE", 4);

				delta = pip->ulRunTimeCounter - task_info_cache_ptr->previous_runtime;

				out += std::format("  {:2d}: {:<14s} {:2s} {:<10s} {:4d} {:5d} {:10d} {:10d} {:8d}\n",
						current_task_id,
						process_name,
						core_string,
						process_state,
						pip->uxCurrentPriority,
						pip->usStackHighWaterMark,
						pip->ulRunTimeCounter,
						delta,
						idle_task ? 0 : ((delta * 100ULL) / total_delta_active));

				task_info_cache_ptr->previous_runtime = pip->ulRunTimeCounter;
			}
		}
	}
	catch(const hard_exception &e)
	{
		delete [] process_info;
		throw(hard_exception(std::format("System::process_list: {}", e.what())));
	}

	delete [] process_info;
}

bool System::process_kill(const std::string &name)
{
	int target_task_id;
	unsigned int ix, reported_processes, processes;
	TaskStatus_t *process_info;
	uint64_t runtime;
	const TaskStatus_t *pip;
	bool found;
	std::string taskname;

	try
	{
		target_task_id = std::stoi(name);
	}
	catch(...)
	{
		target_task_id = -1;
	}

	processes = uxTaskGetNumberOfTasks();
	process_info = new TaskStatus_t[processes];

	try
	{
		reported_processes = uxTaskGetSystemState(process_info, processes, &runtime);

		if(reported_processes != processes)
			throw(hard_exception("uxTaskGetSystemState failed"));

		for(ix = 0, found = false; ix < processes; ix++)
		{
			pip = &process_info[ix];

			if(pip->xTaskNumber >= this->task_id_size)
				throw(hard_exception("pip->xTaskNumber >= this->task_id_size"));

			if(target_task_id == -1)
			{
				if(name == pip->pcTaskName)
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
			taskname = pip->pcTaskName;

			vTaskDelete(pip->xHandle);
			log << std::format("process #{:d}: \"{}\" killed", target_task_id, taskname);
		}
		else
		{
			if(target_task_id == -1)
				log << std::format("process \"{}\" not found", name);
			else
				log << std::format("process #{:d} not found", target_task_id);
		}
	}
	catch(const hard_exception &e)
	{
		delete [] process_info;
		throw(hard_exception(std::format("System::process_kill: {}", e.what())));
	}

	delete [] process_info;

	return(found);
}

int System::get_initial_free_heap()
{
	return(this->initial_free_heap);
}

int System::get_initial_free_spiram()
{
	return(this->initial_free_spiram);
}

int System::get_initial_free_internal()
{
	return(this->initial_free_internal);
}

int System::get_initial_free_total()
{
	return(this->initial_free_total);
}

int System::get_initial_free_rtcram()
{
	return(this->initial_free_rtcram);
}

std::string System::ipv4_addr_to_string(const uint32_t *in /* sockaddr_in->sin_addr.in_addr = uint32_t */)
{
	std::string dst;
	const char *c_dst;

	if(!in)
		throw(hard_exception("System::ipv4_addr_to_string: nullptr"));

	dst.resize(INET_ADDRSTRLEN);
	c_dst = inet_ntop(AF_INET, reinterpret_cast<const void *>(in), dst.data(), dst.size());
	dst.resize(strlen(c_dst));

	return(dst);
}

System::IPV6AddressType System::ipv6_address_type(const void *in /* sockaddr6_in->sin6_addr.s6_addr = char[16] */)
{
	struct in6_addr *s6addr = reinterpret_cast<in6_addr *>(const_cast<void *>(in)); // UGLY
	const uint8_t *byte = reinterpret_cast<const uint8_t *>(in);

	if(!in)
		throw(hard_exception("System::ipv6_address_type: nullptr"));

	if(IN6_IS_ADDR_LOOPBACK(s6addr))
		return(IPV6AddressType::loopback);

	if(IN6_IS_ADDR_LINKLOCAL(s6addr))
		return(IPV6AddressType::link_local);

	if(IN6_IS_ADDR_MULTICAST(s6addr))
		return(IPV6AddressType::multicast);

	if(IN6_IS_ADDR_SITELOCAL(s6addr))
		return(IPV6AddressType::site_local);

	if(IN6_IS_ADDR_V4MAPPED(s6addr))
		return(IPV6AddressType::ipv4_mapped);

	if(IN6_IS_ADDR_UNSPECIFIED(s6addr))
		return(IPV6AddressType::unspecified);

	if((byte[11] == 0xff) && (byte[12] == 0xfe))
		return(IPV6AddressType::global_slaac);

	return(IPV6AddressType::global_static);
}

std::string System::ipv6_address_type_string(const void *in /* sockaddr6_in->sin6_addr.s6_addr = char[16] */)
{
	IPV6AddressType type;
	std::string name;

	if(!in)
		throw(hard_exception("System::ipv6_address_type_string: nullptr"));

	type = this->ipv6_address_type(in);

	try
	{
		name = ipv6_address_type_strings.at(type);
	}
	catch(const std::out_of_range &)
	{
		name = "<illegal>";
	}

	return(name);
}

std::string System::ipv6_addr_to_string(const void *in /* sockaddr6_in->sin6_addr.in6_addr = uint8_t[16] */)
{
	std::string dst;
	const char *c_dst;
	std::string::iterator it;
	const std::uint8_t *byte = reinterpret_cast<const uint8_t *>(in);

	if(!in)
		throw(hard_exception("System::ipv6_addr_to_string: nullptr"));

	if((byte[0] == 0) && (byte[1] == 0) && (byte[2] == 0) && (byte[3] == 0) &&
			(byte[4] == 0) && (byte[5] == 0) && (byte[6] == 0) && (byte[7] == 0) &&
			(byte[8] == 0) && (byte[9] == 0) && (byte[10] == 0xff) && (byte[11] == 0xff))
		return(std::format("{:d}.{:d}.{:d}.{:d}", byte[12], byte[13], byte[14], byte[15]));

	dst.resize(INET6_ADDRSTRLEN);
	c_dst = inet_ntop(AF_INET6, in, dst.data(), dst.size());
	dst.resize(strlen(c_dst));

	for(auto &c : dst)
		switch(c)
		{
			case('A'): { c = 'a'; break; }
			case('B'): { c = 'b'; break; }
			case('C'): { c = 'c'; break; }
			case('D'): { c = 'd'; break; }
			case('E'): { c = 'e'; break; }
			case('F'): { c = 'f'; break; }
			default: { break; }
		}

	return(dst);
}

void System::string_to_ipv6_addr(const std::string &string, void *addr /* sockaddr6_in->sin6_addr.in6_addr = uint8_t[16] */)
{
	int rv;

	if(!addr)
		throw(hard_exception("System::string_to_ipv6_addr: invalid argument"));

	if((rv = ::inet_pton(AF_INET6, string.c_str(), addr)) == 0)
		throw(transient_exception("System::string_to_ipv6_addr: invalid address"));

	if(rv == -1)
		throw(hard_exception("System::string_to_ipv6_addr: invalid address family"));
}

std::string System::mac_addr_to_string(std::string_view mac, bool invert)
{
	std::string dst;

	if(invert)
		dst = std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
				mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
	else
		dst = std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	return(dst);
}

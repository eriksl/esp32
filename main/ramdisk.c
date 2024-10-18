#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ramdisk.h"
#include "string.h"
#include "log.h"
#include "util.h"
#include "info.h"

#include <freertos/FreeRTOS.h>
#include <esp_vfs.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>

enum
{
	fd_table_size = 8,
	file_name_size = 64,
	block_size = 4096,
	block_ptr_size = 1004,
	meta_magic_word_1 = 0x12345678UL,
	meta_magic_word_2 = 0xfedcba98UL,
};

typedef struct
{
	uint8_t data[block_size];
} data_block_t;

_Static_assert(sizeof(data_block_t) == block_size);

typedef struct file_metadata_T
{
	unsigned int magic_word_1;
	struct file_metadata_T *next;
	unsigned int length;
	char filename[file_name_size];
	data_block_t *datablocks[block_ptr_size];
	unsigned int magic_word_2;
} file_metadata_t;

_Static_assert(sizeof(file_metadata_t) == block_size);

typedef struct file_descriptor_T
{
	struct
	{
		unsigned int open:1;
	};
	int fcntl_flags;
	file_metadata_t *metadata;
	unsigned int offset;
} file_descriptor_t;

typedef struct
{
	DIR dir;
	struct dirent dirent;
	long offset;
	char *path;
} vfs_ramdisk_dir_t;

static bool inited = false;
static file_descriptor_t *fd_table = (file_descriptor_t *)0;
static file_metadata_t *root = (file_metadata_t *)0;
static SemaphoreHandle_t data_mutex;

static void data_mutex_take(void)
{
	xSemaphoreTake(data_mutex, portMAX_DELAY);
}

static void data_mutex_give(void)
{
	xSemaphoreGive(data_mutex);
}

static bool meta_valid(const file_metadata_t *meta)
{
	if((meta->magic_word_1 == meta_magic_word_1) && (meta->magic_word_2 == meta_magic_word_2))
		return(true);

	return(false);
}

static bool file_in_use(const char *filename, unsigned int fcntl_flags)
{
	unsigned int fd;
	const file_descriptor_t *fdp;

	assert(inited);

	for(fd = 0; fd < fd_table_size; fd++)
	{
		fdp = &fd_table[fd];

		if(!fdp->open)
			continue;

		if(!strcmp(fdp->metadata->filename, filename))
		{
			if((fcntl_flags & O_WRONLY) || (fcntl_flags & O_RDWR))
				return(true);

			if((fdp->fcntl_flags & O_WRONLY) || (fdp->fcntl_flags & O_RDWR))
				return(true);
		}
	}

	return(false);
}

static file_metadata_t *get_dir_entry(const char *filename, int *index)
{
	unsigned int ix;
	file_metadata_t *entry;

	assert(inited);

	for(entry = root, ix = 0; entry; entry = entry->next, ix++)
	{
		if(!meta_valid(entry))
			log_format("ramdisk: metadata corrupt at entry %u", ix);

		if(!strcmp(filename, entry->filename))
			break;
	}

	if(!entry)
	{
		if(index)
			*index = -1;

		return((file_metadata_t *)0);
	}

	if(index)
		*index = ix;

	return(entry);
}

static void truncate_file(file_metadata_t *meta)
{
	unsigned int datablock, datablocks;

	assert(inited);

	datablocks = sizeof(meta->datablocks) / sizeof(*meta->datablocks);

	for(datablock = 0; datablock < datablocks; datablock++)
	{
		if(meta->datablocks[datablock])
		{
			free(meta->datablocks[datablock]);
			meta->datablocks[datablock] = (data_block_t *)0;
		}
	}

	meta->length = 0;
}

static int ramdisk_open(const char *path, int fcntl_flags, int file_access_mode)
{
	file_metadata_t *entry, *parent;
	file_descriptor_t *fdp;
	const char *relpath;
	unsigned int fd;
	unsigned int datablock, datablocks;

	assert(inited);

	if(*path == '/')
		relpath = &path[1];
	else
		relpath = path;

	data_mutex_take();

	if(file_in_use(relpath, fcntl_flags))
	{
		errno = EBUSY;
		data_mutex_give();
		return(-1);
	}

	if(!(entry = get_dir_entry(relpath, (int *)0)))
	{
		if(fcntl_flags & O_CREAT)
		{
			entry = (file_metadata_t *)util_memory_alloc_spiram(sizeof(file_metadata_t));

			assert(entry);

			entry->next = (file_metadata_t *)0;
			entry->length = 0;
			entry->magic_word_1 = meta_magic_word_1;
			entry->magic_word_2 = meta_magic_word_2;
			strlcpy(entry->filename, relpath, sizeof(entry->filename));

			datablocks = sizeof(entry->datablocks) / sizeof(*entry->datablocks);

			for(datablock = 0; datablock < datablocks; datablock++)
				entry->datablocks[datablock] = (data_block_t *)0;

			for(parent = root; parent && parent->next; parent = parent->next)
				(void)0;

			if(parent)
			{
				assert(!parent->next);
				parent->next = entry;
			}
			else
				root = entry;
		}
		else
		{
			errno = ENOENT;
			data_mutex_give();
			return(-1);
		}
	}

	for(fd = 0; fd < fd_table_size; fd++)
		if(!fd_table[fd].open)
			break;

	if(fd >= fd_table_size)
	{
		errno = ENOMEM;
		data_mutex_give();
		return(-1);
	}

	if(fcntl_flags & O_TRUNC)
		truncate_file(entry);

	fdp = &fd_table[fd];

	fdp->metadata = entry;
	fdp->fcntl_flags = fcntl_flags;
	fdp->offset = (fcntl_flags & O_APPEND) ? entry->length : 0;
	fdp->open = 1;

	data_mutex_give();

	return((int)fd);
}

static int ramdisk_close(int fd)
{
	assert(inited);

	if((fd < 0) || (fd >= fd_table_size))
	{
		errno = EBADF;
		return(-1);
	}

	if(!fd_table[fd].open)
	{
		errno = EBADF;
		return(-1);
	}

	fd_table[fd].open = 0;

	return(0);
}

static ssize_t ramdisk_read(int fd, void *data_in, size_t size)
{
	file_descriptor_t *fdp;
	file_metadata_t *metadata;
	unsigned int block, offset_in_block, available_in_block, chunk;
	ssize_t rv = 0;
	uint8_t *data = (uint8_t *)data_in;

	assert(inited);

	if(!data)
	{
		errno = EINVAL;
		return(-1);
	}

	if((fd < 0) || (fd >= fd_table_size))
	{
		errno = EBADF;
		return(-1);
	}

	data_mutex_take();

	if(!fd_table[fd].open)
	{
		errno = EBADF;
		data_mutex_give();
		return(-1);
	}

	metadata = fd_table[fd].metadata;
	fdp = &fd_table[fd];

	if(fdp->offset > metadata->length)
	{
		log_format("ramdisk: read: offset [%u] > file length [%u]", fdp->offset, metadata->length);
		fdp->offset = metadata->length;
	}

	if((fdp->offset + size) > metadata->length)
		size = metadata->length - fdp->offset;

	while(size > 0)
	{
		block = fdp->offset / block_size;
		offset_in_block = fdp->offset - (block_size * block);
		available_in_block = block_size - offset_in_block;

		if(!metadata->datablocks[block])
		{
			log_format("ramdisk: read: block #%u not allocated", block);
			data_mutex_give();
			errno = EINVAL;
			return(-1);
		}

		chunk = size;

		if(chunk > available_in_block)
			chunk = available_in_block;

		memcpy(data, &metadata->datablocks[block]->data[offset_in_block], chunk);

		rv += chunk;
		size -= chunk;
		data += chunk;
		fdp->offset += chunk;
	}

	data_mutex_give();
	return(rv);
}

static ssize_t ramdisk_write(int fd, const void *data_in, size_t size)
{
	file_descriptor_t *fdp;
	file_metadata_t *metadata;
	unsigned int block, blocks, offset_in_block, available_in_block, chunk;
	ssize_t rv = size;
	const uint8_t *data = (const uint8_t *)data_in;

	assert(inited);

	if(!data)
	{
		errno = EINVAL;
		return(-1);
	}

	if((fd < 0) || (fd >= fd_table_size))
	{
		errno = EBADF;
		return(-1);
	}

	data_mutex_take();

	if(!fd_table[fd].open)
	{
		errno = EBADF;
		data_mutex_give();
		return(-1);
	}

	fdp = &fd_table[fd];

	if(!(fdp->fcntl_flags & O_WRONLY) && !(fdp->fcntl_flags & O_RDWR))
	{
		data_mutex_give();
		errno = EPERM;
		return(-1);
	}

	metadata = fdp->metadata;

	if(fdp->offset > metadata->length)
	{
		log_format("ramdisk: write: offset [%u] > file length [%u]", fdp->offset, metadata->length);
		fdp->offset = metadata->length;
	}

	while(size > 0)
	{
		if((heap_caps_get_free_size(MALLOC_CAP_SPIRAM) * 2) < initial_free_spiram)
		{
			data_mutex_give();
			errno = ENOSPC;
			return(-1);
		}

		blocks = sizeof(metadata->datablocks) / sizeof(*metadata->datablocks);
		block = fdp->offset / block_size;

		if(block >= blocks)
		{
			data_mutex_give();
			errno = EFBIG;
			return(-1);
		}

		offset_in_block = fdp->offset - (block_size * block);
		available_in_block = block_size - offset_in_block;

		if(!metadata->datablocks[block])
			metadata->datablocks[block] = util_memory_alloc_spiram(sizeof(data_block_t));

		chunk = size;

		if(chunk > available_in_block)
			chunk = available_in_block;

		memcpy(&metadata->datablocks[block]->data[offset_in_block], data, chunk);

		size -= chunk;
		data += chunk;
		fdp->offset += chunk;

		if(fdp->offset > metadata->length)
			metadata->length = fdp->offset;
	}

	data_mutex_give();

	return(rv);
}

static int ramdisk_unlink(const char *filename)
{
	file_metadata_t *meta, *parent;
	const char *relpath;

	assert(inited);

	if(*filename == '/')
		relpath = &filename[1];
	else
		relpath = filename;

	data_mutex_take();

	if(file_in_use(relpath, O_RDWR))
	{
		data_mutex_give();
		errno = EBUSY;
		return(-1);
	}

	if(!(meta = get_dir_entry(relpath, (int *)0)))
	{
		data_mutex_give();
		errno = ENOENT;
		return(-1);
	}

	truncate_file(meta);

	if(meta == root)
		root = meta->next;
	else
	{
		for(parent = root; parent; parent = parent->next)
			if(parent->next == meta)
				break;

		if(!parent)
		{
			data_mutex_give();
			log("ramdisk: unlink: no parent");
			errno = EIO;
			return(-1);
		}
		parent->next = meta->next;
	}

	free(meta);

	data_mutex_give();

	return(0);
}

static int ramdisk_stat(const char *path, struct stat *st)
{
	int ix;
	file_metadata_t *entry;
	const char *relpath;

	assert(inited);

	if(*path == '/')
		relpath = &path[1];
	else
		relpath = path;

	data_mutex_take();

	if(!(entry = get_dir_entry(relpath, &ix)))
	{
		data_mutex_give();
		errno = ENOENT;
		return(-1);
	}

	memset(st, 0, sizeof(st));

	st->st_ino = ix;
	st->st_size = entry->length;
	st->st_blksize = block_size;
	st->st_blocks = (entry->length / block_size) + 1;

	data_mutex_give();

	return(0);
}

static DIR *ramdisk_opendir(const char *name)
{
	vfs_ramdisk_dir_t *dir;

	assert(inited);

	dir = util_memory_alloc_spiram(sizeof(*dir));

	dir->offset = 0;
	dir->path = strdup(name);

	return((DIR *)dir);
}

static struct dirent *ramdisk_readdir(DIR *pdir)
{
	vfs_ramdisk_dir_t *dir = (vfs_ramdisk_dir_t *)(void *)pdir;
	struct dirent *dirent = &dir->dirent;
	file_metadata_t *entry;
	unsigned int ix;

	assert(inited);

	data_mutex_take();

	for(entry = root, ix = 0; entry && (ix < dir->offset); entry = entry->next, ix++)
		if(!meta_valid(entry))
			log_format("ramdisk: metadata corrupt in entry %u", ix);

	if(!entry)
	{
		data_mutex_give();
		return(struct dirent *)0;
	}

	dirent->d_type = DT_UNKNOWN;
	dirent->d_ino = ix;
	strlcpy(dirent->d_name, entry->filename, sizeof(dirent->d_name));

	dir->offset++;

	data_mutex_give();
	return(dirent);
}

static int ramdisk_closedir(DIR *pdir)
{
	vfs_ramdisk_dir_t *dir = (vfs_ramdisk_dir_t *)(void *)pdir;

	assert(inited);

	free(dir->path);
	free(dir);

	return(0);
}

static void ramdisk_format(void)
{
	file_metadata_t *entry, *next;

	for(entry = root; entry; entry = next)
	{
		if(!meta_valid(entry))
			log("ramdisk: format: metadata corrupt");

		next = entry->next;
		free(entry);
	}
}

static off_t ramdisk_lseek(int fd, off_t offset_requested, int mode)
{
	file_descriptor_t *fdp;
	file_metadata_t *metadata;
	off_t start_offset, offset;
	unsigned int block;

	assert(inited);

	if((fd < 0) || (fd >= fd_table_size))
	{
		errno = EBADF;
		return(-1);
	}

	data_mutex_take();

	if(!fd_table[fd].open)
	{
		errno = EBADF;
		data_mutex_give();
		return(-1);
	}

	fdp = &fd_table[fd];
	metadata = fdp->metadata;

	switch(mode)
	{
		case(SEEK_CUR):
		{
			start_offset = fdp->offset;
			break;
		}

		case(SEEK_SET):
		{
			start_offset = 0UL;
			break;
		}

		case(SEEK_END):
		{
			start_offset = metadata->length;
			break;
		}

		default:
		{
			errno = EINVAL;
			return(-1);
		}
	}

	offset = start_offset + offset_requested;

	if((offset < 0) || (offset > metadata->length))
	{
		data_mutex_give();
		errno = EINVAL;
		return(-1);
	}

	if(offset > 0)
	{
		block = (offset - 1) / block_size;

		if(!metadata->datablocks[block])
		{
			log_format("ramdisk: lseek: requesting block %u of file %s which is not allocated", block, metadata->filename);
			data_mutex_give();
			errno = EINVAL;
			return(-1);
		}
	}

	fdp->offset = offset;

	data_mutex_give();

	return(fdp->offset);
}

void ramdisk_init(void)
{
	static esp_vfs_t esp_vfs_ramdisk;
	unsigned int fd;

	assert(!inited);
	assert(root == (file_metadata_t *)0);

	data_mutex = xSemaphoreCreateMutex();

	assert(data_mutex);

	data_mutex_take();

	ramdisk_format();

	fd_table = (file_descriptor_t *)util_memory_alloc_spiram(sizeof(file_descriptor_t) * fd_table_size);

	for(fd = 0; fd < fd_table_size; fd++)
		fd_table[fd].open = 0;

	memset(&esp_vfs_ramdisk, 0, sizeof(esp_vfs_ramdisk));

	esp_vfs_ramdisk.flags = ESP_VFS_FLAG_DEFAULT;
	esp_vfs_ramdisk.open = ramdisk_open;
	esp_vfs_ramdisk.close = ramdisk_close;
	esp_vfs_ramdisk.read = ramdisk_read;
	esp_vfs_ramdisk.write = ramdisk_write;
	esp_vfs_ramdisk.stat = ramdisk_stat;
	esp_vfs_ramdisk.unlink = ramdisk_unlink;
	esp_vfs_ramdisk.opendir = ramdisk_opendir;
	esp_vfs_ramdisk.readdir = ramdisk_readdir;
	esp_vfs_ramdisk.closedir = ramdisk_closedir;
	esp_vfs_ramdisk.lseek = ramdisk_lseek;

	util_abort_on_esp_err("esp_vfs_register", esp_vfs_register("/ramdisk", &esp_vfs_ramdisk, (void *)0));

	inited = true;

	data_mutex_give();
}

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
	block_ptr_size = 1003,
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
	unsigned int inode;
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

typedef struct
{
	char mountpoint[16];
	unsigned int instance;
	file_descriptor_t *fd_table;
	file_metadata_t *root;
	SemaphoreHandle_t data_mutex;
} vfs_ramdisk_context_t;

static bool inited = false;

static void data_mutex_take(vfs_ramdisk_context_t *context)
{
	xSemaphoreTake(context->data_mutex, portMAX_DELAY);
}

static void data_mutex_give(vfs_ramdisk_context_t *context)
{
	xSemaphoreGive(context->data_mutex);
}

static bool meta_valid(const file_metadata_t *meta)
{
	if((meta->magic_word_1 == meta_magic_word_1) && (meta->magic_word_2 == meta_magic_word_2))
		return(true);

	return(false);
}

static bool file_in_use(vfs_ramdisk_context_t *context, const char *filename, unsigned int fcntl_flags)
{
	unsigned int fd;
	const file_descriptor_t *fdp;

	assert(inited);

	for(fd = 0; fd < fd_table_size; fd++)
	{
		fdp = &context->fd_table[fd];

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

static file_metadata_t *metadata_from_filename(vfs_ramdisk_context_t *context, const char *filename)
{
	unsigned int ix;
	file_metadata_t *meta;

	assert(inited);

	for(meta = context->root, ix = 0; meta; meta = meta->next, ix++)
	{
		if(!meta_valid(meta))
			log_format("ramdisk: metadata corrupt at entry %u", ix);

		if(!strcmp(filename, meta->filename))
			break;
	}

	if(!meta)
	{
		errno = ENOENT;
		return((file_metadata_t *)0);
	}

	return(meta);
}

static file_metadata_t *metadata_from_fd(vfs_ramdisk_context_t *context, int fd)
{
	file_metadata_t *meta;

	if((fd < 0) || (fd >= fd_table_size))
	{
		errno = EBADF;
		return(file_metadata_t *)0;
	}

	if(!context->fd_table[fd].open)
	{
		errno = EBADF;
		return(file_metadata_t *)0;
	}

	meta = context->fd_table[fd].metadata;

	if(!meta_valid(meta))
		log("ramdisk: metadata corrupt in metadata_from_fd");

	return(meta);
}

static void file_truncate(file_metadata_t *meta, unsigned int length)
{
	if(meta->length > length)
		meta->length = length;
}

static void file_stat(const vfs_ramdisk_context_t *context, const file_metadata_t *meta, struct stat *st)
{
	static const unsigned int blocks = sizeof(meta->datablocks) / sizeof(*meta->datablocks);
	unsigned int block;

	assert(inited);
	assert(blocks == block_ptr_size);

	memset(st, 0, sizeof(st));

	st->st_dev = context->instance;
	st->st_ino = meta->inode;
	st->st_size = meta->length;
	st->st_blksize = block_size;

	for(block = 0, st->st_blocks = 0; block < blocks; block++)
		if(meta->datablocks[block])
			st->st_blocks += block_size / 512;
}

static int ramdisk_open(void *ctx, const char *path, int fcntl_flags, int file_access_mode)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
	file_metadata_t *meta, *parent;
	file_descriptor_t *fdp;
	const char *relpath;
	unsigned int fd;
	unsigned int datablock, datablocks;

	assert(inited);

	if(*path == '/')
		relpath = &path[1];
	else
		relpath = path;

	data_mutex_take(context);

	if(file_in_use(context, relpath, fcntl_flags))
	{
		errno = EBUSY;
		data_mutex_give(context);
		return(-1);
	}

	if(!(meta = metadata_from_filename(context, relpath)))
	{
		if(fcntl_flags & O_CREAT)
		{
			meta = (file_metadata_t *)util_memory_alloc_spiram(sizeof(file_metadata_t));

			assert(meta);

			meta->next = (file_metadata_t *)0;
			meta->length = 0;
			meta->magic_word_1 = meta_magic_word_1;
			meta->magic_word_2 = meta_magic_word_2;
			strlcpy(meta->filename, relpath, sizeof(meta->filename));

			datablocks = sizeof(meta->datablocks) / sizeof(*meta->datablocks);

			for(datablock = 0; datablock < datablocks; datablock++)
				meta->datablocks[datablock] = (data_block_t *)0;

			for(parent = context->root; parent && parent->next; parent = parent->next)
				(void)0;

			if(parent)
			{
				assert(!parent->next);
				parent->next = meta;
				meta->inode = parent->inode + 1;
			}
			else
			{
				meta->inode = 1;
				context->root = meta;
			}
		}
		else
		{
			errno = ENOENT;
			data_mutex_give(context);
			return(-1);
		}
	}

	for(fd = 0; fd < fd_table_size; fd++)
		if(!context->fd_table[fd].open)
			break;

	if(fd >= fd_table_size)
	{
		errno = ENOMEM;
		data_mutex_give(context);
		return(-1);
	}

	if(fcntl_flags & O_TRUNC)
		file_truncate(meta, 0);

	fdp = &context->fd_table[fd];

	fdp->metadata = meta;
	fdp->fcntl_flags = fcntl_flags;
	fdp->offset = (fcntl_flags & O_APPEND) ? meta->length : 0;
	fdp->open = 1;

	data_mutex_give(context);

	return((int)fd);
}

static int ramdisk_close(void *ctx, int fd)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;

	assert(inited);

	if((fd < 0) || (fd >= fd_table_size))
	{
		errno = EBADF;
		return(-1);
	}

	if(!context->fd_table[fd].open)
	{
		errno = EBADF;
		return(-1);
	}

	context->fd_table[fd].open = 0;

	return(0);
}

static ssize_t ramdisk_read(void *ctx, int fd, void *data_in, size_t size)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
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

	data_mutex_take(context);

	if(!context->fd_table[fd].open)
	{
		errno = EBADF;
		data_mutex_give(context);
		return(-1);
	}

	metadata = metadata_from_fd(context, fd);
	assert(metadata);

	fdp = &context->fd_table[fd];

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
			data_mutex_give(context);
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

	data_mutex_give(context);
	return(rv);
}

static ssize_t ramdisk_write(void *ctx, int fd, const void *data_in, size_t size)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
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

	data_mutex_take(context);

	if(!context->fd_table[fd].open)
	{
		errno = EBADF;
		data_mutex_give(context);
		return(-1);
	}

	fdp = &context->fd_table[fd];

	if(!(fdp->fcntl_flags & O_WRONLY) && !(fdp->fcntl_flags & O_RDWR))
	{
		data_mutex_give(context);
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
			data_mutex_give(context);
			errno = ENOSPC;
			return(-1);
		}

		blocks = sizeof(metadata->datablocks) / sizeof(*metadata->datablocks);
		block = fdp->offset / block_size;

		if(block >= blocks)
		{
			data_mutex_give(context);
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

	data_mutex_give(context);

	return(rv);
}

static int ramdisk_unlink(void *ctx, const char *filename)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
	file_metadata_t *meta, *parent;
	const char *relpath;
	struct stat statb;

	assert(inited);

	if(*filename == '/')
		relpath = &filename[1];
	else
		relpath = filename;

	data_mutex_take(context);

	if(file_in_use(context, relpath, O_WRONLY))
	{
		data_mutex_give(context);
		log_format("ramdisk: unlink: file %s in use", relpath);
		errno = EBUSY;
		return(-1);
	}

	if(!(meta = metadata_from_filename(context, relpath)))
	{
		data_mutex_give(context);
		log_format("ramdisk: debug: unlink: file %s does not exist", relpath);
		return(-1);
	}

	file_truncate(meta, 0);

	if(meta == context->root)
		context->root = meta->next;
	else
	{
		for(parent = context->root; parent; parent = parent->next)
			if(parent->next == meta)
				break;

		if(!parent)
		{
			data_mutex_give(context);
			log_format("ramdisk: unlink: file %s: no parent", relpath);
			errno = EIO;
			return(-1);
		}
		parent->next = meta->next;
	}

	meta->magic_word_1 = meta_magic_word_1 ^ 0xffffffff;
	meta->magic_word_2 = meta_magic_word_2 ^ 0xffffffff;

	free(meta);

	data_mutex_give(context);

	if(!stat(relpath, &statb))
		log_format("ramdisk: unlink: debug: file %s not removed", relpath);

	return(0);
}

static DIR *ramdisk_opendir(void *ctx, const char *name)
{
	vfs_ramdisk_dir_t *dir;

	assert(inited);

	dir = util_memory_alloc_spiram(sizeof(*dir));

	dir->offset = 0;
	dir->path = strdup(name);

	return((DIR *)dir);
}

static struct dirent *ramdisk_readdir(void *ctx, DIR *pdir)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
	vfs_ramdisk_dir_t *dir = (vfs_ramdisk_dir_t *)(void *)pdir;
	struct dirent *dirent = &dir->dirent;
	file_metadata_t *meta;
	unsigned int ix;

	assert(inited);

	data_mutex_take(context);

	for(meta = context->root, ix = 0; meta && (ix < dir->offset); meta = meta->next, ix++)
		if(!meta_valid(meta))
			log_format("ramdisk: metadata corrupt in entry %u", ix);

	if(!meta)
	{
		data_mutex_give(context);
		return(struct dirent *)0;
	}

	dirent->d_type = DT_UNKNOWN;
	dirent->d_ino = ix;
	strlcpy(dirent->d_name, meta->filename, sizeof(dirent->d_name));

	dir->offset++;

	data_mutex_give(context);
	return(dirent);
}

static int ramdisk_closedir(void *ctx, DIR *pdir)
{
	vfs_ramdisk_dir_t *dir = (vfs_ramdisk_dir_t *)(void *)pdir;

	assert(inited);

	free(dir->path);
	free(dir);

	return(0);
}

static off_t ramdisk_lseek(void *ctx, int fd, off_t offset_requested, int mode)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
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

	data_mutex_take(context);

	if(!context->fd_table[fd].open)
	{
		errno = EBADF;
		data_mutex_give(context);
		return(-1);
	}

	fdp = &context->fd_table[fd];
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
		data_mutex_give(context);
		errno = EINVAL;
		return(-1);
	}

	if(offset > 0)
	{
		block = (offset - 1) / block_size;

		if(!metadata->datablocks[block])
		{
			log_format("ramdisk: lseek: requesting block %u of file %s which is not allocated", block, metadata->filename);
			data_mutex_give(context);
			errno = EINVAL;
			return(-1);
		}
	}

	fdp->offset = offset;

	data_mutex_give(context);

	return(fdp->offset);
}

static int ramdisk_truncate(void *ctx, const char *path, off_t length)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
	const char *relpath;
	file_metadata_t *meta;

	if(*path == '/')
		relpath = &path[1];
	else
		relpath = path;

	data_mutex_take(context);

	if(!(meta = metadata_from_filename(context, relpath)))
	{
		data_mutex_give(context);
		return(-1);
	}

	if(file_in_use(context, relpath, O_WRONLY))
	{
		data_mutex_give(context);
		errno = EBUSY;
		return(-1);
	}

	file_truncate(meta, length);

	data_mutex_give(context);

	errno = 0;
	return(0);
}

static int ramdisk_ftruncate(void *ctx, int fd, off_t length)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
	file_metadata_t *meta;

	data_mutex_take(context);

	if(!(meta = metadata_from_fd(context, fd)))
	{
		data_mutex_give(context);
		return(-1);
	}

	file_truncate(meta, length);

	data_mutex_give(context);

	errno = 0;
	return(0);
}

static int ramdisk_stat(void *ctx, const char *path, struct stat *st)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
	file_metadata_t *meta;
	const char *relpath;

	assert(inited);

	if(*path == '/')
		relpath = &path[1];
	else
		relpath = path;

	data_mutex_take(context);

	if(!(meta = metadata_from_filename(context, relpath)))
	{
		data_mutex_give(context);
		return(-1);
	}

	file_stat(context, meta, st);

	data_mutex_give(context);

	return(0);
}

static int ramdisk_fstat(void *ctx, int fd, struct stat *st)
{
	vfs_ramdisk_context_t *context = (vfs_ramdisk_context_t *)ctx;
	file_metadata_t *meta;

	assert(inited);

	data_mutex_take(context);

	if(!(meta = metadata_from_fd(context, fd)))
	{
		data_mutex_give(context);
		return(-1);
	}

	file_stat(context, meta, st);

	data_mutex_give(context);

	return(0);
}

void ramdisk_init(void)
{
	vfs_ramdisk_context_t *context;
	esp_vfs_t *esp_vfs_ramdisk;
	file_metadata_t *meta, *next;
	unsigned int fd;

	assert(!inited);

	context = (vfs_ramdisk_context_t *)util_memory_alloc_spiram(sizeof(*context));
	assert(context);
	context->instance = 0;
	context->data_mutex = xSemaphoreCreateMutex();
	assert(context->data_mutex);
	data_mutex_take(context);
	strlcpy(context->mountpoint, "/ramdisk", sizeof(context->mountpoint));
	context->root = (file_metadata_t *)0;

	context->fd_table = (file_descriptor_t *)util_memory_alloc_spiram(sizeof(file_descriptor_t) * fd_table_size);
	assert(context->fd_table);
	for(fd = 0; fd < fd_table_size; fd++)
		context->fd_table[fd].open = 0;

	for(meta = context->root; meta; meta = next)
	{
		if(!meta_valid(meta))
			log("ramdisk: release space: metadata corrupt");

		next = meta->next;
		free(meta);
	}

	esp_vfs_ramdisk = (esp_vfs_t *)util_memory_alloc_spiram(sizeof(*esp_vfs_ramdisk));
	assert(esp_vfs_ramdisk);
	memset(esp_vfs_ramdisk, 0, sizeof(*esp_vfs_ramdisk));

	esp_vfs_ramdisk->flags = ESP_VFS_FLAG_CONTEXT_PTR;
	esp_vfs_ramdisk->open_p = ramdisk_open;
	esp_vfs_ramdisk->close_p = ramdisk_close;
	esp_vfs_ramdisk->read_p = ramdisk_read;
	esp_vfs_ramdisk->write_p = ramdisk_write;
	esp_vfs_ramdisk->unlink_p = ramdisk_unlink;
	esp_vfs_ramdisk->opendir_p = ramdisk_opendir;
	esp_vfs_ramdisk->readdir_p = ramdisk_readdir;
	esp_vfs_ramdisk->closedir_p = ramdisk_closedir;
	esp_vfs_ramdisk->lseek_p = ramdisk_lseek;
	esp_vfs_ramdisk->stat_p = ramdisk_stat;
	esp_vfs_ramdisk->fstat_p = ramdisk_fstat;
	esp_vfs_ramdisk->truncate_p = ramdisk_truncate;
	esp_vfs_ramdisk->ftruncate_p = ramdisk_ftruncate;

	util_abort_on_esp_err("esp_vfs_register", esp_vfs_register(context->mountpoint, esp_vfs_ramdisk, context));

	inited = true;

	data_mutex_give(context);
}

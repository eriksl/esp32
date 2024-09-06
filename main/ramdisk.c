#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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
	ramdisk_block_size = 4096,
	meta_magic_word = 0x12345678UL,
};

typedef struct ramdisk_data_block_T
{
	uint8_t data[ramdisk_block_size];
} ramdisk_data_block_t;

static_assert(sizeof(ramdisk_data_block_t) == ramdisk_block_size);

typedef struct ramdisk_file_metadata_T
{
	struct ramdisk_file_metadata_T *next;
	unsigned int length;
	char filename[64];
	unsigned int magic_word;
	ramdisk_data_block_t *datablocks[1005];
} ramdisk_file_metadata_t;

static_assert(sizeof(ramdisk_file_metadata_t) == ramdisk_block_size);

typedef struct ramdisk_file_descriptor_T
{
	struct ramdisk_file_descriptor_T *next;
	int fd;
	int open_flags;
	ramdisk_file_metadata_t *metadata;
	unsigned int offset;
} ramdisk_file_descriptor_t;

typedef struct
{
	DIR dir;			/*!< VFS DIR struct */
	//ramdisk_dir_t d;	/*!< DIR struct */
	struct dirent dirent;	/*!< Last open dirent */
	long offset;		/*!< Offset of the current dirent */
	char *path;			/*!< Requested directory name */
} vfs_ramdisk_dir_t;

static bool inited = false;
static ramdisk_file_metadata_t *root = (ramdisk_file_metadata_t *)0;
static ramdisk_file_descriptor_t *fds = (ramdisk_file_descriptor_t *)0;
static SemaphoreHandle_t mutex;

static void mutex_take(void)
{
	assert(xSemaphoreTake(mutex, portMAX_DELAY));
}

static void mutex_give(void)
{
	assert(xSemaphoreGive(mutex));
}

static ramdisk_file_descriptor_t *new_fd(ramdisk_file_metadata_t *meta, int open_flags)
{
	int highest, fd;
	ramdisk_file_descriptor_t *entry, *last;

	for(entry = fds, last = (ramdisk_file_descriptor_t *)0, highest = -1; entry; entry = entry->next)
	{
		if(entry->next)
			last = entry->next;

		if(entry->fd > highest)
			highest = entry->fd;
	}

	for(fd = 0; fd < highest; fd++)
	{
		for(entry = fds; entry; entry = entry->next)
			if(entry->fd == fd)
				break;

		if(!entry)
			break;
	}

	entry = util_memory_alloc_spiram(sizeof(*entry));
	entry->next = (ramdisk_file_descriptor_t *)0;
	entry->fd = fd;
	entry->open_flags = open_flags;
	entry->metadata = meta;
	entry->offset = 0;

	if(last)
		last->next = entry;
	else
		fds = entry;

	//log_format("new fd: %d", entry->fd);

	return(entry);
}

static ramdisk_file_descriptor_t *get_fd(int fd)
{
	ramdisk_file_descriptor_t *entry;

	for(entry = fds; entry; entry = entry->next)
		if(entry->fd == fd)
			break;

	return(entry);
}

static int delete_fd(int fd)
{
	ramdisk_file_descriptor_t *entry, *previous;

	for(entry = fds, previous = (ramdisk_file_descriptor_t *)0; entry; previous = entry, entry = entry->next)
		if(entry->fd == fd)
			break;

	if(!entry)
		return(-1);

	if(previous)
		previous = entry->next;
	else
		fds = entry->next;

	free(entry);

	return(0);
}

static bool file_in_use(const char *filename, int flags)
{
	ramdisk_file_descriptor_t *entry;

	for(entry = fds; entry; entry = entry->next)
	{
		if(!strcmp(entry->metadata->filename, filename))
		{
			if((flags & O_WRONLY) || (flags & O_RDWR))
				return(true);

			if((entry->open_flags & O_WRONLY) || (entry->open_flags & O_RDWR))
				return(true);
		}
	}

	return(false);
}

static ramdisk_file_metadata_t *get_dir_entry(const char *filename, int *index)
{
	int ix;
	ramdisk_file_metadata_t *entry;

	for(entry = root, ix = 0; entry; entry = entry->next, ix++)
	{
		if(entry->magic_word != meta_magic_word)
			log_format("ramdisk: invalid magic word at entry %u", ix);

		if(!strcmp(filename, entry->filename))
			break;
	}

	if(!entry)
	{
		if(index)
			*index = -1;

		return((ramdisk_file_metadata_t *)0);
	}

	if(index)
		*index = ix;

	return(entry);
}

static ramdisk_file_metadata_t *create_file(const char *filename)
{
	ramdisk_file_metadata_t *parent, *new;
	unsigned int datablock, datablocks;

	new = util_memory_alloc_spiram(sizeof(*new));

	new->next = (ramdisk_file_metadata_t *)0;
	new->length = 0;
	new->magic_word = meta_magic_word;
	strlcpy(new->filename, filename, sizeof(new->filename));

	datablocks = sizeof(new->datablocks) / sizeof(*new->datablocks);

	for(datablock = 0; datablock < datablocks; datablock++)
		new->datablocks[datablock] = (ramdisk_data_block_t *)0;

	for(parent = root; parent && parent->next; parent = parent->next);

	if(parent)
		parent->next = new;
	else
		root = new;

	return(new);
}

static void truncate_file(ramdisk_file_metadata_t *meta)
{
	unsigned int datablock, datablocks;

	datablocks = sizeof(meta->datablocks) / sizeof(*meta->datablocks);

	for(datablock = 0; datablock < datablocks; datablock++)
	{
		if(meta->datablocks[datablock])
		{
			free(meta->datablocks[datablock]);
			meta->datablocks[datablock] = (ramdisk_data_block_t *)0;
		}
	}

	meta->length = 0;
}

static int ramdisk_open(const char *path, int flags, int file_access_mode)
{
	ramdisk_file_metadata_t *entry;
	ramdisk_file_descriptor_t *fdp;
	const char *relpath;

	if(*path == '/')
		relpath = &path[1];
	else
		relpath = path;

	mutex_take();

	if(file_in_use(relpath, flags))
	{
		errno = EBUSY;
		mutex_give();
		return(-1);
	}

	if(!(entry = get_dir_entry(relpath, (int *)0)))
	{
		if(flags & O_CREAT)
		{
			if(!(entry = create_file(relpath)))
				return(-1);
		}
		else
		{
			errno = ENOENT;
			mutex_give();
			return(-1);
		}
	}

	if(flags & O_TRUNC)
		truncate_file(entry);

	fdp = new_fd(entry, flags);

	if(flags & O_APPEND)
		fdp->offset = entry->length;
	else
		fdp->offset = 0;

	mutex_give();

	return(fdp->fd);
}

static int ramdisk_close(int fd)
{
	mutex_take();

	if(delete_fd(fd))
	{
		mutex_give();
		errno = EBADF;
		return(-1);
	}

	mutex_give();

	return(0);
}

ssize_t ramdisk_read(int fd, void *data, size_t size)
{
	ramdisk_file_descriptor_t *fdp;
	ramdisk_file_metadata_t *metadata;
	unsigned int block, offset_in_block, available_in_block, chunk;
	ssize_t rv = 0;

	mutex_take();

	if(!(fdp = get_fd(fd)))
	{
		mutex_give();
		errno = EBADF;
		return(-1);
	}

	if(!data)
	{
		mutex_give();
		errno = EINVAL;
		return(-1);
	}

	metadata = fdp->metadata;

	if(fdp->offset > metadata->length)
	{
		log_format("ramdisk: read: offset [%u] > file length [%u]", fdp->offset, metadata->length);
		fdp->offset = metadata->length;
	}

	if((fdp->offset + size) > metadata->length)
		size = metadata->length - fdp->offset;

	while(size > 0)
	{
		block = fdp->offset / ramdisk_block_size;
		offset_in_block = fdp->offset - (ramdisk_block_size * block);
		available_in_block = ramdisk_block_size - offset_in_block;

		//log_format("ramdisk: read: block: %u, offset in block: %u, avail: %u", block, offset_in_block, available_in_block);

		if(!metadata->datablocks[block])
		{
			log_format("ramdisk: read: block #%u not allocated", block);
			mutex_give();
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

		//log_format("ramdisk: file length: %u, fd offset: %u, size: %u", metadata->length, fdp->offset, size);
	}

	mutex_give();
	return(rv);
}

ssize_t ramdisk_write(int fd, const void *data, size_t size)
{
	ramdisk_file_descriptor_t *fdp;
	ramdisk_file_metadata_t *metadata;
	unsigned int block, blocks, offset_in_block, available_in_block, chunk;
	ssize_t rv = size;

	mutex_take();

	if(!(fdp = get_fd(fd)))
	{
		mutex_give();
		errno = EBADF;
		return(-1);
	}

	if(!data)
	{
		mutex_give();
		errno = EINVAL;
		return(-1);
	}

	if(!(fdp->open_flags & O_WRONLY) && !(fdp->open_flags & O_RDWR))
	{
		mutex_give();
		errno = EPERM;
		return(-1);
	}

	if((heap_caps_get_free_size(MALLOC_CAP_SPIRAM) * 2) < initial_free_spiram) // FIXME
	{
		mutex_give();
		errno = ENOSPC;
		return(-1);
	}

	metadata = fdp->metadata;

	if(fdp->offset > metadata->length)
	{
		log_format("ramdisk: write: offset [%u] > file length [%u]", fdp->offset, metadata->length);
		fdp->offset = metadata->length;
	}

	//log_format("ramdisk: write: size %u", size);
	//log_format("ramdisk: write: file length: %u, fd offset: %u", metadata->length, fdp->offset);

	while(size > 0)
	{
		blocks = sizeof(metadata->datablocks) / sizeof(*metadata->datablocks);
		block = fdp->offset / ramdisk_block_size;

		if(block >= blocks)
		{
			mutex_give();
			errno = EFBIG;
			return(-1);
		}

		offset_in_block = fdp->offset - (ramdisk_block_size * block);
		available_in_block = ramdisk_block_size - offset_in_block;

		//log_format("ramdisk: 1 block: %u, offset in block: %u, avail: %u", block, offset_in_block, available_in_block);

		if(!metadata->datablocks[block])
			metadata->datablocks[block] = util_memory_alloc_spiram(sizeof(ramdisk_data_block_t));

		chunk = size;

		if(chunk > available_in_block)
			chunk = available_in_block;

		memcpy(&metadata->datablocks[block]->data[offset_in_block], data, chunk);

		size -= chunk;
		data += chunk;
		fdp->offset += chunk;

		if(fdp->offset > metadata->length)
			metadata->length = fdp->offset;

		//log_format("ramdisk: 2 file length: %u, fd offset: %u, size: %u", metadata->length, fdp->offset, size);
	}

	//log_format("ramdisk: write: return %u", rv);

	mutex_give();

	return(rv);
}

static int ramdisk_unlink(const char *filename)
{
	ramdisk_file_metadata_t *meta, *parent;
	const char *relpath;

	if(*filename == '/')
		relpath = &filename[1];
	else
		relpath = filename;

	mutex_take();

	if(file_in_use(relpath, O_RDWR))
	{
		mutex_give();
		errno = EBUSY;
		return(-1);
	}

	if(!(meta = get_dir_entry(relpath, (int *)0)))
	{
		mutex_give();
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
			mutex_give();
			log("ramdisk: unlink: no parent");
			errno = EIO;
			return(-1);
		}
		parent->next = meta->next;
	}

	free(meta);

	mutex_give();

	return(0);
}

static int ramdisk_stat(const char *path, struct stat *st)
{
	int ix;
	ramdisk_file_metadata_t *entry;
	const char *relpath;

	if(*path == '/')
		relpath = &path[1];
	else
		relpath = path;

	mutex_take();

	if(!(entry = get_dir_entry(relpath, &ix)))
	{
		mutex_give();
		errno = ENOENT;
		return(-1);
	}

	memset(st, 0, sizeof(st));

	st->st_ino = ix;
	st->st_size = entry->length;
	st->st_blksize = ramdisk_block_size;
	st->st_blocks = (entry->length / ramdisk_block_size) + 1;

	mutex_give();

	return(0);
}

static DIR *ramdisk_opendir(const char *name)
{
	vfs_ramdisk_dir_t *dir;

	dir = util_memory_alloc_spiram(sizeof(*dir));

	dir->offset = 0;
	dir->path = strdup(name);

	return((DIR *)dir);
}

static struct dirent *ramdisk_readdir(DIR *pdir)
{
	vfs_ramdisk_dir_t *dir = (vfs_ramdisk_dir_t *)pdir;
	struct dirent *dirent = &dir->dirent;
	ramdisk_file_metadata_t *entry;
	unsigned int ix;

	mutex_take();

	for(entry = root, ix = 0; entry && (ix < dir->offset); entry = entry->next, ix++)
		if(entry->magic_word != meta_magic_word)
			log_format("ramdisk: invalid magic word in entry %u", ix);

	if(!entry)
	{
		mutex_give();
		return(struct dirent *)0;
	}

	dirent->d_type = DT_UNKNOWN;
	dirent->d_ino = ix;
	strlcpy(dirent->d_name, entry->filename, sizeof(dirent->d_name));

	dir->offset++;

	mutex_give();
	return(dirent);
}

static int ramdisk_closedir(DIR *pdir)
{
	vfs_ramdisk_dir_t *dir = (vfs_ramdisk_dir_t *)pdir;

	free(dir->path);
	free(dir);

	return(0);
}

void ramdisk_format(void)
{
	ramdisk_file_metadata_t *entry, *next;

	for(entry = root; entry; entry = next)
	{
		if(entry->magic_word != meta_magic_word)
			log("ramdisk: invalid magic word");

		next = entry->next;
		free(entry);
	}
}

off_t ramdisk_lseek(int fd, off_t offset_requested, int mode)
{
	ramdisk_file_descriptor_t *fdp;
	ramdisk_file_metadata_t *metadata;
	off_t start_offset, offset;
	unsigned int block;

	mutex_take();

	if(!(fdp = get_fd(fd)))
	{
		mutex_give();
		errno = EBADF;
		return(-1);
	}

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

	//log_format("ramdisk: seek to %u", (unsigned int)offset);

	if((offset < 0) || (offset > metadata->length))
	{
		mutex_give();
		errno = EINVAL;
		return(-1);
	}

	if(offset > 0)
	{
		block = (offset - 1) / ramdisk_block_size;

		if(!metadata->datablocks[block])
		{
			log_format("ramdisk: lseek: requesting block %d of file %s which is not allocated", block, metadata->filename);
			mutex_give();
			errno = EINVAL;
			return(-1);
		}
	}

	fdp->offset = offset;

	mutex_give();

	return(fdp->offset);
}

void ramdisk_init(void)
{
	static esp_vfs_t esp_vfs_ramdisk;

	assert(!inited);
	assert(root == (ramdisk_file_metadata_t *)0);
	assert((mutex = xSemaphoreCreateMutex()));

	mutex_take();

	ramdisk_format();

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

	mutex_give();
}

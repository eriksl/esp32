#include "ramdisk.h"

#include "log.h"
#include "exception.h"

#include <esp_vfs.h>
#include <esp_vfs_ops.h>

#include <fcntl.h>
#include <sys/ioctl.h>

#include <format>

using namespace Ramdisk;

Root *Root::singleton = nullptr;

Mutex::Mutex(SemaphoreHandle_t *mutex_in) : mutex(mutex_in)
{
	xSemaphoreTake(*this->mutex, portMAX_DELAY);
}

Mutex::~Mutex()
{
	xSemaphoreGive(*this->mutex);
	this->mutex = nullptr;
}


File::File(const std::string &filename_in, unsigned int fileno_in)
	: filename(filename_in), fileno(fileno_in)
{
	time_update(true);
}

unsigned int File::get_fileno() const
{
	return(this->fileno);
}

std::string File::get_filename() const
{
	return(this->filename);
}

unsigned int File::get_length() const
{
	return(this->contents.length());
}

unsigned int File::get_allocated() const
{
	return(this->contents.capacity());
}

struct timespec File::get_ctime() const
{
	return(this->c_time);
}

struct timespec File::get_mtime() const
{
	return(this->m_time);
}

void File::time_update(bool update_ctime)
{
	struct timespec now;

	if(clock_gettime(CLOCK_REALTIME, &now))
		return;

	m_time = now;

	if(update_ctime)
		c_time = m_time;
}

int File::read(unsigned int offset, unsigned int size, uint8_t *data) const
{
	unsigned int osize = size;
	unsigned int length = this->contents.length();

	if(offset > length)
	{
		Log::get() << std::format("ramdisk::File::read: offset out of range: s:{:d} l:{:d} o:{:d})",
				size, length, offset);

		return(-EIO);
	}

	if((offset + size) > length)
		size = length - offset;

	try
	{
		this->contents.copy(data, size, offset);
	}
	catch(const std::out_of_range &e)
	{
		Log::get() << std::format("ramdisk::File::read: out of range: s:{:d}/{:d} l:{:d}/{:d} o:{:d}",
				osize, size,
				length, this->contents.length(),
				offset);
	}

	return(size);
}

int File::write(unsigned int offset, unsigned int length, const uint8_t *data)
{
	if(offset > this->contents.length())
		this->contents.resize(offset + length);

	this->contents.replace(offset, length, data, length);

	this->time_update();

	return(length);
}

int File::truncate(unsigned int length)
{
	this->contents.resize(length);
	this->time_update();

	return(0);
}

int File::rename(const std::string &filename_in)
{
	this->filename = filename_in;

	return(0);
}

Dirent::Dirent(int next_fileno_in)
	:
		next_fileno(next_fileno_in)
{
}

void Dirent::set(int fileno_next_in, unsigned int fileno_in, const std::string &filename_in)
{
	this->next_fileno = fileno_next_in;

	memset(&this->dirent, 0, sizeof(this->dirent));
	this->dirent.d_ino = fileno_in;
	this->dirent.d_type = DT_REG;
	strlcpy(this->dirent.d_name, filename_in.c_str(), sizeof(this->dirent.d_name));
}

int Dirent::get_next_fileno() const
{
	return(this->next_fileno);
}

DIR *Dirent::get_DIR()
{
	return(&this->dir);
}

dirent *Dirent::get_dirent()
{
	return(&this->dirent);
}

Directory::Directory(const std::string &path_in)
	:	path(path_in)
{
}

File *Directory::get_file_by_fileno(unsigned int fileno)
{
	FileMap::iterator it;

	if((it = files.find(fileno)) != files.end())
		return(&it->second);

	return(nullptr);
}

const File *Directory::get_file_by_fileno_const(unsigned int fileno) const
{
	FileMap::const_iterator it;

	if((it = files.find(fileno)) != files.end())
		return(&it->second);

	return(nullptr);
}

File *Directory::get_file_by_name(const std::string &filename)
{
	FileMap::iterator it;

	for(it = this->files.begin(); it != this->files.end(); it++)
		if((this->path + it->second.get_filename()) == filename)
			return(&it->second);

	return(nullptr);
}

const File *Directory::get_file_by_name_const(const std::string &filename) const
{
	FileMap::const_iterator it;

	for(it = this->files.begin(); it != this->files.end(); it++)
		if((this->path + it->second.get_filename()) == filename)
			return(&it->second);

	return(nullptr);
}

unsigned int Directory::get_used() const
{
	FileMap::const_iterator it;
	unsigned int total;

	for(it = this->files.begin(), total = 0; it != this->files.end(); it++)
		total += it->second.get_allocated();

	return(total);
}

int Directory::opendir(const std::string &path_in) const
{
	FileMap::const_iterator it;

	if(path_in != this->path)
		return(-EXDEV);

	if((it = files.begin()) == files.end())
		return(-ENOENT);

	return(it->second.get_fileno());
}

int Directory::readdir(Dirent *dirent) const
{
	FileMap::const_iterator it, next_it;
	int fileno;

	fileno = dirent->get_next_fileno();

	if((fileno < 0) || (it = files.find(fileno)) == files.end())
	{
		dirent->set(-1);
		return(-ENOENT);
	}

	next_it = it;

	if((++next_it) == files.end())
		fileno = -1;
	else
		fileno = next_it->first;

	dirent->set(fileno, it->second.get_fileno(), it->second.get_filename());

	return(0);
}

int Directory::closedir(const Dirent *dirent) const
{
	return(0);
}

int Directory::open(const std::string &path_in, int fcntl_flags, unsigned int new_fileno)
{
	const File *fp;

	if(!(fp = this->get_file_by_name_const(path_in)))
	{
		if(fcntl_flags & O_CREAT)
		{
			if(!path_in.starts_with(this->path))
				return(-ENOENT);

			this->files.insert(std::pair(new_fileno, File(path_in.substr(this->path.length()), new_fileno)));
			fp = this->get_file_by_name_const(path_in);

			if(!fp)
				return(-EIO);
		}
		else
			return(-ENOENT);
	}

	return(fp->get_fileno());
}

int Directory::close(int fd) const
{
	return(0);
}

int Directory::read(unsigned int fileno, unsigned int offset, unsigned int size, uint8_t *data) const
{
	const File *fp;

	if(!(fp = this->get_file_by_fileno_const(fileno)))
		return(-ENOENT);

	return(fp->read(offset, size, data));
}

int Directory::write(unsigned int fileno, unsigned int offset, unsigned int length, const uint8_t *data)
{
	File *fp;

	if(!(fp = this->get_file_by_fileno(fileno)))
		return(-ENOENT);

	return(fp->write(offset, length, data));
}

int Directory::rename(const std::string &from, const std::string &to)
{
	FileMap::iterator it;
	File *fp;
	std::string to_filename;

	if(!to.starts_with(this->path))
		return(-EXDEV);

	if(!(fp = this->get_file_by_name(from)))
		return(-ENOENT);

	to_filename = to.substr(this->path.length());

	for(it = this->files.begin(); it != this->files.end(); it++)
		if(it->second.get_filename() == to_filename)
			break;

	if(it != this->files.end())
		this->files.erase(it);

	return(fp->rename(to_filename));
}

int Directory::unlink(const std::string &path_in)
{
	FileMap::iterator it;
	const File *fp;

	if(!(fp = this->get_file_by_name_const(path_in)))
		return(-ENOENT);

	if((it = this->files.find(fp->get_fileno())) == this->files.end())
		return(-EIO);

	this->files.erase(it);

	return(0);
}

int Directory::clear()
{
	this->files.clear();

	return(0);
}

FileDescriptor::FileDescriptor(unsigned int fd_in, unsigned int fileno_in, unsigned int fcntl_flags_in, unsigned int offset_in, bool fs_in)
	: fd(fd_in), fileno(fileno_in), fcntl_flags(fcntl_flags_in), offset(offset_in), fs(fs_in)
{
}

unsigned int FileDescriptor::get_fd() const
{
	return(this->fd);
}

unsigned int FileDescriptor::get_fileno() const
{
	return(this->fileno);
}

unsigned int FileDescriptor::get_fcntl_flags() const
{
	return(this->fcntl_flags);
}

unsigned int FileDescriptor::get_offset() const
{
	return(this->offset);
}

bool FileDescriptor::is_fs() const
{
	return(this->fs);
}

void FileDescriptor::set_offset(unsigned int new_offset)
{
	this->offset = new_offset;
}

Root::Root(Log &log_in, const std::string &mountpoint_in, unsigned int size_in) :
		log(log_in), mountpoint(mountpoint_in), size(size_in), root("/")
{
	static const esp_vfs_dir_ops_t vfs_dir_ops =
	{
		.stat_p = Root::static_stat,
		.link_p = nullptr,
		.unlink_p = Root::static_unlink,
		.rename_p = Root::static_rename,
		.opendir_p = Root::static_opendir,
		.readdir_p = Root::static_readdir,
		.readdir_r_p = nullptr,
		.telldir_p = nullptr,
		.seekdir_p = nullptr,
		.closedir_p = Root::static_closedir,
		.mkdir_p = nullptr,
		.rmdir_p = nullptr,
		.access_p = nullptr,
		.truncate_p = Root::static_truncate,
		.ftruncate_p = Root::static_ftruncate,
		.utime_p = nullptr,
	};

	static const esp_vfs_fs_ops_t vfs_fs_ops =
	{
		.write_p = Root::static_write,
		.lseek_p = Root::static_lseek,
		.read_p = Root::static_read,
		.pread_p = nullptr,
		.pwrite_p = nullptr,
		.open_p = Root::static_open,
		.close_p = Root::static_close,
		.fstat_p = Root::static_fstat,
		.fcntl_p = nullptr,
		.ioctl_p = Root::static_ioctl,
		.fsync_p = nullptr,
		.dir = &vfs_dir_ops,
		.select = nullptr,
	};

	esp_err_t rv;

	if(this->singleton)
		throw(hard_exception("Ramdisk: already active"));

	this->mutex = xSemaphoreCreateMutex();

	if(!this->mutex)
		throw(hard_exception("Ramdisk: cannot create mutex"));

	if((rv = esp_vfs_register_fs(this->mountpoint.c_str(), &vfs_fs_ops, ESP_VFS_FLAG_CONTEXT_PTR | ESP_VFS_FLAG_STATIC, this)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "Ramdisk: esp_vfs_register_fs")));

	last_fileno = 0;

	this->singleton = this;
}

Root& Root::get()
{
	if(!Root::singleton)
		throw(hard_exception("Ramdisk: not active"));

	return(*singleton);
}

bool Root::file_in_use(const std::string &filename, unsigned int fcntl_flags) const
{
	const File *fp;
	FileDescriptorTable::const_iterator it;

	if(!(fp = this->root.get_file_by_name_const(filename)))
		return(false);

	if((it = this->fd_table.find(fp->get_fileno())) == this->fd_table.end())
		return(false);

	if((fcntl_flags & O_WRONLY) || (fcntl_flags & O_RDWR))
		return(true);

	if(it->second.get_fcntl_flags() & (O_WRONLY | O_RDWR))
		return(true);

	return(false);
}

DIR *Root::static_opendir(void *context, const char *name)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->opendir(std::string(name)));
}

struct dirent *Root::static_readdir(void *context, DIR *pdir)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->readdir(pdir));
}

int Root::static_closedir(void *context, DIR *pdir)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->closedir(pdir));
}

int Root::static_stat(void *context, const char *path, struct stat *st)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->stat(std::string(path), st));
}

int Root::static_fstat(void *context, int fd, struct stat *st)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->fstat(fd, st));
}

int Root::static_ioctl(void *context, int fd, int op, va_list ap)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);

	int *intp = va_arg(ap, int *);

	return(ramdisk->ioctl(fd, op, intp));
}

int Root::static_open(void *context, const char *path, int fcntl_flags, int file_access_mode)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->open(std::string(path), fcntl_flags));
}

int Root::static_close(void *context, int fd)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->close(fd));
}

int Root::static_read(void *context, int fd, void *data, size_t size)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->read(fd, size, static_cast<uint8_t *>(data)));
}

int Root::static_write(void *context, int fd, const void *data, size_t length)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->write(fd, length, static_cast<const uint8_t *>(data)));
}

off_t Root::static_lseek(void *context, int fd, off_t offset, int mode)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->lseek(fd, mode, offset));
}

int Root::static_ftruncate(void *context, int fd, off_t length)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->ftruncate(fd, length));
}

int Root::static_truncate(void *context, const char *path, off_t length)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->truncate(std::string(path), length));
}

int Root::static_unlink(void *context, const char *path)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->unlink(std::string(path)));
}

int Root::static_rename(void *context, const char *from, const char *to)
{
	Root *ramdisk = reinterpret_cast<Root *>(context);
	return(ramdisk->rename(std::string(from), std::string(to)));
}

void Root::all_stat(const File *fp, struct stat *st) const
{
	memset(st, 0, sizeof(st));

	st->st_dev = 0;
	st->st_ino = fp->get_fileno();
	st->st_mode = S_IFREG | 0777;
	st->st_size = fp->get_length();
	st->st_blksize = 512;
	st->st_blocks = fp->get_allocated() / 512UL;
	st->st_atim.tv_sec = 0;
	st->st_atim.tv_nsec = 0;
	st->st_mtim = fp->get_mtime();
	st->st_ctim = fp->get_ctime();
}

int Root::stat(const std::string &path, struct stat *st)
{
	Mutex(&this->mutex);
	const File *fp;

	if(!(fp = this->root.get_file_by_name_const(path)))
	{
		errno = ENOENT;
		return(-1);
	}

	this->all_stat(fp, st);

	return(0);
}

int Root::fstat(int fd, struct stat *st)
{
	Mutex(&this->mutex);
	FileDescriptorTable::const_iterator it;
	const File *fp;

	if((it = this->fd_table.find(fd)) == this->fd_table.end())
	{
		errno = EBADF;
		return(-1);
	}

	if(!(fp = this->root.get_file_by_fileno_const(it->second.get_fileno())))
	{
		errno = ENOENT;
		return(-1);
	}

	this->all_stat(fp, st);

	return(0);
}

int Root::ioctl(int fd, int op, int *intp)
{
	switch(op)
	{
		case(IO_RAMDISK_GET_USED):
		{
			*intp = this->root.get_used();
			break;
		}

		case(IO_RAMDISK_SET_SIZE):
		{
			this->size = *intp;
			break;
		}

		case(IO_RAMDISK_GET_SIZE):
		{
			*intp = this->size;
			break;
		}

		case(IO_RAMDISK_WIPE):
		{
			for(const auto &entry : fd_table)
			{
				if(!(entry.second.get_fcntl_flags() & O_DIRECTORY))
				{
					errno = EBUSY;
					return(-1);
				}
			}

			this->root.clear();

			break;
		}

		default:
		{
			errno = EINVAL;
			return(-1);
		}
	}

	return(0);
}

DIR *Root::opendir(const std::string &path)
{
	Mutex(&this->mutex);
	int fileno;
	Dirent *dirent;

	if((fileno = this->root.opendir(path)) < 0)
	{
		if(fileno == -ENOENT)
			fileno = -1;
		else
		{
			errno = 0 - fileno;
			return(nullptr);
		}
	}

	dirent = new Dirent(fileno);

	this->dirent_table.insert(std::pair(dirent->get_DIR(), dirent));

	return(dirent->get_DIR());
}

struct dirent *Root::readdir(DIR *pdir)
{
	Mutex(&this->mutex);
	DirentTable::iterator it;
	int rv;

	if((it = this->dirent_table.find(pdir)) == this->dirent_table.end())
	{
		errno = EINVAL;
		return(nullptr);
	}

	if((rv = this->root.readdir(it->second)) < 0)
	{
		errno = 0 - rv;
		return(nullptr);
	}

	return(it->second->get_dirent());
}

int Root::closedir(DIR *pdir)
{
	Mutex(&this->mutex);
	DirentTable::iterator it;
	int rv;

	if((it = this->dirent_table.find(pdir)) == this->dirent_table.end())
	{
		errno = EINVAL;
		return(-1);
	}

	if((rv = this->root.closedir(it->second)) < 0)
	{
		errno = 0 - rv;
		return(-1);
	}

	delete it->second;
	this->dirent_table.erase(it);

	return(0);
}

int Root::open(const std::string &path, int fcntl_flags)
{
	Mutex(&this->mutex);
	unsigned int fd, offset;
	int fileno;
	File *fp;
	bool fs;

	if(fcntl_flags & O_DIRECTORY)
	{
		if(fcntl_flags & (O_WRONLY | O_RDWR | O_APPEND | O_CREAT | O_TRUNC))
		{
			errno = EINVAL;
			return(-1);
		}

		if(path != "/")
		{
			errno = ENOENT;
			return(-1);
		}
	}
	else
	{
		if(this->file_in_use(path, fcntl_flags))
		{
			Log::get() << std::format("open: file \"{}\" in use", path.c_str());
			errno = EBUSY;
			return(-1);
		}
	}

	for(fd = 0; fd < fd_max; fd++)
		if(this->fd_table.find(fd) == this->fd_table.end())
			break;

	if(fd >= fd_max)
	{
		errno = ENOMEM;
		return(-1);
	}

	offset = 0;

	if(fcntl_flags & O_DIRECTORY)
	{
		fs = true;
		fileno = 0;
	}
	else
	{
		fs = false;

		if(fcntl_flags & O_CREAT)
			while(this->root.get_file_by_fileno_const(this->last_fileno))
				this->last_fileno++;

		if((fileno = this->root.open(path, fcntl_flags, this->last_fileno)) < 0)
		{
			errno = 0 - fileno;
			return(-1);
		}

		if(!(fp = this->root.get_file_by_fileno(fileno)))
		{
			errno = EIO;
			return(-1);
		}

		if((fcntl_flags & O_RDWR) || (fcntl_flags & O_WRONLY))
		{
			if(fcntl_flags & O_APPEND)
				offset = fp->get_length();

			if(fcntl_flags & O_TRUNC)
				fp->truncate(0);
		}
	}

	this->fd_table.insert(std::pair(fd, FileDescriptor(fd, fileno, fcntl_flags, offset, fs)));

	return(fd);
}

int Root::close(int fd)
{
	Mutex(&this->mutex);
	FileDescriptorTable::iterator it;
	int rv;

	if((it = fd_table.find(fd)) == fd_table.end())
	{
		errno = EBADF;
		return(-1);
	}

	if(!it->second.is_fs())
	{
		if((rv = this->root.close(fd)) < 0)
		{
			errno = 0 - rv;
			return(-1);
		}
	}

	this->fd_table.erase(it);

	return(0);
}

int Root::read(int fd, unsigned int data_size, uint8_t *data)
{
	Mutex(&this->mutex);
	FileDescriptorTable::iterator it;
	int received;

	if((it = this->fd_table.find(fd)) == this->fd_table.end())
	{
		errno = EBADF;
		return(-1);
	}

	if(it->second.is_fs())
	{
		errno = EINVAL;
		return(-1);
	}

	if((received = this->root.read(it->second.get_fileno(), it->second.get_offset(), data_size, data)) < 0)
	{
		errno = 0 - received;
		return(-1);
	}

	it->second.set_offset(it->second.get_offset() + received);

	return(received);
}

int Root::write(int fd, unsigned int length, const uint8_t *data)
{
	Mutex(&this->mutex);
	FileDescriptorTable::iterator it;
	int written;

	if((it = this->fd_table.find(fd)) == this->fd_table.end())
	{
		errno = EBADF;
		return(-1);
	}

	if(it->second.is_fs())
	{
		errno = EINVAL;
		return(-1);
	}

	if(this->root.get_used() > this->size)
	{
		errno = ENOSPC;
		return(-1);
	}

	if((written = this->root.write(it->second.get_fileno(), it->second.get_offset(), length, data)) < 0)
	{
		errno = 0 - written;
		return(-1);
	}

	it->second.set_offset(it->second.get_offset() + written);

	return(written);
}

int Root::lseek(int fd, unsigned int mode, int delta_offset)
{
	Mutex(&this->mutex);
	FileDescriptorTable::iterator it;
	const File *fp;
	int new_offset;

	if((it = this->fd_table.find(fd)) == this->fd_table.end())
	{
		errno = EBADF;
		return(-1);
	}

	if(it->second.is_fs())
	{
		errno = EINVAL;
		return(-1);
	}

	if(!(fp = this->root.get_file_by_fileno_const(it->second.get_fileno())))
	{
		errno = ENOENT;
		return(-1);
	}

	switch(mode)
	{
		case(SEEK_SET):
		{
			new_offset = delta_offset;
			break;
		}

		case(SEEK_CUR):
		{
			new_offset = it->second.get_offset() + delta_offset;
			break;
		}

		case(SEEK_END):
		{
			new_offset = fp->get_length() + delta_offset;
			break;
		}

		default:
		{
			errno = ENOENT;
			return(-1);
		}
	}

	if((new_offset < 0) || (new_offset >= fp->get_length()))
	{
		errno = EINVAL;
		return(-1);
	}

	it->second.set_offset(new_offset);

	return(new_offset);
}

int Root::truncate(const std::string &path, unsigned int length)
{
	Mutex(&this->mutex);
	FileDescriptorTable::iterator it;
	File *fp;
	int rv;

	if(!(fp = this->root.get_file_by_name(path)))
	{
		errno = ENOENT;
		return(-1);
	}

	for(it = this->fd_table.begin(); it != this->fd_table.end(); it++)
	{
		if(it->second.get_fileno() == fp->get_fileno())
		{
			errno = EBUSY;
			return(-1);
		}
	}

	if((rv = fp->truncate(length)) < 0)
	{
		errno = 0 - rv;
		return(-1);
	}

	return(0);
}

int Root::ftruncate(int fd, unsigned int length)
{
	Mutex(&this->mutex);
	FileDescriptorTable::iterator it;
	File *fp;
	int rv;

	if((it = this->fd_table.find(fd)) == this->fd_table.end())
	{
		errno = EBADF;
		return(-1);
	}

	if(it->second.is_fs())
	{
		errno = EINVAL;
		return(-1);
	}

	if(!(fp = this->root.get_file_by_fileno(it->second.get_fileno())))
	{
		errno = ENOENT;
		return(-1);
	}

	if((rv = fp->truncate(length)) < 0)
	{
		errno = 0 - rv;
		return(-1);
	}

	if(it->second.get_offset() > fp->get_length())
		it->second.set_offset(fp->get_length());

	return(0);
}

int Root::unlink(const std::string &path)
{
	Mutex(&this->mutex);
	FileDescriptorTable::iterator it;
	const File *fp;
	int rv;

	Log::get() << std::format("unlink(\"{}\")", path.c_str()); // FIXME

	if(!(fp = this->root.get_file_by_name_const(path)))
	{
		errno = ENOENT;
		return(-1);
	}

	for(it = this->fd_table.begin(); it != this->fd_table.end(); it++)
	{
		if(it->second.get_fileno() == fp->get_fileno())
		{
			errno = EBUSY;
			return(-1);
		}
	}

	if((rv = this->root.unlink(path)) < 0)
	{
		errno = 0 - rv;
		return(-1);
	}

	return(0);
}

int Root::rename(const std::string &from, const std::string &to)
{
	Mutex(&this->mutex);
	const File *fp;
	FileDescriptorTable::const_iterator it;
	unsigned int fileno;
	int rv;

	if((fp = this->root.get_file_by_name(to)))
	{
		fileno = fp->get_fileno();

		for(it = this->fd_table.begin(); it != this->fd_table.end(); it++)
			if(it->second.get_fileno() == fileno)
				break;

		if(it != this->fd_table.end())
		{
			errno = EEXIST;
			return(-1);
		}
	}

	if((rv = this->root.rename(from, to)) < 0)
	{
		errno = 0 - rv;
		return(-1);
	}

	return(0);
}

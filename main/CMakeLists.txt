idf_component_register(
	SRCS
		"alias.cpp"
		"bt.cpp"
		"cli.cpp"
		"command.cpp"
		"config.cpp"
		"console.cpp"
		"display.cpp"
		"display-spi-generic.cpp"
		"display-ra8875.cpp"
		"esp32-common/encryption.cpp"
		"esp32-common/exception.cpp"
		"fs.cpp"
        "i2c.cpp"
		"info.cpp"
		"io.cpp"
		"init.cpp"
		"ledpixel.cpp"
		"ledpwm.cpp"
		"log.cpp"
		"mcpwm.cpp"
		"notify.cpp"
		"ota.cpp"
		"esp32-common/packet.cpp"
		"perftest.cpp"
		"pdm.cpp"
		"process.cpp"
		"ramdisk.cpp"
		"script.cpp"
		"sensor.cpp"
		"tcp.cpp"
		"udp.cpp"
		"util.cpp"
		"wlan.cpp"
	INCLUDE_DIRS
		"")

include_directories(esp32-common)

#		-Wtrailing-whitespace=any
#		-Wleading-whitespace=tabs

set(ALLFLAGS
		-Wall
		-Wextra
		-Werror
		-Wdouble-promotion
		-Wformat=2
		-Wformat-overflow=2
		-Wformat-signedness
		-Wnull-dereference
		-Wshift-overflow=2
		-Wimplicit-fallthrough=5
		-Wunused-const-variable=2
		-Wuse-after-free=3
		-Wuninitialized
		-Wstrict-overflow=2
		-Wstringop-overflow=4
		-Wsuggest-attribute=noreturn
		-Wsuggest-attribute=format
		-Walloc-size
		-Walloc-zero
		-Wcalloc-transposed-args
		-Warray-bounds=2
		-Wduplicated-branches
		-Wduplicated-cond
		-Wtrampolines
		-Wfloat-equal
		-Wshadow
		-Wframe-larger-than=2048
		-Wunsafe-loop-optimizations
		-Wpointer-arith
		-Wcast-qual
		-Wcast-align
		-Wwrite-strings
		-Wdangling-pointer=2
		-Wlogical-op
		-Wmissing-declarations
		-Winvalid-utf8
		-Woverlength-strings
		-Wmissing-noreturn
		-Wmissing-format-attribute
		-Wno-inline
		-Wno-redundant-decls
		-Wno-packed
		-Wno-conversion
		-Wno-float-conversion
		-Wno-sign-conversion
		-Wno-vla
		-fno-strict-aliasing
)

#		-Wmissing-include-dirs
#		-Wdeprecated-non-prototype

set(CFLAGS
		-std=gnu23
		-Wbad-function-cast
		-Wstrict-prototypes
		-Wold-style-definition
		-Wmissing-prototypes
		-Wmissing-variable-declarations
		-Wdiscarded-array-qualifiers
		-Wno-pointer-sign
)

set(CXXFLAGS
)

#		-Wctor-dtor-privacy // breaks boost::format
#		-Wzero-as-null-pointer-constant // breaks boost::format

string(JOIN " " ALLCFLAGS ${ALLFLAGS} ${CFLAGS})
string(JOIN " " ALLCXXFLAGS ${ALLFLAGS} ${CXXFLAGS})

FILE(GLOB CSOURCES *.c)
FILE(GLOB CXXSOURCES *.cpp)

set_source_files_properties(${CSOURCES}
		PROPERTIES COMPILE_FLAGS ${ALLCFLAGS}
)

set_source_files_properties(${CXXSOURCES}
		PROPERTIES COMPILE_FLAGS ${ALLCXXFLAGS}
)

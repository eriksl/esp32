idf_component_register(
	SRCS
		"bt.c"
		"cli.c"
		"config.c"
		"console.c"
		"display.c"
		"display-spi-generic.c"
		"fs.c"
        "i2c.c"
		"info.c"
		"io.c"
		"init.c"
		"ledpixel.c"
		"ledpwm.c"
		"log.c"
		"mcpwm.c"
		"notify.c"
		"packet.c"
		"perftest.c"
		"ota.c"
		"pdm.c"
		"ramdisk.c"
		"string.c"
		"sensor.c"
		"util.c"
		"wlan.c"
	INCLUDE_DIRS
		"")

set(ALLFLAGS
		-Wall
		-Wextra
		-Werror
		-Wformat-overflow=2
		-Wshift-overflow=2
		-Wimplicit-fallthrough=5
		-Wformat-signedness
		-Wformat-truncation=1
		-Wstringop-overflow=4
		-Wunused-const-variable=2
		-Warray-bounds=2
		-Wswitch-bool
		-Wduplicated-branches
		-Wduplicated-cond
		-Wlto-type-mismatch
		-Wnull-dereference
		-Wpacked
		-Wfloat-equal
		-Wmissing-include-dirs
		-Wmissing-noreturn
		-Wbool-compare
		-Wsuggest-attribute=noreturn
		-Wsuggest-attribute=format
		-Wmissing-format-attribute
		-Wtrampolines
		-Wframe-larger-than=2048
		-Wunsafe-loop-optimizations
		-Wshadow
		-Wpointer-arith
		-Wcast-qual
		-Wwrite-strings
		-Wlogical-op
		-Wvla
		-Wdisabled-optimization
		-Wunreachable-code
		-Wmissing-declarations
		-Wcast-align
		-Wmultistatement-macros
		-Warray-bounds=2
		-Wdouble-promotion
		-Wno-redundant-decls
		-Wno-inline
		-Wno-packed
		-fno-strict-aliasing
)

set(CFLAGS
		-std=gnu23
		-Wbad-function-cast
		-Wmissing-prototypes
		-Wold-style-definition
		-Wno-pointer-sign
		-Wenum-int-mismatch
		-Wdiscarded-array-qualifiers
)

set(CXXFLAGS
		-Wctor-dtor-privacy
		-Weffc++
		-Wold-style-cast
		-Woverloaded-virtual
)

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

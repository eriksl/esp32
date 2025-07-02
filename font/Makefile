# generate bitmap fonts for graphical displays

MAKEFONT =			./makefont
FONT_CODE_RANGE =	913-937,945-969,8211,8212,8224-8226,8364,8482,8486,9834-9835,57344-57347

MAKEFLAGS += --no-builtin-rules

.PRECIOUS:				$(MAKEFONT).c Makefile *.bdf
.PHONY:					all

all:					$(MAKEFONT)
						$(MAKEFONT)	spleen-5x8.bdf	font_5x8.fnt	$(FONT_CODE_RANGE)	 5  8 8 3 0 0
						$(MAKEFONT)	ter-u12b.bdf	font_6x12.fnt	$(FONT_CODE_RANGE)	 6 12 8 2 0 0
						$(MAKEFONT)	ter-u14b.bdf	font_7x14.fnt	$(FONT_CODE_RANGE)	 8 14 8 1 0 0
						$(MAKEFONT)	ter-u18b.bdf	font_10x18.fnt	$(FONT_CODE_RANGE)	10 18 0 6 0 0
						$(MAKEFONT)	ter-u24b.bdf	font_12x23.fnt	$(FONT_CODE_RANGE)	12 24 0 4 1 0
						$(MAKEFONT)	ter-u28b.bdf	font_14x27.fnt	$(FONT_CODE_RANGE)	14 28 0 2 1 0
						$(MAKEFONT)	ter-u32b.bdf	font_16x30.fnt	$(FONT_CODE_RANGE)	16 32 0 1 2 0

$(MAKEFONT):			$(MAKEFONT).c
						gcc -Wall -Werror -g -O0 -s $< -lssl -lcrypto -o $@

clean:
						rm -f $(MAKEFONT) $(BIN)

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <openssl/evp.h>

typedef enum
{
	s_start,
	s_from,
	s_minus,
	s_to,
	s_comma,
} range_state_t;

enum
{
	ranges_size = 16,
	basic_glyphs_size = 256,
	extra_glyphs_size = 128,
	cols_size = 16,
	rows_size = 32,
};

typedef enum
{
	magic_word = 0xf0bdf11e,
} magic_word_t;

typedef struct __attribute__((packed))
{
	uint32_t codepoint;
	uint16_t row[rows_size];
} glyph_t;

_Static_assert(sizeof(glyph_t) == 68);

typedef struct __attribute__((packed))
{
	magic_word_t magic_word:32;
	uint8_t checksum[32];
	struct
	{
		uint32_t width;
		uint32_t height;
	} raw;
	struct
	{
		uint32_t width;
		uint32_t height;
	} net;
	uint32_t extra_glyphs;
	glyph_t basic_glyph[basic_glyphs_size];
	glyph_t extra_glyph[extra_glyphs_size];
} font_t;

_Static_assert(offsetof(font_t, basic_glyph) == 56);
_Static_assert(offsetof(font_t, extra_glyph) == 17464);

int main(int argc, char *const argv[])
{
	font_t font;
	glyph_t *glyph;
	FILE *input, *output;
	const char *range_string;
	unsigned int range[ranges_size][2];
	range_state_t range_state;
	unsigned int current_char;
	unsigned int current_range;
	unsigned int current_value_from;
	unsigned int current_value_to;
	unsigned int font_width, font_height;
	unsigned int width, height;
	unsigned int shave_left, shave_right;
	unsigned int shave_top, shave_bottom;
	unsigned int range_index;
	char linebuf[1024], keyword[1024], sarg1[1024], sarg2[1024];
	int iarg1, iarg2, iarg3, iarg4;
	bool bitmap_state, include_glyph;
	char current_char_name[1024];
	unsigned int current_char_codepoint;
	unsigned int hexval, hexvaladd;
	const uint8_t *lptr;
	unsigned int current_row_input, current_row_output;
	unsigned int ix;
	int bitindex;
	unsigned int binval, binbitindex;
	uint8_t bin_buffer[4];
	EVP_MD_CTX *hash_ctx;
	uint8_t hash[32];
	unsigned int hash_size = 32;
	char hash_digit[8];
	char hash_text[128];

	if(argc < 10)
	{
		fprintf(stderr, "usage: makefont <bdf font file> <output font file> <included code points> <width> <height> <shave_left> <shave_right> <shave_top> <shave_bottom>\n");
		return(-1);
	}

	if((input = fopen(argv[1], "r")) == (FILE *)0)
	{
		perror("input bdf font file cannot be opened");
		exit(-1);
	}

	unlink(argv[2]);

	if((output = fopen(argv[2], "w")) == (FILE *)0)
	{
		perror("output binary file cannot be opened");
		exit(-1);
	}

	for(range_index = 0; range_index < ranges_size; range_index++)
		range[range_index][0] = range[range_index][1] = 0;

	range_string = argv[3];

	current_range = 0;
	current_value_from = 0;
	current_value_to = 0;
	range_state = s_start;

	for(current_char = 0; (current_char < strlen(range_string)) && (current_range < ranges_size); current_char++)
	{
		switch(range_state)
		{
			case(s_start):
			{
				if((range_string[current_char] >= '0') && (range_string[current_char] <= '9'))
				{
					current_value_from = range_string[current_char] - '0';
					range_state = s_from;
					continue;
				}

				fprintf(stderr, "range: invalid syntax (1)\n");
				exit(1);
			}

			case(s_from):
			{
				if((range_string[current_char] >= '0') && (range_string[current_char] <= '9'))
				{
					current_value_from = (current_value_from * 10) + (range_string[current_char] - '0');
					continue;
				}

				if(range_string[current_char] == '-')
				{
					range_state = s_minus;
					continue;
				}

				if(range_string[current_char] == ',')
				{
					range[current_range][0] = current_value_from;
					range[current_range][1] = current_value_from;
					current_value_from = current_value_to = 0;
					current_range++;

					range_state = s_start;
					continue;
				}

				fprintf(stderr, "range: invalid syntax (2)\n");
				exit(1);
			}

			case(s_minus):
			{
				if(current_value_from < 256)
				{
					fprintf(stderr, "range-from invalid: %u (< 256)\n", current_value_from);
					exit(1);
				}

				if((range_string[current_char] >= '0') && (range_string[current_char] <= '9'))
				{
					current_value_to = range_string[current_char] - '0';
					range_state = s_to;
					continue;
				}

				fprintf(stderr, "range: invalid syntax (3)\n");
				exit(1);
			}

			case(s_to):
			{
				if((range_string[current_char] >= '0') && (range_string[current_char] <= '9'))
				{
					current_value_to = (current_value_to * 10) + (range_string[current_char] - '0');
					continue;
				}

				if(range_string[current_char] == ',')
				{
					range[current_range][0] = current_value_from;
					range[current_range][1] = current_value_to;

					if(range[current_range][0] > range[current_range][1])
					{
						fprintf(stderr, "invalid range [%u,%u]", range[current_range][0], range[current_range][1]);
						exit(1);
					}

					current_value_from = current_value_to = 0;
					current_range++;
					range_state = s_start;

					continue;
				}

				fprintf(stderr, "range: invalid syntax (2)\n");
				exit(1);
			}

			default:
			{
				fprintf(stderr, "range: internal error\n");
				exit(1);
			}
		}
	}

	if((current_range < ranges_size) && (current_value_from > 0))
	{
		if(current_value_to == 0)
			current_value_to = current_value_from;

		range[current_range][0] = current_value_from;
		range[current_range][1] = current_value_to;
		current_value_from = current_value_to = 0;
		current_range++;
	}

	font_width = strtoul(argv[4], (char **)0, 0);
	font_height = strtoul(argv[5], (char **)0, 0);
	shave_left = strtoul(argv[6], (char **)0, 0);
	shave_right = strtoul(argv[7], (char **)0, 0);
	shave_top = strtoul(argv[8], (char **)0, 0);
	shave_bottom = strtoul(argv[9], (char **)0, 0);

	if(font_width > cols_size)
	{
		fprintf(stderr, "width must be less/equal to %u\n", cols_size);
		exit(1);
	}

	if(font_height > rows_size)
	{
		fprintf(stderr, "height must be less/equal to %u\n", rows_size);
		exit(1);
	}

	if((shave_left + shave_right) > cols_size)
	{
		fprintf(stderr, "invalid values for shave left/right\n");
		exit(1);
	}

	if((shave_top + shave_bottom) > rows_size)
	{
		fprintf(stderr, "invalid values for shave top/bottom\n");
		exit(1);
	}

	width =	cols_size - shave_left - shave_right;
	height = font_height - shave_top - shave_bottom;

	printf("*** font dimensions: %u[16-%u-%u] x %u[-%u-%u] -> %u x %u\n",
			font_width, shave_left, shave_right,
			font_height, shave_top, shave_bottom,
			width, height);

	for(range_index = 0; (range_index < ranges_size) && (range[range_index][1] > 0); range_index++)
		printf("*** range[%u]: from %u to %u\n", range_index, range[range_index][0], range[range_index][1]);

	font.magic_word = magic_word;
	memset(&font.checksum, 0, sizeof(font.checksum));
	font.raw.width = font_width;
	font.raw.height = font_height;
	font.net.width = width;
	font.net.height = height;
	font.extra_glyphs = 0;

	for(ix = 0; ix < basic_glyphs_size; ix++)
	{
		glyph = &font.basic_glyph[ix];
		glyph->codepoint = 0;

		for(current_row_output = 0; current_row_output < rows_size; current_row_output++)
			glyph->row[current_row_output] = 0;
	}

	for(ix = 0; ix < extra_glyphs_size; ix++)
	{
		glyph = &font.extra_glyph[ix];
		glyph->codepoint = 0;

		for(current_row_output = 0; current_row_output < rows_size; current_row_output++)
			glyph->row[current_row_output] = 0;
	}

	bitmap_state = false;
	current_char_codepoint = 0;
	current_row_input = 0;
	current_row_output = 0;
	glyph = (glyph_t *)0;

	while(fgets(linebuf, sizeof(linebuf), input))
	{
		if(!bitmap_state)
		{
			if(sscanf(linebuf, "%1024s", keyword) >= 1)
			{
				if(!strcmp(keyword, "FONTBOUNDINGBOX"))
				{
					if(sscanf(linebuf, "%1024s %d %d %d %d", sarg1, &iarg1, &iarg2, &iarg3, &iarg4) == 5)
					{
						if((font_width != iarg1) || (font_height != iarg2))
						{
							fprintf(stderr, "dimensions not constant\n");
							exit(1);
						}
					}
				}
				else
				{
					if(!strcmp(keyword, "CHARSET_REGISTRY"))
					{
						if((sscanf(linebuf, "%1024s %1024s", sarg1, sarg2) == 2) && strcmp("\"ISO10646\"", sarg2))
						{
							fprintf(stderr, "encoding should be ISO 10646\n");
							exit(1);
						}
					}
					else
					{
						if(!strcmp(keyword, "STARTCHAR"))
						{
							if(sscanf(linebuf, "%1024s %1024s", sarg1, sarg2) == 2)
							{
								unsigned int len;

								strncpy(current_char_name, linebuf + 10, sizeof(current_char_name));
								len = strlen(current_char_name);

								if(current_char_name[len - 1] == '\n')
									current_char_name[len - 1] = '\0';
							}
						}
						else
						{
							if(!strcmp(keyword, "ENCODING"))
							{
								if(sscanf(linebuf, "%1024s %d", sarg1, &iarg1) == 2)
								{
									current_char_codepoint = (unsigned int)iarg1;

									include_glyph = false;
									glyph = (glyph_t *)0;

									if(current_char_codepoint < basic_glyphs_size)
										include_glyph = true;
									else
									{
										for(range_index = 0; (range_index < ranges_size) && (range[range_index][1] > 0); range_index++)
										{
											if((range[range_index][0] <= current_char_codepoint) && (range[range_index][1] >= current_char_codepoint))
											{
												include_glyph = true;
												break;
											}
										}
									}

									if(include_glyph)
									{
										if(current_char_codepoint < basic_glyphs_size)
											glyph = &font.basic_glyph[current_char_codepoint];
										else
										{
											if((font.extra_glyphs + 1) >= extra_glyphs_size)
											{
												fprintf(stderr, "out of memory allocated for extra glyphs: %u\n", font.extra_glyphs);
												exit(1);
											}

											glyph = &font.extra_glyph[font.extra_glyphs++];
										}

										glyph->codepoint = current_char_codepoint;
									}

									printf("%s code %3u: %s\n", include_glyph ? "+" : "-", current_char_codepoint, current_char_name);

									current_row_input = 0;
									current_row_output = 0;
								}
							}
							else
								if(!strcmp(linebuf, "BITMAP\n"))
									bitmap_state = true;
						}
					}
				}
			}
		}
		else
		{
			if(!strcmp("ENDCHAR\n", linebuf))
				bitmap_state = false;
			else
			{
				if(include_glyph)
				{
					for(lptr = (const uint8_t *)linebuf, hexval = 0; *lptr != '\0'; lptr++)
					{
						hexvaladd = 0;

						if((*lptr >= '0') && (*lptr <= '9'))
							hexvaladd = *lptr - '0';
						else
							if((*lptr >= 'a') && (*lptr <= 'f'))
								hexvaladd = (*lptr - 'a') + 10;
							else
								if((*lptr >= 'A') && (*lptr <= 'F'))
									hexvaladd = (*lptr - 'A') + 10;
								else
									break;

						hexval <<= 4;
						hexval |= hexvaladd;
					}

					if((current_row_input >= shave_top) && (current_row_input < (height - shave_bottom)))
					{
						if(current_row_input >= rows_size)
						{
							fprintf(stderr, "too many input rows in code point %u: %u\n", current_char_codepoint, current_row_input);
							exit(1);
						}

						if(current_row_output >= rows_size)
						{
							fprintf(stderr, "too many output rows in code point %u: %u\n", current_char_codepoint, current_row_output);
							exit(1);
						}

						for(bitindex = (cols_size - 1) - shave_left, binval = 0, binbitindex = 0; bitindex >= (int)shave_right; bitindex--, binbitindex++)
							if(hexval & (1 << bitindex))
								binval |= 1 << binbitindex;

						glyph->row[current_row_output++] = binval;
					}
				}

				current_row_input++;
			}
		}
	}

	for(ix = 0; ix < basic_glyphs_size; ix++)
	{
		glyph = &font.basic_glyph[ix];

		printf("* basic glyph %u/%u\n", ix, glyph->codepoint);

		for(current_row_output = 0; current_row_output < rows_size; current_row_output++)
		{
			printf("%2u: ", current_row_output);

			binval = glyph->row[current_row_output];

			for(bitindex = 0; bitindex < cols_size; bitindex++)
				printf("%c", (binval & (1 << bitindex)) ? 'X' : '.');

			bin_buffer[0] = (binval & 0x000000ff) >> 0;
			bin_buffer[1] = (binval & 0x0000ff00) >> 8;
			bin_buffer[2] = (binval & 0x00ff0000) >> 16;
			bin_buffer[3] = (binval & 0xff000000) >> 24;

			printf(" %04x %02x%02x%02x%02x\n", binval, bin_buffer[0], bin_buffer[1], bin_buffer[2], bin_buffer[3]);
		}
	}

	for(ix = 0; ix < extra_glyphs_size; ix++)
	{
		glyph = &font.extra_glyph[ix];

		printf("* extra glyph %u/%u\n", ix, glyph->codepoint);

		for(current_row_output = 0; current_row_output < rows_size; current_row_output++)
		{
			printf("%2u: ", current_row_output);

			binval = glyph->row[current_row_output];

			for(bitindex = 0; bitindex < cols_size; bitindex++)
				printf("%c", (binval & (1 << bitindex)) ? 'X' : '.');

			bin_buffer[0] = (binval & 0x000000ff) >> 0;
			bin_buffer[1] = (binval & 0x0000ff00) >> 8;
			bin_buffer[2] = (binval & 0x00ff0000) >> 16;
			bin_buffer[3] = (binval & 0xff000000) >> 24;

			printf(" %04x %02x%02x%02x%02x\n", binval, bin_buffer[0], bin_buffer[1], bin_buffer[2], bin_buffer[3]);
		}
	}

	fclose(input);

	hash_ctx = EVP_MD_CTX_new();
	EVP_DigestInit_ex(hash_ctx, EVP_sha256(), (ENGINE *)0);
	EVP_DigestUpdate(hash_ctx, &font, sizeof(font));
	EVP_DigestFinal_ex(hash_ctx, hash, &hash_size);
	EVP_MD_CTX_free(hash_ctx);
	*hash_text = '\0';

	for(ix = 0; ix < hash_size; ix++)
	{
		snprintf(hash_digit, sizeof(hash_digit), "%02x", (unsigned int)hash[ix]);
		strcat(hash_text, hash_digit);
	}

	printf("*** font finished, %u basic glyphs, %u extra glyphs, checksum: %s\n", basic_glyphs_size, font.extra_glyphs, hash_text);

	memcpy(&font.checksum, hash, sizeof(font.checksum));
	fwrite(&font, sizeof(font), 1, output);

	fclose(output);
}

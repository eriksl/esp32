#ifndef _log_h_
#define _log_h_

// ugly workaround to get #define argument number overloading
#define _GET_MACRO_(_1, _2, _3, _4, _5, _NAME_, ...) _NAME_
#define log(...) _GET_MACRO_(__VA_ARGS__, _log_4_, _log_3_, _log_2_, _log_1_, _log_0_)(__VA_ARGS__)

#define _log_0_(f) \
do { \
	log_simple(f); \
} while(0)

#define _log_1_(f, p1) \
do { \
	log_vargs(f, p1); \
} while(0)

#define _log_2_(f, p1, p2) \
do { \
	log_vargs(f, p1, p2); \
} while(0)

#define _log_3_(f, p1, p2, p3) \
do { \
	log_vargs(f, p1, p2, p3); \
} while(0)

#define _log_4_(f, p1, p2, p3, p4) \
do { \
	log_vargs(f, p1, p2, p3, p4); \
} while(0)

void log_init(void);
void log_simple(const char *string);
void log_vargs(const char *f, ...) __attribute__ ((format (printf, 1, 2)));

void command_info_log(cli_command_call_t *call);
void command_log(cli_command_call_t *call);
void command_log_clear(cli_command_call_t *call);

#endif

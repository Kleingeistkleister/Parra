#ifndef PYHELPER_H
#define PYHELPER_H

// get_monotonic is a function that returns the current time in seconds
// since an arbitrary point in the past.  It is used to measure time
// intervals and to schedule events.  The time returned by get_monotonic
// is not related to the system clock and is not affected by changes to
// the system clock.

double get_monotonic(void);

// The pollreactor is a simple event loop that can monitor file
// descriptors for readability and timers for expiration.  The
// pollreactor is used to implement the serial port and network
// communication in Klipper.
struct timespec fill_time(double time);

// The set_python_logging_callback function is used to set a callback
// function that will be called when the Klipper code logs a message.
// The callback function should take a single string argument,

void set_python_logging_callback(void (*func)(const char *));
// The set_python_logging_callback function is used to set a callback
// function that will be called when the Klipper code logs a message.
// The callback function should take a single string argument,

void errorf(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

// The set_python_logging_callback function is used to set a callback
// function that will be called when the Klipper code logs a message.
// The callback function should take a single string argument,
// and should return a string that will be logged by the Klipper code.

// The set_python_logging_callback function is used to set a callback
// function that will be called when the Klipper code logs a message.
// The callback function should take a single string argument,
// and should return a string that will be logged by the Klipper code.

void report_errno(char *where, int rc);

// The set_python_logging_callback function is used to set a callback
// function that will be called when the Klipper code logs a message.
// The callback function should take a single string argument,
char *dump_string(char *outbuf, int outbuf_size, char *inbuf, int inbuf_size);

#endif // pyhelper.h

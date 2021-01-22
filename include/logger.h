#ifndef LOGGER_H
#define LOGGER_H

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>

/**
 * Linkage of logger function is internal, i.e., only visible to
 * the logger.h translation unit. Making the function static ou-
 * tside a class or struct is similar to making the scope of a
 * function private in a class.
 */
static void logger(int& t_argc, char** t_argvptr)
{
    google::ParseCommandLineFlags(&t_argc, &t_argvptr, true);
    google::InitGoogleLogging(t_argvptr[0]);
    google::SetLogDestination(google::GLOG_INFO, "Log.txt");
}
#endif /* LOGGER_H */

# Logging

## Environment variables

* XLOG_LEVEL: the log level. Default: 2
    - 0: Trace, 1: Debug, 2: Info, 3: Warn, 4: Error, 5: Fatal, 6: Off
* XLOG_ENABLE_LOGFILE: whether to enable the log file (TRUE/true/1 to enable). Default: false
* XLOG_FOLDER: the folder where log files are stored. Default: ~/.xmotion/log

## Known limitations

* You need to make sure you have write access to the folder you specified for the log files, if XLOG_ENABLE_LOGFILE is
  TRUE.
* Do not make any logging calls after a signal is received. This may result in an undefined behavior as memory may not
  be handled properly.
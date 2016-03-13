#ifndef LOG_PARSER_H
#define LOG_PARSER_H

#include <string>
#include <vector>

class LogParser {
public:
    LogParser();
    ~LogParser();

public:
    std::vector<std::string> log_head_;
    std::vector<std::vector<double>> log_data_;

private:
    void ProcessLogHead(std::string log_head);
    void ProcessLogEntry(std::string entry_str);

public:
    void ParseLogFile(std::string file);
};

#endif // LOG_PARSER_H

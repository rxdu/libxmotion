#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "log_parser.h"

LogParser::LogParser()
{

}

LogParser::~LogParser()
{
    log_head_.clear();
    log_data_.clear();
}

void LogParser::ParseLogFile(std::string file)
{
    std::ifstream log_data_s(file);
    std::string log_entry;

    // clear data from last function call
    log_head_.clear();
    log_data_.clear();

    // line starts from 1, used to index log entries
    unsigned long line_num = 0;

    while (std::getline(log_data_s, log_entry))
    {
        // increase line num first
        line_num++;

        // line 1: log file information
        // line 2: empty line
        // line 3: headers of data
        // line 4 ~ end: data
        if(line_num < 4) {
            if(line_num == 3)
                ProcessLogHead(log_entry);
            continue;
        }

//        std::cout << log_entry << std::endl;

        // there is an empty entry after the last line of data
        if(!log_entry.empty())
            ProcessLogEntry(log_entry);
        else
            break;
    }

//    for(auto it = log_data_.begin(); it != log_data_.end(); it++) {
//        for(auto itv = (*it).begin(); itv != (*it).end(); itv++)
//            std::cout<<(*itv) << " , ";
//        std::cout << std::endl;
//    }
}

void LogParser::ProcessLogHead(std::string log_head)
{
    std::stringstream entry_stream(log_head);
    std::string head;
    unsigned int title_index = 0;

    while (std::getline(entry_stream, head, ','))
    {
        // remove space and '\t'
        head.erase(std::remove(head.begin(),head.end(),' '),head.end());
        head.erase(std::remove(head.begin(),head.end(),'\t'),head.end());

        if(title_index > 1)
            log_head_.push_back(head);

        title_index++;
    }

//    for(auto it = log_titles_.begin(); it != log_titles_.end(); it++)
//        std::cout << (*it) << std::endl;
}

void LogParser::ProcessLogEntry(std::string entry_str)
{
    std::stringstream entry_stream(entry_str);
    std::string entry_data;
    std::vector<std::string> str;

    while (std::getline(entry_stream, entry_data, ','))
    {
        // remove space and '\t'
        entry_data.erase(std::remove(entry_data.begin(),entry_data.end(),' '),entry_data.end());
        entry_data.erase(std::remove(entry_data.begin(),entry_data.end(),'\t'),entry_data.end());

        //        std::cout << entry_data << std::endl;
        str.push_back(entry_data);
    }

    // data at index 1 is logging level - only process INFO level
    if(!str.empty()) {
        if(str[2].compare("INFO"))
        {
            std::vector<double> vec;

            for(auto it = str.begin(); it != str.end(); it++)
            {
                if(it != str.begin()+1)
                    vec.push_back(std::stod((*it)));
            }

            log_data_.push_back(vec);
        }
    }
}

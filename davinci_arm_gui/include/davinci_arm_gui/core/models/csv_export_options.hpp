#pragma once

#include <string>
#include <vector>

namespace davinci_arm::models {

struct CsvExportOptions {
    std::vector<std::string> columns{};
    int decimals{6};
    bool include_header_comments{true};
    bool sort_by_time{true};
};

}  // namespace davinci_arm::models

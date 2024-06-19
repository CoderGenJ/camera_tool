#include <dirent.h>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace dir_common {
bool is_directory(const std::string &path);
void traverse_directory(const std::string &dir_path,
                        std::vector<std::string> &file_list);
} // namespace dir_common

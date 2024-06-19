#include <dirent.h>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace dir_common {
// Function to check if the path is a directory
bool is_directory(const std::string &path) {
  struct stat statbuf;
  if (stat(path.c_str(), &statbuf) != 0) {
    return false;
  }
  return S_ISDIR(statbuf.st_mode);
}

void traverse_directory(const std::string &dir_path,
                        std::vector<std::string> &file_list) {
  DIR *dir = opendir(dir_path.c_str());
  if (dir == nullptr) {
    std::cerr << "The path specified is not a valid directory: " << dir_path
              << std::endl;
    return;
  }

  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    std::string entry_name = entry->d_name;
    if (entry_name == "." || entry_name == "..") {
      continue;
    }

    std::string full_path = dir_path + "/" + entry_name;

    if (is_directory(full_path)) {
      traverse_directory(full_path, file_list);
    } else {
      file_list.push_back(full_path);
    }
  }
  closedir(dir);
}

} // namespace dir_common
//
// Created by zhou on 9/30/19.
//

#include "util/file_io_util.h"

std::vector<std::string> GetFilesInFolder(const std::string &cate_dir) {
    std::vector<std::string> files;

    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(cate_dir.c_str())) == NULL) {
        std::cout<<"Open dir error..."<<'\n';
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL) {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8) ///file
            files.push_back(ptr->d_name);
        else if (ptr->d_type == 10) ///link file
            continue;
        else if (ptr->d_type == 4) ///dir
        {
            files.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    return files;
}



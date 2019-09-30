//
// Created by zhou on 9/30/19.
//

#include "util/str_util.h"

std::string Int2FixedString(int num, unsigned fix_num) {
    char str[99];
    sprintf(str, "%0*d", fix_num, num);
    return str;
}

std::vector <std::string> SplitString(const std::string &src,
                                      const std::string &separator) {

    std::vector <std::string> dst;

    std::string temp;
    size_t pos = 0, offset = 0;

    while ((pos = src.find_first_of(separator, offset)) != std::string::npos) {
        temp = src.substr(offset, pos - offset);
        if (temp.length() > 0) {
            dst.push_back(temp);
        }
        offset = pos + 1;
    }

    temp = src.substr(offset, src.length() - offset);
    if (temp.length() > 0) {
        dst.push_back(temp);
    }

    return dst;
}



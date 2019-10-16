//
// Created by zhou on 9/27/19.
//

#ifndef SRC_STR_UTIL_H
#define SRC_STR_UTIL_H

#include <cstdio>
#include <vector>
#include <string>

std::string Int2FixedString(int num, unsigned fix_num);

std::string Int2FixedStringWithSign(int num, unsigned fix_num);

double String2Double(std::string num);

std::string Double2String(double num);

std::vector<std::string> SplitString(const std::string &src,
                                     const std::string &separator);


#endif //SRC_STR_UTIL_H

//
// Created by zhou on 9/30/19.
//

#include "util/str_util.h"

std::string Int2FixedString(int num, unsigned fix_num) {
    char str[99];
    sprintf(str, "%0*d", fix_num, num);
    return str;
}

std::string Int2FixedStringWithSign(int num, unsigned fix_num)
{
    char str[9999];
    if (num > 0)
        sprintf(str, "+%0*d", fix_num - 1, num);
    else
        sprintf(str, "%0*d", fix_num, num);
    return str;
}

double String2Double(std::string num) {
    bool minus = false;      //标记是否是负数
    std::string real = num;       //real表示num的绝对值
    if (num.at(0) == '-') {
        minus = true;
        real = num.substr(1, num.size() - 1);
    }

    char c;
    int i = 0;
    double result = 0.0, dec = 10.0;
    bool isDec = false;       //标记是否有小数
    unsigned long size = real.size();
    while (i < size) {
        c = real.at(i);
        if (c == '.') {//包含小数
            isDec = true;
            i++;
            continue;
        }
        if (!isDec) {
            result = result * 10 + c - '0';
        } else {//识别小数点之后都进入这个分支
            result = result + (c - '0') / dec;
            dec *= 10;
        }
        i++;
    }

    if (minus == true) {
        result = -result;
    }

    return result;
}

std::string Double2String(double num) {
    return std::to_string(num);
}

std::vector<std::string> SplitString(const std::string &src,
                                     const std::string &separator) {

    std::vector<std::string> dst;

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



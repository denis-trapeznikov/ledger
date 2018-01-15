#ifndef STRING_TRIM_HPP
#define STRING_TRIM_HPP

// after
// http://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
#include <algorithm>
#include <cctype>
#include <functional>
#include <locale>
namespace fetch {
namespace string {
void TrimFromRight(std::string &s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(),
                       std::not1(std::ptr_fun<int, int>(std::isspace))));
}

void TrimFromLeft(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       std::not1(std::ptr_fun<int, int>(std::isspace)))
              .base(),
          s.end());
}

void Trim(std::string &s) {
  TrimFromRight(s);
  TrimFromLeft(s);
}
};
};
#endif

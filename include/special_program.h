#ifndef __SPECIAL_PROGRAM_H
#define __SPECIAL_PROGRAM_H

#include <vector>

namespace recording {
    struct RecordUnit;
}

namespace special_program {
    std::vector<recording::RecordUnit>& ghost();
    std::vector<recording::RecordUnit>& phantom();
    std::vector<recording::RecordUnit>& joker();
}

#endif

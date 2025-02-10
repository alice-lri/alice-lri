#include "accurate_ri.h"
#include <iostream>
#include "utils/Logger.h"
#include "utils/Timer.h"

void hello() {
    PROFILE_SCOPE("hello");
    LOG_DEBUG("Hello, World!");
}

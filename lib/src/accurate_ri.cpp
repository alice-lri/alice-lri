#include "accurate_ri.h"
#include <iostream>
#include "utils/logger.h"
#include "utils/timer.h"

void hello() {
    PROFILE_SCOPE("hello");
    LOG_DEBUG("Hello, World!");
}

#include "accurate_ri.h"
#include "FileUtils.h"

int main() {
    FileUtils::Points points = FileUtils::loadBinaryFile("points.bin");
    accurate_ri::execute(points.x, points.y, points.z);
}

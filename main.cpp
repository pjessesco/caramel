#include <iostream>

#include <logger.h>
#include <shape.h>

#include <Peanut/Peanut.h>

int main() {
    std::cout << "Hello, World!" << std::endl;

    Caramel::OBJMesh("/Users/jino/caramel/bunny.obj");
    Caramel::OBJMesh("/Users/jino/caramel/ajax.obj");

    WARNING("Warning example");
    LOG("Log example");
    ERROR("Error example");

    Peanut::Matrix<int, 2, 2> mat;

    return 0;
}

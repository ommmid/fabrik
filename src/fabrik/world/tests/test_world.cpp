#include <iostream>

#include "world.h"

int main(){
    std::cout << "Hello, from testing world!\n";

    worlds::world world_1(20);

    world_1.describe();

}

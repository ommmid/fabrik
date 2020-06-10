
#include <iostream>

#include "world.h"

namespace worlds
{
    

    world::world(int a)
    {
        age_ = a;
    };

    void world::describe()
    {
        std::cout << "a world with " << age_ << " years old has been created. Yaaayyyyy!" << std::endl;
    };

}

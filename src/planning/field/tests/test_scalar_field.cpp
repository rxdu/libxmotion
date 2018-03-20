#include <iostream>
#include "field/scalar_field.hpp"

using namespace librav;

int main()
{
    ScalarField field(5, 5);

    std::cout << "field created" << std::endl;
    field.SetOriginCoordinate(2, 2);

    field.SetValueAtCoordinate(2, 2, 5.6);
    field.SetValueAtCoordinate(1, 2, 2.5);
    field.SetValueAtCoordinate(0, 0, 1.6);
    field.SetValueAtCoordinate(-1, -1, 2.6);
    field.SetValueAtCoordinate(-1, -2, 1.8);

    field.PrintField(true);

    std::cout << "done" << std::endl;
    
    return 0;
}
#include <iostream>
#include <string>

int main()
{
    int x = 5;
    while(x--)
    {
        std::cout << x << "\n";
        std::cout << rand() % 50 << "\n";
    }
}

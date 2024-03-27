#include <iostream>
#include "PrimeChecker.hpp" //header file for primechecker file

int main(int argc, char** argv) {
    if (argc == 2) {
        int number = std::stoi(argv[1]); // retrieves input from console
        PrimeChecker pc;
        std::cout << "tengse, raghav; " << number << " is a prime number? " << pc.isPrime(number) << std::endl; //print message
    }
    return 0;
}


#include <iostream>
#include "PrimeChecker.hpp" // Includes prime checker header file

int main(int argc, char** argv) {
    // Checks for the correct number of arguments
    if (argc == 2) {
        int number = std::stoi(argv[1]);
        PrimeChecker pc;
        std::cout << "Padhya, Jitish; " << number << " is a prime number? " << pc.isPrime(number) << std::endl;
    }
    return 0; // returns 0, indicating the end after a successful execution of program.
}
#include <iostream>

#include "markov/markov_chain.hpp"

using namespace rnav;

int main()
{
    MarkovChain<2>::State states;
    MarkovChain<2>::Transition trans;

    states << 1.0, 0.0;
    trans << 0.9, 0.1, 0.5, 0.5;

    MarkovChain<2> mchain(states, trans);

    std::cout << "k = " << 2 << "\n"
              << mchain.CalculateStateAt(2) << std::endl;
    std::cout << "size: " << mchain.GetStateNumber() << std::endl;

    mchain.Propagate(2);

    std::cout << "k = " << 2 << "\n"
              << mchain[2] << std::endl;

    std::cout << "size: " << mchain.GetStateNumber() << std::endl;

    std::cout << "-----" << std::endl;
    for (int32_t k = 0; k < mchain.GetStateNumber(); ++k)
        std::cout << "k = " << k << "\n"
                  << mchain[k] << std::endl;

    return 0;
}
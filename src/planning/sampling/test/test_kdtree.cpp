#include <iostream>
#include <cstdint>

#include "spatial/spatial.hpp"
#include "spatial/point_multimap.hpp"
#include "spatial/point_multiset.hpp"
#include "spatial/neighbor_iterator.hpp"

#include "sampling/space/realvector_space.hpp"
#include "interface/time/stopwatch.hpp"

using namespace robosw;

int main()
{
    constexpr int32_t N = 3;

    RealVectorSpace<N> rvspace;

    // rvspace.SetBound(0, 1, 1.5);
    // rvspace.SetBound(1, 2, 3.5);
    // rvspace.SetBound(2, 3, 4.5);

    rvspace.PrintInfo();

    ///////////////////////////////////////////////////////////////////////////

    using StateType = RealVectorSpace<N>::StateType;

    std::vector<std::shared_ptr<StateType>> states;
    for (int i = 0; i < 5; ++i)
        states.push_back(rvspace.SampleUniform());

    struct state_accessor
    {
        double operator()(spatial::dimension_type dim, std::shared_ptr<StateType>state) const
        {
            return (*state)[dim];
        }
    };

    typedef spatial::point_multiset<N, std::shared_ptr<StateType>, spatial::accessor_less<state_accessor, std::shared_ptr<StateType>>> KdTreeType;

    KdTreeType kdtree;
    kdtree.insert(states.begin(), states.end());

    std::cout << "kd tree size: " << kdtree.size() << std::endl;

    auto query_state = states.front();
    std::cout << "query state: " << *query_state << std::endl;
    for (int i = 0; i < 5; ++i)
        std::cout << "distance to " << *(states[i]) << ": " << rvspace.EvaluateDistance(query_state, states[i]) << std::endl;

    std::cout << "---------------------" << std::endl;

    spatial::neighbor_iterator<KdTreeType> iter = spatial::neighbor_begin(kdtree, query_state);
    std::cout << *(*iter) << " is at " << distance(iter) << " metres\n";
    for (int i = 0; i < 5 - 1; ++i)
        std::cout << *(*++iter) << " at " << distance(iter) << " metres\n";

    std::cout << "---------------------" << std::endl;

    // note: read documentation for the actual meaning of lower/upper bound
    spatial::neighbor_iterator<KdTreeType> iterbd = spatial::neighbor_lower_bound(kdtree, query_state, 0.5);
    std::cout << *(*iterbd) << " is at " << distance(iterbd) << " metres\n";
    while (++iterbd != kdtree.end())
        std::cout << *(*iterbd) << " at " << distance(iterbd) << " metres\n";

    return 0;
}
/*
 * priority_queue.h
 *
 *  Created on: Feb 2, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_PRIORITY_QUEUE_H_
#define SRC_GRAPH_PRIORITY_QUEUE_H_

#include <vector>
#include <queue>
#include <functional>

namespace librav {

/// A simple priority queue structure used as A* open list.
// Source: http://www.redblobgames.com/pathfinding/a-star/implementation.html
template<typename T, typename Number=double>
struct PriorityQueue {
	typedef std::pair<Number, T> PQElement;

	std::priority_queue<PQElement, std::vector<PQElement>,
	std::greater<PQElement>> elements;

	inline bool empty() const { return elements.empty(); }

	inline void put(T item, Number priority) {
		elements.emplace(priority, item);
	}

	inline T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};

// TODO to be finished, not completed yet
/*template<typename Comparable>
class PriorityQueue {
public:
	PriorityQueue();
	~PriorityQueue();

private:

public:
	bool IsEmpty( ) const;
	const Comparable & FindMin( ) const;

	void Insert(const Comparable& x);
	void DeleteMin( );
	void DeleteMin( Comparable & minItem );
	void MakeEmpty( );

private:
	int	size_;  // Number of elements in heap
	std::vector<Comparable> elements_;    // The heap array

	void buildHeap( );
	void percolateDown( int hole );
};*/

}

#endif /* SRC_GRAPH_PRIORITY_QUEUE_H_ */

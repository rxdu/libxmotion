/* 
 * tree.hpp
 * 
 * Created on: Jan 07, 2019 09:19
 * Description: implementation of tree is very similar to graph
 *          except that one vertex in a tree has only one parent
 *          and only available operations on a tree are slightly
 *          different
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef TREE_HPP
#define TREE_HPP

#ifdef NOT_USE_UNORDERED_MAP
#include <map>
#else
#include <unordered_map>
#endif

#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <type_traits>

#include "graph/details/default_indexer.hpp"

namespace autodrive
{
/// Tree class template.
template <typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
class Tree
{
  public:
    class Edge;
    class Vertex;
    using TreeType = Tree<State, Transition, StateIndexer>;

#ifdef NOT_USE_UNORDERED_MAP
    typedef std::map<int64_t, Vertex *> VertexMapType;
#else
    typedef std::unordered_map<int64_t, Vertex *> VertexMapType;
#endif
    typedef typename VertexMapType::iterator VertexMapTypeIterator;

  public:
    /*---------------------------------------------------------------------------------*/
    /*                              Vertex Iterator                                    */
    /*---------------------------------------------------------------------------------*/
    ///@{
    /// Vertex iterator for unified access.
    /// Wraps the "value" part of VertexMapType::iterator
    class const_vertex_iterator : public VertexMapTypeIterator
    {
      public:
        const_vertex_iterator() : VertexMapTypeIterator(){};
        explicit const_vertex_iterator(VertexMapTypeIterator s) : VertexMapTypeIterator(s){};

        const Vertex *operator->() const { return (Vertex *const)(VertexMapTypeIterator::operator->()->second); }
        const Vertex &operator*() const { return *(VertexMapTypeIterator::operator*().second); }
    };

    class vertex_iterator : public const_vertex_iterator
    {
      public:
        vertex_iterator() : const_vertex_iterator(){};
        explicit vertex_iterator(VertexMapTypeIterator s) : const_vertex_iterator(s){};

        Vertex *operator->() { return (Vertex *const)(VertexMapTypeIterator::operator->()->second); }
        Vertex &operator*() { return *(VertexMapTypeIterator::operator*().second); }
    };
    ///@}

    /*---------------------------------------------------------------------------------*/
    /*                               Edge Template                                     */
    /*---------------------------------------------------------------------------------*/
    ///@{
    /// Edge class template.
    struct Edge
    {
        Edge(vertex_iterator src, vertex_iterator dst, Transition trans) : src_(src), dst_(dst), trans_(trans){};
        ~Edge() = default;

        Edge(const Edge &other) = default;
        Edge &operator=(const Edge &other) = default;
        Edge(Edge &&other) = default;
        Edge &operator=(Edge &&other) = default;

        vertex_iterator src_;
        vertex_iterator dst_;
        Transition trans_;

        /// Check if current edge is identical to the other (all src_, dst_, trans_).
        bool operator==(const Edge &other);

        /// Print edge information, assuming member "trans_" is printable.
        void PrintEdge();
    };
    ///@}

    /*---------------------------------------------------------------------------------*/
    /*                              Vertex Template                                    */
    /*---------------------------------------------------------------------------------*/
    ///@{
    /// Vertex class template.
    struct Vertex
    {
        /** @name Big Five
     *  Edge iterators to access vertices in the tree.
     */
        ///@{
        Vertex(State s, int64_t id) : state_(s), vertex_id_(id) {}
        ~Vertex() = default;

        // do not allow copy or assign
        Vertex() = delete;
        Vertex(const State &other) = delete;
        Vertex &operator=(const State &other) = delete;
        Vertex(State &&other) = delete;
        Vertex &operator=(State &&other) = delete;
        ///@}

        // generic attributes
        State state_;
        const int64_t vertex_id_;
        StateIndexer GetStateIndex;

        // edges connecting to other vertices
        typedef std::vector<Edge> EdgeListType;
        EdgeListType edges_to_;

        // parent vertex that contain edges connecting to current vertex
        vertex_iterator parent_vertex_;

        // attributes for search algorithms
        bool is_checked_ = false;
        bool is_in_openlist_ = false;
        double f_cost_ = std::numeric_limits<double>::max();
        double g_cost_ = std::numeric_limits<double>::max();
        double h_cost_ = std::numeric_limits<double>::max();
        vertex_iterator search_parent_;

        /** @name Edge access.
     *  Edge iterators to access vertices in the tree.
     */
        ///@{
        // edge iterator for easy access
        typedef typename EdgeListType::iterator edge_iterator;
        typedef typename EdgeListType::const_iterator const_edge_iterator;
        edge_iterator edge_begin() { return edges_to_.begin(); }
        edge_iterator edge_end() { return edges_to_.end(); }
        const_edge_iterator edge_begin() const { return edges_to_.cbegin(); }
        const_edge_iterator edge_end() const { return edges_to_.cend(); }
        ///@}

        /** @name Edge Operations
     *  Modify or query edge information of the vertex.
     */
        ///@{
        /// Returns true if two vertices have the same id. Otherwise, return false.
        bool operator==(const Vertex &other);

        /// Returns the id of current vertex.
        int64_t GetVertexID() const { return vertex_id_; }

        /// Look for the edge connecting to the vertex with give id.
        edge_iterator FindEdge(int64_t dst_id);

        /// Look for the edge connecting to the vertex with give state.
        template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
        edge_iterator FindEdge(T dst_state);

        /// Check if the vertex with given id or state is a neighbour of current vertex.
        template <typename T>
        bool CheckNeighbour(T dst);

        /// Get all neighbor vertices of this vertex.
        std::vector<vertex_iterator> GetNeighbours();

        /// Clear exiting search info before a new search
        void ClearVertexSearchInfo();
    };
    ///@}

    /*---------------------------------------------------------------------------------*/
    /*                               Tree Template                                    */
    /*---------------------------------------------------------------------------------*/
  public:
    /** @name Big Five
   *  Constructor, copy/move constructor, copy/move assignment operator, destructor. 
   */
    ///@{
    /// Default Tree constructor.
    Tree() = default;
    /// Copy constructor.
    Tree(const TreeType &other);
    /// Move constructor
    Tree(TreeType &&other);
    /// Assignment operator
    TreeType &operator=(const TreeType &other);
    /// Move assignment operator
    TreeType &operator=(TreeType &&other);

    /// Default Tree destructor.
    /// Tree class is only responsible for the memory recycling of its internal objects, such as
    /// vertices and edges. If a state is associated with a vertex by its pointer, the memory allocated
    //  for the state object will not be managed by the tree and needs to be recycled separately.
    ~Tree();
    ///@}

    /** @name Vertex Access
   *  Vertex iterators to access vertices in the tree.
   */
    ///@{
    vertex_iterator vertex_begin() { return vertex_iterator{vertex_map_.begin()}; }
    vertex_iterator vertex_end() { return vertex_iterator{vertex_map_.end()}; }
    const_vertex_iterator vertex_begin() const { return const_vertex_iterator{vertex_map_.begin()}; }
    const_vertex_iterator vertex_end() const { return const_vertex_iterator{vertex_map_.end()}; }
    ///@}

    /** @name Edge Access
   *  Edge iterators to access edges in the vertex.
   */
    ///@{
    typedef typename Vertex::edge_iterator edge_iterator;
    typedef typename Vertex::const_edge_iterator const_edge_iterator;
    ///@}

    /** @name Tree Operations
   *  Modify vertex or edge of the tree. 
   */
    ///@{
    /// This function is used to create a vertex in the tree that associates with the given node.
    vertex_iterator AddVertex(State state);

    /// This function checks if a vertex exists in the tree and remove it if presents.
    void RemoveVertex(int64_t state_id);

    template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
    void RemoveVertex(T state) { RemoveVertex(GetStateIndex(state)); }

    /// This function returns the root vertex of the tree
    vertex_iterator GetRootVertex() const { return root_; }

    // / This function returns the parent vertex of the specified node
    vertex_iterator GetParentVertex(int64_t state_id);

    template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
    vertex_iterator GetParentVertex(State state) { return GetParentVertex(TreeType::GetStateIndex(state)); }

    /// This function checks depth of the specified node in the three, assuming each node only has one parent
    int32_t GetVertexDepth(int64_t state_id);

    template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
    int32_t GetVertexDepth(T state) { return GetVertexDepth(TreeType::GetStateIndex(state)); }

    /// This function is used to add an edge between the vertices associated with the given two states.
    /// Update the transition if edge already exists.
    void AddEdge(State sstate, State dstate, Transition trans);

    /// This function is used to remove the directed edge from src_node to dst_node.
    bool RemoveEdge(State sstate, State dstate);

    /// This function removes a subtree including the specified vertex
    void RemoveSubtree(int64_t state_id);

    template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
    void RemoveSubtree(T state) { RemoveSubtree(TreeType::GetStateIndex(state)); }

    /// This functions is used to access all edges of a tree
    std::vector<edge_iterator> GetAllEdges() const;

    /// This function return the vertex iterator with specified id
    inline vertex_iterator FindVertex(int64_t vertex_id) { return vertex_iterator{vertex_map_.find(vertex_id)}; }

    /// This function return the vertex iterator with specified state
    template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
    inline vertex_iterator FindVertex(T state) { return vertex_iterator{vertex_map_.find(GetStateIndex(state))}; }

    /// Get total number of vertices in the tree
    int64_t GetTotalVertexNumber() const { return vertex_map_.size(); }

    /// Get total number of edges in the tree
    int64_t GetTotalEdgeNumber() const { return GetAllEdges().size(); }

    /* Utility functions */
    /// This function is used to reset states of all vertice for a new search
    void ResetAllVertices();

    /// This function removes all edges and vertices in the tree
    void ClearAll();
    ///@}

  protected:
    /** @name Internal variables and functions.
     *  Internal variables and functions.
     */
    vertex_iterator root_{vertex_iterator(TreeType::vertex_map_.end())};

    ///@{
    /// This function returns an index of the give state.
    /// The default indexer returns member variable "id_", assuming it exists.
    StateIndexer GetStateIndex;
    VertexMapType vertex_map_;

    /// Returns the iterator to the pair whose value is "state" in the vertex map.
    /// Create a new pair if one does not exit yet and return the iterator to the
    /// newly created pair.
    vertex_iterator ObtainVertexFromVertexMap(State state);
    ///@}
};

template <typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
using Tree_t = Tree<State, Transition, StateIndexer>;
} // namespace autodrive

#include "graph/details/tree/tree_edge_impl.hpp"
#include "graph/details/tree/tree_vertex_impl.hpp"
#include "graph/details/tree/tree_impl.hpp"

#endif /* TREE_HPP */

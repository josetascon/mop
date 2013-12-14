/**
 * @file GraphBFS.hpp
 * @brief This file has all functions related with graph
 *
 * @author José David Tascón Vidarte
 * @date Dec/09/2013
 */

#ifndef __GRAPHBFS_HPP__
#define __GRAPHBFS_HPP__

// Std Libraries
#include <iostream>
#include <string>
#include <vector>
#include <utility>		//std::pair

// Boost Libraries
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>


template < typename tt > class bfs_parent_visitor: public boost::default_bfs_visitor
{
public:
    bfs_parent_visitor( std::vector< tt > *path, std::vector< tt > *dtime, std::vector< bool > *visited )
    : path(path), dtime(dtime), visited(visited) { }
    
    template < typename Vertex, typename Graph >
    void discover_vertex(Vertex u, const Graph & g) const
    {
        dtime->push_back( tt(u) );
    }
    
    template < typename Edge, typename Graph >
    void examine_edge( Edge e, const Graph & g) const
    {
        if( !(*visited)[target(e, g)] )
        {
	  (*path)[target(e, g)] = tt( source(e, g) );
	  (*visited)[target(e, g)] = true;
        }
    }
    
    std::vector< tt > *path;
    std::vector< tt > *dtime;
    std::vector< bool > *visited;
};


// ================================================================================================
// ========================================= CLASS GraphBFS ======================================
// ================================================================================================

class GraphBFS
{
private:
    int nvertices;
    int nedges;
    int init_bfs;
    std::vector< float > weights;
    std::vector< std::pair <int,int> > edges_pairs;
    
    typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS,
    boost::no_property, boost::property<boost::edge_weight_t, float> > Graph;
    
    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    
public:
    GraphBFS(int &verts, int &edge, std::vector< std::pair <int,int> > &edge_vec)
    : nvertices(verts), nedges(edge), edges_pairs(edge_vec) 
    {
        init_bfs = -1;
        weights.resize( edge, 1.0 );
    };
    
    GraphBFS(int &verts, int &edge, std::vector< std::pair <int,int> > &edge_vec, std::vector< float > &ww )
    : nvertices(verts), nedges(edge), edges_pairs(edge_vec), weights(ww) { init_bfs = -1; };
    
    void setInitBFS( int &init ) { init_bfs = init; };
    void getInitBFS( int &init ) { init = init_bfs; };
    
    void solveBFS( std::vector< int > &discover, std::vector< int > &parent )
    {
        // Create undirect graph
        Graph g(edges_pairs.begin(), edges_pairs.end(), weights.begin(), nvertices);
        
        std::vector< int > path( num_vertices(g) );
        std::vector< int > dtime; 		//empty vector
        std::vector< bool > visited(num_vertices(g), false);
        
        // Init of BFS
        /// TODO select origin as vertex with major connectivity        
        if ( init_bfs == -1 ) 
        {
	  init_bfs = 0;
	  path[0] = 0;
        }
        else path[init_bfs] = init_bfs;
        visited[init_bfs] = true;
        
        bfs_parent_visitor< int > vis( &path, &dtime, &visited);		// created visitor to save path and discover order
        boost::breadth_first_search(g, vertex(init_bfs, g), visitor(vis));	// Solve breath first search
        
        // Debug
        std::cout << "Discovered order:\n";
        for (int k = 0; k < path.size(); ++k)
        {
	  std::cout << dtime[k] << ":\t";
	  std::cout << "parent: " << path[ dtime[k] ] << "\n";
        }
        
        parent = path;
        discover = dtime;
    }
};

#endif
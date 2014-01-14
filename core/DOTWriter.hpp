/**
 * @file DOTWriter.hpp
 * @brief This file has all functions related with dot export
 *
 * @author José David Tascón Vidarte
 * @date Dec/09/2013
 */

#ifndef __DOTWRITER_HPP__
#define __DOTWRITER_HPP__

// Std Libraries
#include <iostream>
#include <string>
#include <vector>
#include <utility>		//std::pair

// ================================================================================================
// ========================================= CLASS DOTWriter ======================================
// ================================================================================================
class DOTWriter
{
private:
    int width;
    int height;
    std::string graph_type;
    std::string ratio;
    std::string edge_style;
    std::string node_shape;
    std::string rankdir;
    
public:
    DOTWriter()
    {
        graph_type = "graph"; //undirect graph
        width = 5; height = 3; ratio = "fill"; edge_style = "bold"; node_shape = "circle"; rankdir = "LR";
    };
    DOTWriter(std::string grapht): graph_type(grapht)
    {
        width = 5; height = 3; ratio = "fill"; edge_style = "bold"; node_shape = "circle"; rankdir = "LR";
    };
    
    void setSize(int &w, int &h) { width = w; height = h; };
    void setGraph(std::string &g) { graph_type = g; };
    void setRatio(std::string &r) { ratio = r; };
    void setEdgeStyle(std::string &es) { edge_style = es; };
    void setNodeShape(std::string &ns) { node_shape = ns; };
    void setRankdir(std::string &rd) { rankdir = rd; };
    
    void getSize(int &w, int &h) { w = width; h = height; };
    void getGraph(std::string &g) { g = graph_type; };
    void getRatio(std::string &r) { r = ratio; };
    void getEdgeStyle(std::string &es) { es = edge_style; };
    void getNodeShape(std::string &ns) { ns = node_shape; };
    void getRankdir(std::string &rd) { rd = rankdir; };
    
    template <typename Tedge>
    void exportDOT(const char* filename, std::vector< std::pair <Tedge,Tedge> > *edges_pairs )
    {
        std::vector< std::string > *names = NULL;
        std::vector< float > *weights = NULL;
        exportDOTGeneral(filename, edges_pairs, names, weights);
    }
    
    template <typename Tedge, typename Tw >
    void exportDOT(const char* filename, std::vector< std::pair <Tedge,Tedge> > *edges_pairs, std::vector< Tw > *weights  )
    {
        exportDOTGeneral(filename, edges_pairs, NULL, weights);
    }
    
    template <typename Tedge, typename Tw >
    void exportDOT(const char* filename, std::vector< std::pair <Tedge,Tedge> > *edges_pairs, std::vector< std::string > *names, 
		std::vector< Tw > *weights )
    {
        exportDOTGeneral(filename, edges_pairs, names, weights);
    }
    
    template <typename Tedge, typename Tw >
    void exportDOTGeneral(const char* filename, std::vector< std::pair <Tedge,Tedge> > *edges_pairs, std::vector< std::string > *names, 
		std::vector< Tw > *weights )
    {
        std::ofstream dot_file(filename);
        dot_file << graph_type << " D {\n"
	  << "  rankdir=" << rankdir <<"\n"
	  << "  size=\"" << width << "," << height << "\"\n"
	  << "  ratio=\"" << ratio << "\"\n"
	  << "  edge[style=\"" << edge_style << "\"]\n" 
	  << "  node[shape=\"" << node_shape << "\"]\n";
	  
	  for( int ed = 0; ed < edges_pairs->size(); ++ed)
	  {
	      if (names != NULL)
	      {
		dot_file << (*names)[ (*edges_pairs)[ed].first  ];
		std::string str1 = ( graph_type.compare("graph") )? " -> ": " -- " ;
		dot_file << str1;
		dot_file << (*names)[ (*edges_pairs)[ed].second ];
	      }
	      else
	      {
		dot_file << (*edges_pairs)[ed].first;
		std::string str1 = ( graph_type.compare("graph") )? " -> ": " -- " ;
		dot_file << str1;
		dot_file << (*edges_pairs)[ed].second;
	      }
	      
	      if (weights != NULL)
	      {
		dot_file << "[label=\"";
		dot_file << (*weights)[ed];
		dot_file << "\"]";
	      }
	      
	      dot_file << " ";
	  
	      //         << "[label=\"" << get(get(&EdgeProperties::weights, g), e) << "\"";
	      //       if (parent[v] == u)
	      //         dot_file << ", color=\"black\"";
	      //       else
	      //         dot_file << ", color=\"grey\"";
	      //       dot_file << "]";
	  }
	  
	  dot_file << "\n}";
	  dot_file.close();
    }   
};

#endif
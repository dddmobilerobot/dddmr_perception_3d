/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//@For graph
#include <perception_3d/static_graph.h>

namespace perception_3d
{

StaticGraph::StaticGraph(){
  graph_ptr_ = new graph_t();
}

StaticGraph::~StaticGraph(){
  if(graph_ptr_)
    delete graph_ptr_;
}

void StaticGraph::insertNode(unsigned int node, edge_t& a_edge){
  (*graph_ptr_)[node].insert(a_edge);
}

void StaticGraph::insertWeight(unsigned int node, float weight){
  node_weight_[node] = weight;
}

graph_t* StaticGraph::getGraphPtr(){
  return graph_ptr_;
}

edges_t StaticGraph::getEdge(unsigned int node){
  return (*graph_ptr_)[node];
}

float StaticGraph::getNodeWeight(unsigned int node){
  return node_weight_[node];
}

void StaticGraph::clear(){
  node_weight_.clear();
  (*graph_ptr_).clear();
}

}
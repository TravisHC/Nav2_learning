// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include "nav2_smac_planner/node_basic.hpp"

namespace nav2_smac_planner
{
//  2d无需操作，为空
template<typename Node2D>
void NodeBasic<Node2D>::processSearchNode()
{
}

//  hybrid中确保了每个节点只处理一次pose，并且记录了motion_index和turn_dir
template<>
void NodeBasic<NodeHybrid>::processSearchNode()
{
  // We only want to override the node's pose if it has not yet been visited
  // to prevent the case that a node has been queued multiple times and
  // a new branch is overriding one of lower cost already visited.
  if (!this->graph_node_ptr->wasVisited()) {
    this->graph_node_ptr->pose = this->pose;
    this->graph_node_ptr->setMotionPrimitiveIndex(this->motion_index, this->turn_dir);
  }
}

//  同样地，lattice中也确保了每个节点只处理一次pose防止新的轨迹分支覆盖了已经访问过的且具有较低成本的节点
//  lattice中还记录了motion_primitive和backward
//  这里的prim_ptr是一个结构体的指针，包含了id，始末角度，转弯半径，长度等等信息，backward指是否是反向的轨迹
template<>
void NodeBasic<NodeLattice>::processSearchNode()
{
  // We only want to override the node's pose/primitive if it has not yet been visited
  // to prevent the case that a node has been queued multiple times and
  // a new branch is overriding one of lower cost already visited.
  if (!this->graph_node_ptr->wasVisited()) {
    this->graph_node_ptr->pose = this->pose;
    this->graph_node_ptr->setMotionPrimitive(this->prim_ptr);
    this->graph_node_ptr->backwards(this->backward);
  }
}

template<>
void NodeBasic<Node2D>::populateSearchNode(Node2D * & node)
{
  this->graph_node_ptr = node;
}

template<>
void NodeBasic<NodeHybrid>::populateSearchNode(NodeHybrid * & node)
{
  this->pose = node->pose;
  this->graph_node_ptr = node;
  this->motion_index = node->getMotionPrimitiveIndex();
  this->turn_dir = node->getTurnDirection();
}

template<>
void NodeBasic<NodeLattice>::populateSearchNode(NodeLattice * & node)
{
  this->pose = node->pose;
  this->graph_node_ptr = node;
  this->prim_ptr = node->getMotionPrimitive();
  this->backward = node->isBackward();
}

template class NodeBasic<Node2D>;
template class NodeBasic<NodeHybrid>;
template class NodeBasic<NodeLattice>;

}  // namespace nav2_smac_planner

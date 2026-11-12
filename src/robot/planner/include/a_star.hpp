#pragma once

#include <cstddef>
#include <functional>
#include <queue>

// 2D grid index
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy)
      : x(xx)
      , y(yy) {}
  CellIndex()
      : x(0)
      , y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode {
  CellIndex index;
  double
      g_score; // cost of the path ALREADY TRAVELLED to this node (known, exact)
  double h_score; // estimated euclidean distance to goal (guessed, heuristic)
  double f_score; // f = g + h
  std::queue<CellIndex> came_from;

  AStarNode(CellIndex idx, double f)
      : index(idx)
      , f_score(f) {}
  AStarNode() = default;
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
  // this comparator is asking: Should node a be pushed BELOW node b
  bool operator()(const AStarNode &a, const AStarNode &b) {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};
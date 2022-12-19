/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC
   license. For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <boost/compute/detail/lru_cache.hpp>
#include <boost/functional/hash.hpp>
#include <cmath>
#include <functional>
#include <set>
#include <vector>

namespace AStar {
struct Vec2i {
  int x, y;

  bool operator==(const Vec2i& coordinates_);
};

typedef std::pair<std::pair<int, int>, std::pair<int, int>> PointPair;

// struct PointPair {
//   Vec2i source_, target_;
//   bool operator==(const PointPair& rhs) {
//     return source_ == rhs.source_ && target_ == rhs.target_;
//   }

//   PointPair(Vec2i source, Vec2i target) : source_(source), target_(target) {}

//   friend std::size_t hash_value(PointPair const& p) {
//     std::size_t seed = 0;
//     boost::hash_combine(seed, p.source_.x);
//     boost::hash_combine(seed, p.source_.y);
//     boost::hash_combine(seed, p.target_.x);
//     boost::hash_combine(seed, p.target_.y);
//     return seed;
//   }

// };

using uint = unsigned int;
using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
using CoordinateList = std::vector<Vec2i>;

struct Node {
  uint G, H;
  Vec2i coordinates;
  Node* parent;

  Node(Vec2i coord_, Node* parent_ = nullptr);
  uint getScore();
};

using NodeSet = std::set<Node*>;

class Generator {
  Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
  void releaseNodes(NodeSet& nodes_);

 public:
  Generator();
  bool detectCollision(Vec2i coordinates_);
  void setWorldSize(Vec2i worldSize_);
  void setDiagonalMovement(bool enable_);
  void setHeuristic(HeuristicFunction heuristic_);
  void printMap(Vec2i source_ = {-1, -1}, Vec2i target_ = {-1, -1});
  CoordinateList findPath(Vec2i source_, Vec2i target_);
  CoordinateList findPathCached(Vec2i source_, Vec2i target_,
                                bool omit_start_end);
  void addCollision(Vec2i coordinates_);
  void removeCollision(Vec2i coordinates_);
  void clearCollisions();
  void setCache(Vec2i source_, Vec2i target_, CoordinateList path_);
  CoordinateList getCache(Vec2i source_, Vec2i target_);
  void clearCache();

 private:
  CoordinateList findPath_(Vec2i source_, Vec2i target_);
  HeuristicFunction heuristic;
  CoordinateList direction, walls;
  Vec2i worldSize;
  uint directions;
  bool cache_lock_ = false;
  boost::compute::detail::lru_cache<PointPair, CoordinateList> path_cache =
      boost::compute::detail::lru_cache<PointPair, CoordinateList>(300);
};

class Heuristic {
  static Vec2i getDelta(Vec2i source_, Vec2i target_);

 public:
  static uint manhattan(Vec2i source_, Vec2i target_);
  static uint euclidean(Vec2i source_, Vec2i target_);
  static uint octagonal(Vec2i source_, Vec2i target_);
};
}  // namespace AStar

#endif  // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
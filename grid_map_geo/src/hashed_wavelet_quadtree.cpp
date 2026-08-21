#include "grid_map_geo/hashed_wavelet_quadtree.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <unordered_map>
#include <utility>

#include <grid_map_core/iterators/GridMapIterator.hpp>

// Self-contained 2D Haar-wavelet hashed-block-sparse quadtree: no external
// tree library. Each hashed block is a fixed-height (`tree_height`) quadtree
// whose nodes live in one pooled `std::vector<Node>` per block, addressed by
// `int32_t` index rather than pointer -- this avoids the per-node
// malloc/free overhead of a naive pointer tree (measured at ~2.7x a dense
// float array's memory on real terrain in an earlier prototype of this
// class), while still collapsing whole subtrees of locally-uniform terrain
// to nothing via prune().
//
// IMPORTANT invariant for anyone touching Block's node-pool code: never hold
// a Node&/Node* across a `nodes_.push_back()` (or emplace_back) -- vector
// reallocation invalidates it. Every access re-indexes `nodes_[i]` fresh.

namespace {

// Child order: 0=(x=0,y=0) 1=(x=1,y=0) 2=(x=0,y=1) 3=(x=1,y=1).
struct Node {
  std::array<float, 3> detail{0.f, 0.f, 0.f};    // {horizontal, vertical, diagonal}
  std::array<int32_t, 4> child{-1, -1, -1, -1};  // index into the block's node pool, -1 = none
};

// 2D Haar transform, used only in its single-child forms below (a full
// 4-children<->{scale,detail} round trip is never needed: queries only ever
// walk one child at a time root->leaf, and updates inject a delta at one
// leaf and propagate it leaf->root). The transform is exactly invertible;
// if all 4 children of a node are equal, all 3 detail coefficients are
// exactly zero in IEEE-754 (equal-value subtraction, then division by a
// power of two -- neither step rounds), which is the actual mechanism that
// lets prune() collapse whole subtrees of locally-uniform terrain.
//
// Backward: reconstruct just one child's value from its parent's scale +
// detail.
float haarBackwardSingleChild(float scale, const std::array<float, 3>& detail, int child_index) {
  const float sx = (child_index & 1) ? -1.0f : 1.0f;
  const float sy = (child_index & 2) ? -1.0f : 1.0f;
  return scale + sx * detail[0] + sy * detail[1] + sx * sy * detail[2];
}

// Forward transform of a delta applied to only one child (others held at 0),
// i.e. the {scale, detail} contribution that `delta` at `child_index` alone
// injects into its parent -- by linearity, `+=`-ing this into an existing
// node's data (and propagating `scale_delta` up as the next level's input)
// correctly reflects "this one leaf changed by delta", leaving all siblings
// under every ancestor exactly unchanged.
void haarForwardSingleChild(float delta, int child_index, float& scale_delta, std::array<float, 3>& detail_delta) {
  const float sx = (child_index & 1) ? -1.0f : 1.0f;
  const float sy = (child_index & 2) ? -1.0f : 1.0f;
  scale_delta = delta * 0.25f;
  detail_delta[0] = sx * delta * 0.25f;
  detail_delta[1] = sy * delta * 0.25f;
  detail_delta[2] = sx * sy * delta * 0.25f;
}

// Detail coefficients below this magnitude are treated as exactly zero by
// threshold(), which is what lets prune() collapse a subtree that is
// uniform up to floating-point noise, not just bit-exactly uniform.
constexpr float kDetailEpsilon = 1e-3f;

// One multi-resolution leaf cell within a single block, in LOCAL (cell-
// index) units -- unit conversion to world-frame meters happens one level
// up, in Impl::getCells(), consistent with Block otherwise knowing nothing
// about world coordinates.
struct LocalCell {
  int local_x0, local_y0;  // south-west corner, in leaf-cell units
  int span;                // side length, in leaf-cell units (a power of two)
  float value;
};

// One hashed block: a fixed-height (tree_height) 2D Haar wavelet quadtree,
// backed by a pooled, index-addressed node array.
class Block {
 public:
  explicit Block(int tree_height) : tree_height_(tree_height) {}

  float getCellValue(int ix, int iy, int height) const {
    const int depth = tree_height_ - height;
    float scale = root_scale_;
    int32_t node_index = root_index_;
    for (int level = 1; level <= depth && node_index >= 0; ++level) {
      const int bit = tree_height_ - level;
      const int child_index = ((ix >> bit) & 1) | (((iy >> bit) & 1) << 1);
      const Node& node = nodes_[node_index];
      scale = haarBackwardSingleChild(scale, node.detail, child_index);
      node_index = node.child[child_index];
    }
    return scale;
  }

  // Adds `delta` to the finest-resolution (leaf) cell (ix, iy), allocating
  // ancestor nodes as needed. O(tree_height).
  void addToCellValue(int ix, int iy, float delta) {
    // Pass 1: walk root -> leaf, allocating any missing ancestor nodes.
    // Every lookup below re-indexes nodes_[...] fresh (never caches a
    // reference across an allocation) -- see the file-level comment.
    // Fixed-size (not std::vector) to keep this hot path allocation-free;
    // 64 is enormous headroom over any realistic tree_height (a handful of
    // levels -- 2^64 leaf cells per block side is not a real map).
    std::array<int32_t, 64> path{};
    int32_t parent_index = -1;       // -1 sentinel: "parent is the block root"
    int parent_child_index = -1;
    for (int level = 1; level <= tree_height_; ++level) {
      int32_t node_index = (parent_index < 0) ? root_index_ : nodes_[parent_index].child[parent_child_index];
      if (node_index < 0) {
        node_index = static_cast<int32_t>(nodes_.size());
        nodes_.emplace_back();
        if (parent_index < 0) {
          root_index_ = node_index;
        } else {
          nodes_[parent_index].child[parent_child_index] = node_index;
        }
      }
      path[level - 1] = node_index;
      const int bit = tree_height_ - level;
      parent_child_index = ((ix >> bit) & 1) | (((iy >> bit) & 1) << 1);
      parent_index = node_index;
    }

    // Pass 2: propagate `delta` bottom-up, leaf to root. No allocation
    // happens here, so caching nodes_[path[..]] per iteration is safe.
    float running_delta = delta;
    for (int level = tree_height_; level >= 1; --level) {
      const int bit = tree_height_ - level;
      const int child_index = ((ix >> bit) & 1) | (((iy >> bit) & 1) << 1);
      float scale_delta;
      std::array<float, 3> detail_delta;
      haarForwardSingleChild(running_delta, child_index, scale_delta, detail_delta);
      Node& node = nodes_[path[level - 1]];
      node.detail[0] += detail_delta[0];
      node.detail[1] += detail_delta[1];
      node.detail[2] += detail_delta[2];
      running_delta = scale_delta;
    }
    root_scale_ += running_delta;
    needs_threshold_ = true;
    needs_prune_ = true;
  }

  // Decomposes this block into its actual multi-resolution leaf cells (one
  // per already-collapsed subtree, at whatever level it stopped at -- see
  // walkCells). Cost is proportional to the number of allocated nodes, not
  // to tree_height or leaf count.
  void collectCells(std::vector<LocalCell>& out) const { walkCells(root_index_, root_scale_, 0, 0, 0, out); }

  // Computes a bounded-merge plan for this block: entries for every
  // subtree that can be safely flattened to one value within max_error of
  // every ORIGINAL leaf underneath it (checked directly against true
  // min/max, not a coefficient threshold -- see boundedMergeNode). Emits
  // nothing for regions that don't need to change (already-atomic leaves,
  // or subtrees that don't qualify at any level). The caller is
  // responsible for actually applying `plan` (this method only computes
  // it; Block has no notion of world coordinates to rewrite cells with).
  void computeBoundedMergePlan(float max_error, std::vector<LocalCell>& plan) const {
    if (root_index_ < 0) return;  // already a single flat value; nothing to coarsen
    const auto [min_v, max_v] = boundedMergeNode(root_index_, root_scale_, 0, 0, 0, max_error, plan);
    if (max_v - min_v <= 2.0f * max_error) {
      // The WHOLE block qualifies -- there's no ancestor above the block
      // root to defer this commitment to (boundedMergeNode only commits
      // on behalf of a node's *children*), so commit it here.
      plan.push_back(LocalCell{0, 0, 1 << tree_height_, (min_v + max_v) * 0.5f});
    }
  }

  void threshold() {
    if (!needs_threshold_) return;
    for (Node& node : nodes_) {
      for (float& d : node.detail) {
        if (std::abs(d) < kDetailEpsilon) d = 0.0f;
      }
    }
    needs_threshold_ = false;
  }

  void prune() {
    if (!needs_prune_) return;
    threshold();
    // Deliberately no `.reserve(nodes_.size())` here: that would reserve
    // capacity for the OLD (pre-compaction, possibly fully-dense) node
    // count, and std::vector never shrinks capacity back down on its own --
    // for a block that collapses from e.g. 1365 nodes down to 0, that
    // leaves ~38KB of permanently-unshrinkable dead capacity behind. Let
    // `compacted` grow organically to its true (small, post-compaction)
    // size instead, then shrink_to_fit() to drop any geometric-growth
    // overshoot before it becomes nodes_'s new (permanent) buffer.
    std::vector<Node> compacted;
    root_index_ = compactSubtree(root_index_, compacted);
    compacted.shrink_to_fit();
    nodes_.swap(compacted);
    needs_prune_ = false;
  }

  // Reports capacity(), not size(): size() alone would hide any future
  // regression where a vector's actual (resident) buffer ends up larger
  // than its logical contents -- see the capacity-shrinking note in
  // prune() above, which is exactly the bug this would otherwise mask.
  size_t getMemoryUsage() const { return sizeof(Block) + nodes_.capacity() * sizeof(Node); }
  bool hasAnyNode() const { return root_index_ >= 0; }

  float rootScale() const { return root_scale_; }
  void setRootScale(float v) { root_scale_ = v; }
  int32_t rootIndex() const { return root_index_; }
  void setRootIndex(int32_t idx) { root_index_ = idx; }
  const std::vector<Node>& nodes() const { return nodes_; }
  std::vector<Node>& nodesMutable() { return nodes_; }

 private:
  int tree_height_;
  float root_scale_ = 0.f;
  int32_t root_index_ = -1;
  std::vector<Node> nodes_;
  bool needs_threshold_ = false;
  bool needs_prune_ = false;

  // Post-order compaction of the subtree rooted at `old_index` (an index
  // into the OLD `nodes_`, untouched until prune() swaps it out at the end)
  // into `out`. Returns the subtree's new index in `out`, or -1 if it
  // collapsed entirely (no children and exactly-zero detail).
  int32_t compactSubtree(int32_t old_index, std::vector<Node>& out) const {
    if (old_index < 0) return -1;
    Node node = nodes_[old_index];
    bool has_child = false;
    for (int i = 0; i < 4; ++i) {
      node.child[i] = compactSubtree(node.child[i], out);
      if (node.child[i] >= 0) has_child = true;
    }
    const bool has_nonzero_detail = node.detail[0] != 0.f || node.detail[1] != 0.f || node.detail[2] != 0.f;
    if (!has_child && !has_nonzero_detail) {
      return -1;
    }
    out.push_back(node);
    return static_cast<int32_t>(out.size()) - 1;
  }

  // Breadth-first walk emitting one LocalCell per already-collapsed
  // subtree. `node_index` is "the node that would split the square
  // [local_x0, local_x0+span) x [local_y0, local_y0+span) further" (span =
  // 2^(tree_height_-level)); if it's absent (-1), that square is uniform at
  // `value` and gets emitted whole. No explicit "at max depth" check is
  // needed: addToCellValue() never populates a node's child[] past
  // level == tree_height_ (see its allocation loop), so node_index < 0
  // already fires exactly at true leaves too -- same check, same code path.
  // Reuses haarBackwardSingleChild (already exact/verified for point
  // queries) applied to all 4 children instead of just the one on a
  // point-query's path.
  void walkCells(int32_t node_index, float value, int level, int local_x0, int local_y0,
                 std::vector<LocalCell>& out) const {
    if (node_index < 0) {
      out.push_back(LocalCell{local_x0, local_y0, 1 << (tree_height_ - level), value});
      return;
    }
    const Node& node = nodes_[node_index];
    const int childspan = 1 << (tree_height_ - level - 1);
    for (int i = 0; i < 4; ++i) {
      const float child_value = haarBackwardSingleChild(value, node.detail, i);
      const int cx = local_x0 + (i & 1) * childspan;
      const int cy = local_y0 + ((i >> 1) & 1) * childspan;
      walkCells(node.child[i], child_value, level + 1, cx, cy, out);
    }
  }

  // Returns the TRUE (min, max) of every ORIGINAL leaf value underneath
  // node_index (or just {value, value} if node_index < 0, i.e. this is
  // already a single atomic value). This is exact regardless of tree
  // depth -- min/max of a union is just the min/max of the parts' min/max
  // -- so, unlike thresholding a wavelet detail coefficient, there is no
  // compounding-error risk: a caller checking (max - min) against a
  // tolerance is checking a real fact about the underlying data, not an
  // approximation that can drift across levels.
  //
  // As a SIDE EFFECT, this commits a plan entry (into `plan`) for any
  // CHILD of this node whose own subtree qualifies (its own true range
  // fits within max_error) but THIS node's own combined range does not --
  // i.e. it finds, for each branch, the LARGEST safe merge along that
  // path, and defers committing a node that DOES qualify to its own
  // parent (which might be able to extend the merge even further), only
  // finalizing it at the point where extending no longer works. The
  // block-level caller (computeBoundedMergePlan) handles the one case
  // this can't: the whole block itself qualifying, since there's no
  // ancestor above the block root to defer to.
  std::pair<float, float> boundedMergeNode(int32_t node_index, float value, int level, int local_x0, int local_y0,
                                           float max_error, std::vector<LocalCell>& plan) const {
    if (node_index < 0) return {value, value};
    const Node& node = nodes_[node_index];
    const int childspan = 1 << (tree_height_ - level - 1);
    std::array<std::pair<float, float>, 4> child_ranges;
    std::array<int, 4> child_x{}, child_y{};
    for (int i = 0; i < 4; ++i) {
      const float child_value = haarBackwardSingleChild(value, node.detail, i);
      child_x[i] = local_x0 + (i & 1) * childspan;
      child_y[i] = local_y0 + ((i >> 1) & 1) * childspan;
      child_ranges[i] = boundedMergeNode(node.child[i], child_value, level + 1, child_x[i], child_y[i], max_error, plan);
    }
    float combined_min = child_ranges[0].first;
    float combined_max = child_ranges[0].second;
    for (int i = 1; i < 4; ++i) {
      combined_min = std::min(combined_min, child_ranges[i].first);
      combined_max = std::max(combined_max, child_ranges[i].second);
    }
    if (combined_max - combined_min <= 2.0f * max_error) {
      return {combined_min, combined_max};  // qualifies as a whole; defer to our own parent
    }
    // Doesn't qualify as a whole: commit whichever of our children do.
    // (A child that doesn't qualify either already committed whatever DID
    // qualify further down, inside its own recursive call above.)
    for (int i = 0; i < 4; ++i) {
      if (node.child[i] < 0) continue;  // already atomic at its current size; nothing to coarsen
      const float child_min = child_ranges[i].first;
      const float child_max = child_ranges[i].second;
      if (child_max - child_min <= 2.0f * max_error) {
        plan.push_back(LocalCell{child_x[i], child_y[i], childspan, (child_min + child_max) * 0.5f});
      }
    }
    return {combined_min, combined_max};
  }
};

void writeNode(std::ostream& os, const std::vector<Node>& nodes, int32_t index) {
  const Node& node = nodes[index];
  uint8_t mask = 0;
  for (int i = 0; i < 4; ++i) {
    if (node.child[i] >= 0) mask |= static_cast<uint8_t>(1u << i);
  }
  os.write(reinterpret_cast<const char*>(&mask), sizeof(mask));
  os.write(reinterpret_cast<const char*>(node.detail.data()), sizeof(float) * 3);
  for (int i = 0; i < 4; ++i) {
    if (node.child[i] >= 0) writeNode(os, nodes, node.child[i]);
  }
}

// Reads one node (and its subtree) into `nodes`, returning its index.
// `nodes[index] = ...` below always re-indexes rather than caching a
// reference, since the recursive calls in between may reallocate `nodes`.
int32_t readNode(std::istream& is, std::vector<Node>& nodes) {
  uint8_t mask = 0;
  is.read(reinterpret_cast<char*>(&mask), sizeof(mask));
  Node node;
  is.read(reinterpret_cast<char*>(node.detail.data()), sizeof(float) * 3);
  const int32_t index = static_cast<int32_t>(nodes.size());
  nodes.push_back(node);
  for (int i = 0; i < 4; ++i) {
    if (mask & (1u << i)) {
      const int32_t child_index = readNode(is, nodes);
      nodes[index].child[i] = child_index;
    }
  }
  return index;
}

int64_t floorDiv(int64_t a, int64_t b) {
  int64_t q = a / b;
  const int64_t r = a % b;
  if (r != 0 && ((r < 0) != (b < 0))) --q;
  return q;
}

int64_t packBlockKey(int32_t bx, int32_t by) {
  return (static_cast<int64_t>(bx) << 32) | static_cast<uint32_t>(by);
}

// Inverse of packBlockKey(); the uint32_t->int32_t cast below reinterprets
// the low 32 bits as two's complement, which is what packBlockKey produced.
void unpackBlockKey(int64_t key, int32_t& bx, int32_t& by) {
  bx = static_cast<int32_t>(key >> 32);
  by = static_cast<int32_t>(static_cast<uint32_t>(key & 0xffffffffLL));
}

constexpr char kMagic[4] = {'H', 'W', 'Q', '1'};

}  // namespace

struct HashedWaveletQuadtree::Impl {
  int tree_height;
  double min_cell_width;
  float default_value;
  std::unordered_map<int64_t, Block> blocks;

  Impl(int tree_height, double min_cell_width, float default_value)
      : tree_height(tree_height), min_cell_width(min_cell_width), default_value(default_value) {}

  double blockWidth() const { return min_cell_width * static_cast<double>(int64_t(1) << tree_height); }

  struct CellLocation {
    int32_t bx, by;
    int ix, iy;
  };

  CellLocation locate(const Eigen::Vector2d& world_position) const {
    const int64_t gx = static_cast<int64_t>(std::floor(world_position.x() / min_cell_width));
    const int64_t gy = static_cast<int64_t>(std::floor(world_position.y() / min_cell_width));
    const int64_t cells_per_block = int64_t(1) << tree_height;
    const int64_t bx = floorDiv(gx, cells_per_block);
    const int64_t by = floorDiv(gy, cells_per_block);
    return {static_cast<int32_t>(bx), static_cast<int32_t>(by), static_cast<int>(gx - bx * cells_per_block),
            static_cast<int>(gy - by * cells_per_block)};
  }

  float getCellValue(const Eigen::Vector2d& world_position, int height) const {
    const CellLocation loc = locate(world_position);
    const auto it = blocks.find(packBlockKey(loc.bx, loc.by));
    if (it == blocks.end()) return default_value;
    return it->second.getCellValue(loc.ix, loc.iy, height);
  }

  void addToCellValue(const Eigen::Vector2d& world_position, float delta) {
    const CellLocation loc = locate(world_position);
    Block& block = blocks.try_emplace(packBlockKey(loc.bx, loc.by), tree_height).first->second;
    block.addToCellValue(loc.ix, loc.iy, delta);
  }

  // NOTE: deliberately does NOT compute its delta from the public
  // getCellValue() above. A not-yet-allocated block's true starting point
  // (once allocated, right below) is 0.f -- its root_scale_ default --
  // *not* `default_value` (that's a separate, block-existence-gated public
  // API concept). Computing delta against `default_value` here would only
  // be correct when default_value happens to be 0.f; for a nonzero
  // default_value it would silently bake a `value - default_value` offset
  // into a block whose real baseline is 0.f, corrupting every fresh
  // block's first write by exactly `default_value`. So: allocate the block
  // first, then diff against *its own* (block-internal, zero-baseline)
  // current reconstruction, exactly like addToCellValue's delta already
  // does for O(tree height) updates.
  void setCellValue(const Eigen::Vector2d& world_position, float value) {
    const CellLocation loc = locate(world_position);
    Block& block = blocks.try_emplace(packBlockKey(loc.bx, loc.by), tree_height).first->second;
    block.addToCellValue(loc.ix, loc.iy, value - block.getCellValue(loc.ix, loc.iy, 0));
  }

  std::vector<HashedWaveletQuadtree::Cell> getCells(const Eigen::Vector2d& region_min,
                                                     const Eigen::Vector2d& region_max) const {
    std::vector<HashedWaveletQuadtree::Cell> result;
    const CellLocation loc_min = locate(region_min);
    const CellLocation loc_max = locate(region_max);
    const double cells_per_block = static_cast<double>(int64_t(1) << tree_height);
    std::vector<LocalCell> local_cells;
    for (int32_t bx = loc_min.bx; bx <= loc_max.bx; ++bx) {
      for (int32_t by = loc_min.by; by <= loc_max.by; ++by) {
        const auto it = blocks.find(packBlockKey(bx, by));
        if (it == blocks.end()) continue;
        local_cells.clear();
        it->second.collectCells(local_cells);
        const Eigen::Vector2d block_min(bx * cells_per_block * min_cell_width, by * cells_per_block * min_cell_width);
        for (const LocalCell& lc : local_cells) {
          const double size = lc.span * min_cell_width;
          const Eigen::Vector2d min_corner = block_min + Eigen::Vector2d(lc.local_x0, lc.local_y0) * min_cell_width;
          // Include by CENTER falling in the region, not by any bounding-box
          // overlap: hashed blocks (blockWidth() wide) don't generally
          // divide evenly into a source raster's extent, so blocks along
          // the true data's edge extend into never-written space beyond
          // it. Those regions still collapse (they're uniformly
          // default_value) into their own cells, which -- under a mere
          // "touches the region at all" test -- would get reported even
          // when barely inside (an edge-touching cell with zero-area
          // overlap) or only marginally so (a large all-default padding
          // cell whose bulk sits outside, see boundary bug report). A
          // cell's center is representative of where it actually sits, and
          // cheaply excludes both cases without needing to track "was this
          // leaf ever explicitly written" as separate state.
          const Eigen::Vector2d center_pos = min_corner + Eigen::Vector2d(size, size) * 0.5;
          if (center_pos.x() < region_min.x() || center_pos.x() > region_max.x() ||
              center_pos.y() < region_min.y() || center_pos.y() > region_max.y()) {
            continue;
          }
          result.push_back(HashedWaveletQuadtree::Cell{min_corner, size, lc.value});
        }
      }
    }
    return result;
  }

  void boundedPrune(float max_error) {
    const double cells_per_block = static_cast<double>(int64_t(1) << tree_height);
    std::vector<LocalCell> plan;
    // Computing each block's plan BEFORE any rewriting, into a plain
    // value-type vector (no references into the tree), then applying it
    // afterward -- mixing the two would mean rewriting a block while
    // still reading its (now-stale) merge decisions. Rewriting only ever
    // touches the SAME block key already being iterated (every plan
    // entry's coordinates stay within that block's own footprint, by
    // construction -- see computeBoundedMergePlan), so this never inserts
    // a new key into `blocks` and never invalidates this loop.
    for (auto& block_entry : blocks) {
      int32_t bx, by;
      unpackBlockKey(block_entry.first, bx, by);
      plan.clear();
      block_entry.second.computeBoundedMergePlan(max_error, plan);
      if (plan.empty()) continue;
      const Eigen::Vector2d block_min(bx * cells_per_block * min_cell_width, by * cells_per_block * min_cell_width);
      for (const LocalCell& lc : plan) {
        for (int dx = 0; dx < lc.span; ++dx) {
          for (int dy = 0; dy < lc.span; ++dy) {
            const Eigen::Vector2d world_position =
                block_min + Eigen::Vector2d(lc.local_x0 + dx + 0.5, lc.local_y0 + dy + 0.5) * min_cell_width;
            setCellValue(world_position, lc.value);
          }
        }
      }
    }
    // The rewrite above makes every merged region exactly constant, so
    // the existing lossless threshold()/prune() collapses it for free --
    // all the actual tree mutation happens through code already covered
    // by the rest of this class's test suite, not new coefficient math.
    for (auto& block_entry : blocks) {
      block_entry.second.threshold();
      block_entry.second.prune();
    }
  }
};

HashedWaveletQuadtree::HashedWaveletQuadtree(int tree_height, double min_cell_width, float default_value)
    : impl_(std::make_unique<Impl>(tree_height, min_cell_width, default_value)) {}

HashedWaveletQuadtree::~HashedWaveletQuadtree() = default;
HashedWaveletQuadtree::HashedWaveletQuadtree(HashedWaveletQuadtree&&) noexcept = default;
HashedWaveletQuadtree& HashedWaveletQuadtree::operator=(HashedWaveletQuadtree&&) noexcept = default;

float HashedWaveletQuadtree::getCellValue(const Eigen::Vector2d& world_position) const {
  return getCellValue(world_position, 0);
}

float HashedWaveletQuadtree::getCellValue(const Eigen::Vector2d& world_position, int height) const {
  return impl_->getCellValue(world_position, height);
}

void HashedWaveletQuadtree::setCellValue(const Eigen::Vector2d& world_position, float value) {
  impl_->setCellValue(world_position, value);
}

void HashedWaveletQuadtree::addToCellValue(const Eigen::Vector2d& world_position, float update) {
  impl_->addToCellValue(world_position, update);
}

void HashedWaveletQuadtree::reconstructRegion(grid_map::GridMap& map, const std::string& layer_name,
                                              int query_height, const Eigen::Vector2d& origin_offset) const {
  grid_map::Matrix& layer = map[layer_name];
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Eigen::Vector2d position;
    map.getPosition(*it, position);
    const grid_map::Index index(*it);
    layer(index(0), index(1)) = getCellValue(position + origin_offset, query_height);
  }
}

double HashedWaveletQuadtree::getMinCellWidth() const { return impl_->min_cell_width; }

double HashedWaveletQuadtree::resolutionAtHeight(int height) const {
  return impl_->min_cell_width * static_cast<double>(int64_t(1) << height);
}

int HashedWaveletQuadtree::getTreeHeight() const { return impl_->tree_height; }

void HashedWaveletQuadtree::threshold() {
  for (auto& block_entry : impl_->blocks) block_entry.second.threshold();
}

void HashedWaveletQuadtree::prune() {
  for (auto& block_entry : impl_->blocks) block_entry.second.prune();
}

void HashedWaveletQuadtree::boundedPrune(float max_error) { impl_->boundedPrune(max_error); }

size_t HashedWaveletQuadtree::getMemoryUsage() const {
  size_t total = 0;
  for (const auto& [key, block] : impl_->blocks) total += block.getMemoryUsage();
  return total;
}

bool HashedWaveletQuadtree::empty() const { return impl_->blocks.empty(); }

std::vector<HashedWaveletQuadtree::BlockInfo> HashedWaveletQuadtree::getBlockInfo() const {
  std::vector<BlockInfo> result;
  result.reserve(impl_->blocks.size());
  const double block_width = impl_->blockWidth();
  for (const auto& [key, block] : impl_->blocks) {
    int32_t bx, by;
    unpackBlockKey(key, bx, by);
    result.push_back(BlockInfo{Eigen::Vector2d(bx * block_width, by * block_width), block_width, block.rootScale(),
                               block.nodes().size()});
  }
  return result;
}

std::vector<HashedWaveletQuadtree::Cell> HashedWaveletQuadtree::getCells(const Eigen::Vector2d& region_min,
                                                                         const Eigen::Vector2d& region_max) const {
  return impl_->getCells(region_min, region_max);
}

bool HashedWaveletQuadtree::saveToFile(const std::string& path) const {
  std::ofstream os(path, std::ios::binary);
  if (!os) return false;
  os.write(kMagic, sizeof(kMagic));
  const int32_t tree_height = impl_->tree_height;
  const double min_cell_width = impl_->min_cell_width;
  const float default_value = impl_->default_value;
  const uint64_t num_blocks = impl_->blocks.size();
  os.write(reinterpret_cast<const char*>(&tree_height), sizeof(tree_height));
  os.write(reinterpret_cast<const char*>(&min_cell_width), sizeof(min_cell_width));
  os.write(reinterpret_cast<const char*>(&default_value), sizeof(default_value));
  os.write(reinterpret_cast<const char*>(&num_blocks), sizeof(num_blocks));
  for (const auto& [key, block] : impl_->blocks) {
    int32_t bx, by;
    unpackBlockKey(key, bx, by);
    os.write(reinterpret_cast<const char*>(&bx), sizeof(bx));
    os.write(reinterpret_cast<const char*>(&by), sizeof(by));
    const float root_scale = block.rootScale();
    os.write(reinterpret_cast<const char*>(&root_scale), sizeof(root_scale));
    const uint8_t has_root = block.hasAnyNode() ? 1 : 0;
    os.write(reinterpret_cast<const char*>(&has_root), sizeof(has_root));
    if (has_root) writeNode(os, block.nodes(), block.rootIndex());
    if (!os) return false;
  }
  return static_cast<bool>(os);
}

bool HashedWaveletQuadtree::loadFromFile(const std::string& path) {
  std::ifstream is(path, std::ios::binary);
  if (!is) return false;
  char magic[4];
  is.read(magic, sizeof(magic));
  if (!is || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) return false;

  int32_t tree_height = 0;
  double min_cell_width = 0.0;
  float default_value = 0.0f;
  uint64_t num_blocks = 0;
  is.read(reinterpret_cast<char*>(&tree_height), sizeof(tree_height));
  is.read(reinterpret_cast<char*>(&min_cell_width), sizeof(min_cell_width));
  is.read(reinterpret_cast<char*>(&default_value), sizeof(default_value));
  is.read(reinterpret_cast<char*>(&num_blocks), sizeof(num_blocks));
  if (!is) return false;

  auto new_impl = std::make_unique<Impl>(tree_height, min_cell_width, default_value);
  for (uint64_t i = 0; i < num_blocks; ++i) {
    int32_t bx = 0, by = 0;
    float root_scale = 0.f;
    uint8_t has_root = 0;
    is.read(reinterpret_cast<char*>(&bx), sizeof(bx));
    is.read(reinterpret_cast<char*>(&by), sizeof(by));
    is.read(reinterpret_cast<char*>(&root_scale), sizeof(root_scale));
    is.read(reinterpret_cast<char*>(&has_root), sizeof(has_root));
    if (!is) return false;
    Block block(tree_height);
    block.setRootScale(root_scale);
    if (has_root) {
      const int32_t root_index = readNode(is, block.nodesMutable());
      block.setRootIndex(root_index);
      if (!is) return false;
    }
    new_impl->blocks.emplace(packBlockKey(bx, by), std::move(block));
  }
  impl_ = std::move(new_impl);
  return true;
}

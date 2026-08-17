#include "grid_map_geo/hashed_wavelet_quadtree.hpp"

#include <fstream>
#include <utility>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <wavemap/core/data_structure/chunked_ndtree/chunked_ndtree.h>
#include <wavemap/core/data_structure/spatial_hash.h>
#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/map/cell_types/haar_coefficients.h>
#include <wavemap/core/map/cell_types/haar_transform.h>
#include <wavemap/core/utils/math/int_math.h>

// This file instantiates wavemap's (ethz-asl/wavemap) generic, dim/value-
// templated wavelet-tree primitives at dim=2 with a plain float value, to
// get a "wavelet quadtree" for 2.5D elevation data. It deliberately mirrors
// wavemap's own dim=3 HashedChunkedWaveletOctree/HashedChunkedWaveletOctreeBlock
// classes (library/cpp/include/wavemap/core/map/hashed_chunked_wavelet_octree*.h)
// rather than wavemap's public map classes, which fix dim=3 and occupancy
// log-odds semantics that don't apply to elevation.
//
// Deliberately the *chunked* tree variant, not the plain per-node-heap-
// allocated Ndtree used in an earlier version of this file: a plain Ndtree
// allocates one heap object per tree node (payload + a child-array pointer),
// which costs more memory than a dense array whenever a region has little
// local redundancy to prune away (confirmed on real terrain: it used ~2.7x
// a dense float array's memory). ChunkedNdtree instead packs several tree
// levels' worth of nodes into one contiguous array per chunk, amortizing
// that per-node bookkeeping -- wavemap's own docs recommend it as the best
// variant for both speed and RAM usage.

namespace {

constexpr int kDim = 2;
// Number of tree levels packed into each contiguous chunk allocation.
// Matches wavemap's own default for its (dim=3) HashedChunkedWaveletOctree;
// we have no profiling data suggesting a different value is better for
// dim=2, so we keep it rather than introduce an untuned free parameter.
constexpr int kChunkHeight = 3;

using wavemap::FloatingPoint;
using QuadtreeIndex = wavemap::NdtreeIndex<kDim>;
using Coefficients = wavemap::HaarCoefficients<FloatingPoint, kDim>;
using Transform = wavemap::HaarTransform<FloatingPoint, kDim>;
using QuadtreeType = wavemap::ChunkedQuadtree<Coefficients::Details, kChunkHeight>;
using Index2D = wavemap::Index2D;
using Point2D = wavemap::Point2D;

// Mirrors wavemap::HashedChunkedWaveletOctreeBlock, at dim=2, without the
// occupancy-specific min/max log-odds clamping (elevation has no natural
// bound to clamp values to).
class QuadtreeBlock {
 public:
  explicit QuadtreeBlock(int tree_height)
      : tree_height_(tree_height), ndtree_(tree_height_ - 1) {}

  bool empty() const {
    return !ndtree_.getRootNode().hasAtLeastOneChild() &&
          !ndtree_.getRootNode().hasNonzeroData();
  }
  size_t size() const { return ndtree_.size(); }
  size_t getMemoryUsage() const { return ndtree_.getMemoryUsage(); }

  FloatingPoint getCellValue(const QuadtreeIndex& index) const {
    const wavemap::MortonIndex morton_code =
        wavemap::convert::nodeIndexToMorton(index);
    QuadtreeType::NodeConstPtrType node = &ndtree_.getRootNode();
    FloatingPoint value = root_scale_coefficient_;
    for (int parent_height = tree_height_; node && index.height < parent_height;
        --parent_height) {
      const wavemap::NdtreeIndexRelativeChild child_index =
          QuadtreeIndex::computeRelativeChildIndex(morton_code, parent_height);
      value = Transform::backwardSingleChild({value, node->data()}, child_index);
      node = node->getChild(child_index);
    }
    return value;
  }

  // NOTE: wavemap's own *SetCellValue (which this class otherwise mirrors)
  // computes its injected delta relative to the reconstructed value one
  // level *above* the target leaf (its immediate parent's own scale
  // coefficient), not the leaf's own current value. That is only equivalent
  // to "overwrite this leaf's value" on a leaf that has never been touched
  // before (both happen to be zero); on a second set/overwrite of the same
  // leaf it produces the wrong result, verified both in this port and in
  // wavemap's own unmodified dim=3 classes (plain and chunked). Since our
  // use case (absolute overwrite for Kalman-fused elevation) genuinely
  // needs repeated overwrites of the same cell to behave correctly,
  // implement setCellValue directly in terms of the (already exact)
  // addToCellValue: read the leaf's true current value via getCellValue(),
  // then inject exactly the delta needed.
  void setCellValue(const QuadtreeIndex& index, FloatingPoint new_value) {
    addToCellValue(index, new_value - getCellValue(index));
  }

  void addToCellValue(const QuadtreeIndex& index, FloatingPoint update) {
    setNeedsPruning();
    setNeedsThresholding();
    const wavemap::MortonIndex morton_code =
        wavemap::convert::nodeIndexToMorton(index);
    std::vector<QuadtreeType::NodeRefType> ancestors;
    const int height_difference = tree_height_ - index.height;
    ancestors.reserve(height_difference);
    ancestors.emplace_back(ndtree_.getRootNode());
    for (int parent_height = tree_height_; index.height + 1 < parent_height;
        --parent_height) {
      const wavemap::NdtreeIndexRelativeChild child_index =
          QuadtreeIndex::computeRelativeChildIndex(morton_code, parent_height);
      QuadtreeType::NodeRefType current_parent = ancestors.back();
      QuadtreeType::NodeRefType child =
          current_parent.getOrAllocateChild(child_index);
      ancestors.emplace_back(child);
    }

    Coefficients::Parent coefficients{update, {}};
    for (int parent_height = index.height + 1; parent_height <= tree_height_;
        ++parent_height) {
      QuadtreeType::NodeRefType current_node = ancestors.back();
      ancestors.pop_back();
      const wavemap::NdtreeIndexRelativeChild child_index =
          QuadtreeIndex::computeRelativeChildIndex(morton_code, parent_height);
      coefficients =
          Transform::forwardSingleChild(coefficients.scale, child_index);
      current_node.data() += coefficients.details;
    }
    root_scale_coefficient_ += coefficients.scale;
  }

  void threshold() {
    if (needs_thresholding_) {
      recursiveThreshold(ndtree_.getRootNode(), root_scale_coefficient_);
      needs_thresholding_ = false;
    }
  }

  void prune() {
    if (needs_pruning_) {
      threshold();
      recursivePrune(ndtree_.getRootNode());
      needs_pruning_ = false;
    }
  }

  int treeHeight() const { return tree_height_; }
  Coefficients::Scale& rootScale() { return root_scale_coefficient_; }
  const Coefficients::Scale& rootScale() const { return root_scale_coefficient_; }
  QuadtreeType::NodeRefType rootNode() { return ndtree_.getRootNode(); }
  QuadtreeType::NodeConstRefType rootNode() const { return ndtree_.getRootNode(); }

 private:
  int tree_height_;
  QuadtreeType ndtree_;
  Coefficients::Scale root_scale_coefficient_{0.f};
  bool needs_pruning_ = false;
  bool needs_thresholding_ = false;

  void setNeedsPruning() { needs_pruning_ = true; }
  void setNeedsThresholding() { needs_thresholding_ = true; }

  static void recursiveThreshold(QuadtreeType::NodeRefType node,
                                 FloatingPoint& node_scale_coefficient) {
    auto& node_detail_coefficients = node.data();
    Coefficients::CoefficientsArray child_scale_coefficients =
        Transform::backward({node_scale_coefficient, node_detail_coefficients});
    for (int child_idx = 0; child_idx < QuadtreeIndex::kNumChildren;
        ++child_idx) {
      if (auto child_node = node.getChild(child_idx); child_node) {
        recursiveThreshold(*child_node, child_scale_coefficients[child_idx]);
      }
      // No clamping step here: unlike occupancy log-odds, elevation has no
      // natural bound. Thresholding is purely about re-compressing the
      // detail coefficients below, not about clamping values.
    }
    const auto forward_result = Transform::forward(child_scale_coefficients);
    node_detail_coefficients = forward_result.details;
    node_scale_coefficient = forward_result.scale;
  }

  static void recursivePrune(QuadtreeType::NodeRefType node) {
    bool has_at_least_one_child = false;
    for (int child_idx = 0; child_idx < QuadtreeIndex::kNumChildren;
        ++child_idx) {
      if (auto child_node = node.getChild(child_idx); child_node) {
        recursivePrune(*child_node);
        if (!child_node->hasAtLeastOneChild() &&
            !child_node->hasNonzeroData(1e-3f)) {
          node.eraseChild(child_idx);
        } else {
          has_at_least_one_child = true;
        }
      }
    }
    node.hasAtLeastOneChild() = has_at_least_one_child;
  }
};

void writeNode(std::ostream& os, QuadtreeType::NodeConstRefType node) {
  uint8_t child_mask = 0;
  for (int i = 0; i < QuadtreeIndex::kNumChildren; ++i) {
    if (node.hasChild(i)) child_mask |= static_cast<uint8_t>(1u << i);
  }
  os.write(reinterpret_cast<const char*>(&child_mask), sizeof(child_mask));
  const auto& details = node.data();
  os.write(reinterpret_cast<const char*>(details.data()),
          sizeof(FloatingPoint) * Coefficients::kNumDetailCoefficients);
  for (int i = 0; i < QuadtreeIndex::kNumChildren; ++i) {
    if (node.hasChild(i)) {
      writeNode(os, *node.getChild(i));
    }
  }
}

void readNode(std::istream& is, QuadtreeType::NodeRefType node) {
  uint8_t child_mask = 0;
  is.read(reinterpret_cast<char*>(&child_mask), sizeof(child_mask));
  is.read(reinterpret_cast<char*>(node.data().data()),
         sizeof(FloatingPoint) * Coefficients::kNumDetailCoefficients);
  for (int i = 0; i < QuadtreeIndex::kNumChildren; ++i) {
    if (child_mask & (1u << i)) {
      QuadtreeType::NodeRefType child = node.getOrAllocateChild(i);
      readNode(is, child);
    }
  }
}

}  // namespace

struct HashedWaveletQuadtree::Impl {
  int tree_height;
  double min_cell_width;
  float default_value;
  wavemap::SpatialHash<QuadtreeBlock, kDim> block_map;

  Impl(int tree_height, double min_cell_width, float default_value)
      : tree_height(tree_height),
        min_cell_width(min_cell_width),
        default_value(default_value) {}

  Index2D indexToBlockIndex(const QuadtreeIndex& node_index) const {
    const Index2D min_corner_index =
        wavemap::convert::nodeIndexToMinCornerIndex(node_index);
    return wavemap::convert::indexToBlockIndex(min_corner_index, tree_height);
  }

  QuadtreeIndex indexToCellIndex(QuadtreeIndex index) const {
    const int height_difference = tree_height - index.height;
    index.position = wavemap::int_math::div_exp2_floor_remainder(
        index.position, height_difference);
    return index;
  }

  QuadtreeIndex worldToNodeIndex(const Eigen::Vector2d& world_position,
                                 int height) const {
    const Point2D point = world_position.cast<FloatingPoint>();
    return wavemap::convert::pointToNodeIndex<kDim>(
        point, static_cast<FloatingPoint>(min_cell_width), height);
  }

  FloatingPoint getCellValue(const QuadtreeIndex& index) const {
    const Index2D block_index = indexToBlockIndex(index);
    const QuadtreeBlock* block = block_map.getBlock(block_index);
    if (!block) {
      return default_value;
    }
    return block->getCellValue(indexToCellIndex(index));
  }

  void setCellValue(const QuadtreeIndex& index, FloatingPoint value) {
    const Index2D block_index = indexToBlockIndex(index);
    QuadtreeBlock& block = block_map.getOrAllocateBlock(block_index, tree_height);
    block.setCellValue(indexToCellIndex(index), value);
  }

  void addToCellValue(const QuadtreeIndex& index, FloatingPoint update) {
    const Index2D block_index = indexToBlockIndex(index);
    QuadtreeBlock& block = block_map.getOrAllocateBlock(block_index, tree_height);
    block.addToCellValue(indexToCellIndex(index), update);
  }
};

HashedWaveletQuadtree::HashedWaveletQuadtree(int tree_height,
                                             double min_cell_width,
                                             float default_value)
    : impl_(std::make_unique<Impl>(tree_height, min_cell_width, default_value)) {}

HashedWaveletQuadtree::~HashedWaveletQuadtree() = default;
HashedWaveletQuadtree::HashedWaveletQuadtree(HashedWaveletQuadtree&&) noexcept = default;
HashedWaveletQuadtree& HashedWaveletQuadtree::operator=(HashedWaveletQuadtree&&) noexcept = default;

float HashedWaveletQuadtree::getCellValue(const Eigen::Vector2d& world_position) const {
  return getCellValue(world_position, 0);
}

float HashedWaveletQuadtree::getCellValue(const Eigen::Vector2d& world_position,
                                          int height) const {
  return impl_->getCellValue(impl_->worldToNodeIndex(world_position, height));
}

void HashedWaveletQuadtree::setCellValue(const Eigen::Vector2d& world_position,
                                         float value) {
  impl_->setCellValue(impl_->worldToNodeIndex(world_position, 0), value);
}

void HashedWaveletQuadtree::addToCellValue(const Eigen::Vector2d& world_position,
                                           float update) {
  impl_->addToCellValue(impl_->worldToNodeIndex(world_position, 0), update);
}

void HashedWaveletQuadtree::reconstructRegion(grid_map::GridMap& map,
                                              const std::string& layer_name,
                                              int query_height,
                                              const Eigen::Vector2d& origin_offset) const {
  grid_map::Matrix& layer = map[layer_name];
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Eigen::Vector2d position;
    map.getPosition(*it, position);
    const grid_map::Index index(*it);
    layer(index(0), index(1)) = getCellValue(position + origin_offset, query_height);
  }
}

double HashedWaveletQuadtree::getMinCellWidth() const {
  return impl_->min_cell_width;
}

double HashedWaveletQuadtree::resolutionAtHeight(int height) const {
  return impl_->min_cell_width * static_cast<double>(wavemap::int_math::exp2(height));
}

int HashedWaveletQuadtree::getTreeHeight() const { return impl_->tree_height; }

void HashedWaveletQuadtree::threshold() {
  impl_->block_map.forEachBlock(
      [](const Index2D& /*block_index*/, QuadtreeBlock& block) { block.threshold(); });
}

void HashedWaveletQuadtree::prune() {
  impl_->block_map.forEachBlock(
      [](const Index2D& /*block_index*/, QuadtreeBlock& block) { block.prune(); });
}

size_t HashedWaveletQuadtree::getMemoryUsage() const {
  size_t total = 0;
  impl_->block_map.forEachBlock(
      [&total](const Index2D& /*block_index*/, const QuadtreeBlock& block) {
        total += block.getMemoryUsage();
      });
  return total;
}

bool HashedWaveletQuadtree::empty() const { return impl_->block_map.empty(); }

bool HashedWaveletQuadtree::saveToFile(const std::string& path) const {
  std::ofstream os(path, std::ios::binary);
  if (!os) return false;
  const int32_t tree_height = impl_->tree_height;
  const double min_cell_width = impl_->min_cell_width;
  const float default_value = impl_->default_value;
  uint64_t num_blocks = 0;
  impl_->block_map.forEachBlock(
      [&num_blocks](const Index2D&, const QuadtreeBlock&) { ++num_blocks; });
  os.write(reinterpret_cast<const char*>(&tree_height), sizeof(tree_height));
  os.write(reinterpret_cast<const char*>(&min_cell_width), sizeof(min_cell_width));
  os.write(reinterpret_cast<const char*>(&default_value), sizeof(default_value));
  os.write(reinterpret_cast<const char*>(&num_blocks), sizeof(num_blocks));
  bool ok = true;
  impl_->block_map.forEachBlock([&os, &ok](const Index2D& block_index,
                                           const QuadtreeBlock& block) {
    if (!ok) return;
    const int32_t bx = block_index.x();
    const int32_t by = block_index.y();
    os.write(reinterpret_cast<const char*>(&bx), sizeof(bx));
    os.write(reinterpret_cast<const char*>(&by), sizeof(by));
    const Coefficients::Scale root_scale = block.rootScale();
    os.write(reinterpret_cast<const char*>(&root_scale), sizeof(root_scale));
    writeNode(os, block.rootNode());
    if (!os) ok = false;
  });
  return ok && static_cast<bool>(os);
}

bool HashedWaveletQuadtree::loadFromFile(const std::string& path) {
  std::ifstream is(path, std::ios::binary);
  if (!is) return false;
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
    is.read(reinterpret_cast<char*>(&bx), sizeof(bx));
    is.read(reinterpret_cast<char*>(&by), sizeof(by));
    Coefficients::Scale root_scale = 0.f;
    is.read(reinterpret_cast<char*>(&root_scale), sizeof(root_scale));
    if (!is) return false;
    QuadtreeBlock& block =
        new_impl->block_map.getOrAllocateBlock(Index2D(bx, by), tree_height);
    block.rootScale() = root_scale;
    readNode(is, block.rootNode());
    if (!is) return false;
  }
  impl_ = std::move(new_impl);
  return true;
}

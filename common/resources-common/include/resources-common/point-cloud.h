#ifndef RESOURCES_COMMON_POINT_CLOUD_H_
#define RESOURCES_COMMON_POINT_CLOUD_H_

#include <vector>

#include <Eigen/Core>

namespace resources {

typedef Eigen::Matrix<uint8_t, 4, 1> RgbaColor;

struct PointCloud {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<float> xyz;
  std::vector<float> normals;
  std::vector<unsigned char> colors;
  std::vector<float> scalars;

  void resize(
      const size_t size, const bool has_normals = true,
      const bool has_colors = true, const bool has_scalars = true) {
    xyz.resize(3 * size);

    if (has_normals) {
      normals.resize(3 * size);
    }

    if (has_colors) {
      colors.resize(3 * size);
    }

    if (has_scalars) {
      scalars.resize(1 * size);
    }
  }

  void reserve(
      const size_t size, const bool has_normals = true,
      const bool has_colors = true, const bool has_scalars = true) {
    xyz.reserve(3 * size);

    if (has_normals) {
      normals.reserve(3 * size);
    }

    if (has_colors) {
      colors.reserve(3 * size);
    }

    if (has_scalars) {
      scalars.reserve(1 * size);
    }
  }

  size_t size() const {
    CHECK_EQ(xyz.size() % 3, 0u);
    return (xyz.size() / 3);
  }

  bool empty() const {
    return xyz.empty();
  }

  bool operator==(const PointCloud& other) const {
    bool is_same = xyz == other.xyz;
    is_same &= normals == other.normals;
    is_same &= colors == other.colors;
    is_same &= scalars == other.scalars;
    return is_same;
  }
};

}  // namespace resources

#endif  // RESOURCES_COMMON_POINT_CLOUD_H_

#pragma once

#include <vector>

namespace RILO{

struct PointXYZ {
  float x;
  float y;
  float z;

  friend inline bool operator==(const PointXYZ& lhs, const PointXYZ& rhs) {
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
  }
  friend inline bool operator!=(const PointXYZ& lhs, const PointXYZ& rhs) { return !(lhs == rhs); }
};

struct PointXYZI {
  float x;
  float y;
  float z;
  float intensity;

  friend inline bool operator==(const PointXYZI& lhs, const PointXYZI& rhs) {
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z) && (lhs.intensity == rhs.intensity);
  }
  friend inline bool operator!=(const PointXYZI& lhs, const PointXYZI& rhs) { return !(lhs == rhs); }
};

template<typename PointT>
class PointCloud {
public:
    /**
     * @brief Default constructor for PointCloud
     */
    PointCloud() = default;
    typedef std::shared_ptr<PointCloud> Ptr;

    // c++ container named requirements
    using value_type = PointT;
    using reference = PointT&;
    using const_reference = const PointT&;
    using difference_type = typename std::vector<PointT>::difference_type;
    using size_type = typename std::vector<PointT>::size_type;

    // iterators
    using iterator = typename std::vector<PointT>::iterator;
    using const_iterator = typename std::vector<PointT>::const_iterator;
    inline iterator begin() { return points_.begin(); }
    inline iterator end() { return points_.end(); }
    inline const_iterator begin() const { return points_.begin(); }
    inline const_iterator end() const { return points_.end(); }

    // capacity
    inline size_t size() const { return points_.size(); }
    inline void reserve(size_t n) { points_.reserve(n); }
    inline bool empty() { return points_.empty(); }

    // element access
    inline PointT& operator[](size_t i) { return points_[i]; }
    inline const PointT& operator[](size_t i) const { return points_[i]; }
    inline PointT& at(size_t n) { return points_.at(n); }
    inline const PointT& at(size_t n) const { return points_.at(n); }
    inline PointT& front() { return points_.front(); }
    inline const PointT& front() const { return points_.front(); }
    inline PointT& back() { return points_.back(); }
    inline const PointT& back() const { return points_.back(); }

    inline void push_back(const PointT& pt) { points_.push_back(pt); }
    inline void push_back(PointT& pt) { points_.push_back(pt); }

    inline void clear() { points_.clear(); }

private:
    std::vector<PointT> points_;
};

}


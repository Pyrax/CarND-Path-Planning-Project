#ifndef PATH_PLANNING_COORDS_H
#define PATH_PLANNING_COORDS_H

class Coords {
  std::vector<std::pair<double, double>> points;

public:
  Coords () = default;

  Coords (std::vector<double> x_coords, std::vector<double> y_coords) {
    auto n = x_coords.size();
    this->points.reserve(n);

    for(int i = 0; i < n; i++) {
      this->points.emplace_back(x_coords[i], y_coords[i]);
    }
  }

  Coords (Coords &lhs, Coords &rhs) {
    for (const auto &point : lhs.points) {
      this->points.emplace_back(point);
    }
    for (const auto &point : rhs.points) {
      this->points.emplace_back(point);
    }
  }

  std::vector<double> get_x() {
    std::vector<double> x;
    x.reserve(this->points.size());
    for (auto &point : this->points) {
      x.push_back(point.first);
    }
    return x;
  }

  std::vector<double> get_y() {
    std::vector<double> y;
    y.reserve(this->points.size());
    for (auto &point : this->points) {
      y.push_back(point.second);
    }
    return y;
  }
};

#endif //PATH_PLANNING_COORDS_H

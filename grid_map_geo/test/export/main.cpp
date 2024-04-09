#include <iostream>

#include "grid_map_geo/grid_map_geo.hpp"

int main() {
  // Instantiate the class and call it so this doesn't get optimized out.
  auto gmg = GridMapGeo();
  std::cout << gmg.getCoordinateName() << std::endl;
  return 0;
}

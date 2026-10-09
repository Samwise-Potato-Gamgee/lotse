// The warehouse grid: free cells, obstacles and named stations read from a
// text file. The reader also accepts the public MAPF benchmark map format.

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "lotse/core/grid.hpp"

namespace lotse::core {

/// A 4-connected grid of free cells and obstacles with optional named stations.
class Map {
public:
    /// Parses a map from text. Headers are `type`, `name`, `height`, `width`,
    /// then `map` and `height` rows. An optional `stations` block follows with
    /// `name x y` lines.
    /// @param text the whole map file contents
    /// @return the parsed map; throws std::runtime_error on malformed input
    [[nodiscard]] static Map fromText(std::string_view text);

    /// Loads a map from a file.
    /// @param path file to read
    /// @return the parsed map, or nothing when the file is missing or malformed
    [[nodiscard]] static std::optional<Map> fromFile(const std::string& path);

    /// Returns the map name, "map" when the file had no `name` line.
    [[nodiscard]] const std::string& name() const noexcept { return name_; }

    /// Returns the number of columns.
    [[nodiscard]] int width() const noexcept { return width_; }

    /// Returns the number of rows.
    [[nodiscard]] int height() const noexcept { return height_; }

    /// Returns true when the cell lies inside the grid.
    /// @param cell cell to test
    [[nodiscard]] bool contains(GridCell cell) const noexcept;

    /// Returns true when the cell is inside the grid and not an obstacle.
    /// @param cell cell to test
    [[nodiscard]] bool isFree(GridCell cell) const noexcept;

    /// Returns the free cells in row-major order.
    [[nodiscard]] const std::vector<GridCell>& freeCells() const noexcept { return free_cells_; }

    /// Looks up a named station.
    /// @param station_name station name to find
    /// @return the station cell, or nothing when the name is unknown
    [[nodiscard]] std::optional<GridCell> station(std::string_view station_name) const;

    /// Returns all named stations in file order.
    [[nodiscard]] const std::vector<std::pair<std::string, GridCell>>& stations() const noexcept {
        return stations_;
    }

    /// Formats the map as text: '.' for free cells, '#' for obstacles.
    [[nodiscard]] std::string toString() const;

private:
    std::string name_ = "map";
    int width_ = 0;
    int height_ = 0;
    std::vector<bool> free_;
    std::vector<GridCell> free_cells_;
    std::vector<std::pair<std::string, GridCell>> stations_;
};

}  // namespace lotse::core

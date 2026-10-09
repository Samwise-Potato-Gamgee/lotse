// Implements the map reader declared in map.hpp.

#include "lotse/core/map.hpp"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace lotse::core {
namespace {

/// Splits text into lines and drops a trailing carriage return from each.
/// @param text the text to split
[[nodiscard]] std::vector<std::string> splitLines(std::string_view text) {
    std::vector<std::string> lines;
    std::istringstream stream{std::string(text)};
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
    }
    return lines;
}

/// Returns the first argument of a header line, empty when there is none.
/// @param line the header line
[[nodiscard]] std::string firstWord(const std::string& line) {
    std::istringstream stream{line};
    std::string word;
    stream >> word;
    return word;
}

/// Returns true for the characters that count as a free cell.
/// @param symbol one map character
[[nodiscard]] bool isFreeSymbol(char symbol) noexcept {
    return symbol == '.' || symbol == 'G' || symbol == 'S';
}

/// The trailing block the reader is currently inside.
enum class Block : std::uint8_t { None, Stations, Parking };

}  // namespace

Map Map::fromText(std::string_view text) {
    const std::vector<std::string> lines = splitLines(text);
    Map map;
    int declaredHeight = -1;
    int declaredWidth = -1;

    std::size_t index = 0;
    bool sawMapKey = false;
    for (; index < lines.size(); ++index) {
        const std::string& line = lines[index];
        if (line.empty()) {
            continue;
        }
        if (line == "map") {
            sawMapKey = true;
            ++index;
            break;
        }
        const std::string key = firstWord(line);
        if (key == "type") {
            continue;
        }
        if (key == "name") {
            std::istringstream stream{line};
            std::string ignored;
            if (!(stream >> ignored >> map.name_)) {
                throw std::runtime_error("map: name line has no value");
            }
            continue;
        }
        if (key == "height") {
            std::istringstream stream{line};
            std::string ignored;
            if (!(stream >> ignored >> declaredHeight)) {
                throw std::runtime_error("map: height line has no value");
            }
            continue;
        }
        if (key == "width") {
            std::istringstream stream{line};
            std::string ignored;
            if (!(stream >> ignored >> declaredWidth)) {
                throw std::runtime_error("map: width line has no value");
            }
            continue;
        }
        throw std::runtime_error("map: unknown header key '" + key + "'");
    }

    if (!sawMapKey || declaredHeight <= 0 || declaredWidth <= 0) {
        throw std::runtime_error("map: missing map block, height or width");
    }

    map.height_ = declaredHeight;
    map.width_ = declaredWidth;
    map.free_.assign(
        static_cast<std::size_t>(declaredHeight) * static_cast<std::size_t>(declaredWidth), false);

    for (int y = 0; y < declaredHeight; ++y, ++index) {
        if (index >= lines.size()) {
            throw std::runtime_error("map: fewer rows than the declared height");
        }
        const std::string& row = lines[index];
        if (static_cast<int>(row.size()) != declaredWidth) {
            throw std::runtime_error("map: row has the wrong width");
        }
        for (int x = 0; x < declaredWidth; ++x) {
            const bool free = isFreeSymbol(row[static_cast<std::size_t>(x)]);
            const auto position =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(declaredWidth)) +
                static_cast<std::size_t>(x);
            map.free_[position] = free;
            if (free) {
                map.free_cells_.push_back(GridCell{x, y});
            }
        }
    }

    while (index < lines.size() && lines[index].empty()) {
        ++index;
    }
    Block block = Block::None;
    for (; index < lines.size(); ++index) {
        const std::string& line = lines[index];
        if (line.empty()) {
            continue;
        }
        if (line == "stations") {
            block = Block::Stations;
            continue;
        }
        if (line == "parking") {
            block = Block::Parking;
            continue;
        }
        if (block == Block::None) {
            throw std::runtime_error("map: unexpected content after the map block");
        }
        std::istringstream stream{line};
        if (block == Block::Stations) {
            std::string stationName;
            int x = 0;
            int y = 0;
            if (!(stream >> stationName >> x >> y)) {
                throw std::runtime_error("map: station line must be 'name x y'");
            }
            const GridCell cell{x, y};
            if (!map.isFree(cell)) {
                throw std::runtime_error("map: station is not on a free cell");
            }
            map.stations_.emplace_back(stationName, cell);
            continue;
        }
        int x = 0;
        int y = 0;
        if (!(stream >> x >> y)) {
            throw std::runtime_error("map: parking line must be 'x y'");
        }
        const GridCell cell{x, y};
        if (!map.isFree(cell)) {
            throw std::runtime_error("map: parking cell is not a free cell");
        }
        map.parking_cells_.push_back(cell);
    }
    return map;
}

std::optional<Map> Map::fromFile(const std::string& path) {
    std::ifstream input{path};
    if (!input.is_open()) {
        return std::nullopt;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    try {
        return Map::fromText(buffer.str());
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

bool Map::contains(GridCell cell) const noexcept {
    return cell.x >= 0 && cell.y >= 0 && cell.x < width_ && cell.y < height_;
}

bool Map::isFree(GridCell cell) const noexcept {
    if (!contains(cell)) {
        return false;
    }
    return free_[(static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(width_)) +
                 static_cast<std::size_t>(cell.x)];
}

std::optional<GridCell> Map::station(std::string_view station_name) const {
    for (const std::pair<std::string, GridCell>& entry : stations_) {
        if (entry.first == station_name) {
            return entry.second;
        }
    }
    return std::nullopt;
}

Map Map::withObstacles(const std::vector<GridCell>& cells) const {
    Map copy = *this;
    bool changed = false;
    for (const GridCell& cell : cells) {
        if (!contains(cell)) {
            continue;
        }
        const auto position =
            (static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(width_)) +
            static_cast<std::size_t>(cell.x);
        if (copy.free_[position]) {
            copy.free_[position] = false;
            changed = true;
        }
    }
    if (!changed) {
        return copy;
    }
    copy.free_cells_.clear();
    copy.free_cells_.reserve(free_cells_.size());
    for (int y = 0; y < copy.height_; ++y) {
        for (int x = 0; x < copy.width_; ++x) {
            const GridCell cell{x, y};
            if (copy.isFree(cell)) {
                copy.free_cells_.push_back(cell);
            }
        }
    }
    return copy;
}

std::string Map::toString() const {
    std::ostringstream out;
    out << "name " << name_ << "\n";
    out << "height " << height_ << "\n";
    out << "width " << width_ << "\n";
    out << "map\n";
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            out << (isFree(GridCell{x, y}) ? '.' : '#');
        }
        out << "\n";
    }
    if (!stations_.empty()) {
        out << "stations\n";
        for (const std::pair<std::string, GridCell>& entry : stations_) {
            out << entry.first << " " << entry.second.x << " " << entry.second.y << "\n";
        }
    }
    if (!parking_cells_.empty()) {
        out << "parking\n";
        for (const GridCell& cell : parking_cells_) {
            out << cell.x << " " << cell.y << "\n";
        }
    }
    return out.str();
}

}  // namespace lotse::core

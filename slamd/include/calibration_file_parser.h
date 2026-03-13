// ─────────────────────────────────────────────────────────────────────────────
// calibration_file_parser.h — shared helper for parsing calibration_file key
// ─────────────────────────────────────────────────────────────────────────────
// Extracts the calibration_file YAML key from a config file, resolves the
// path relative to the config directory, and loads the CalibrationBundle.
// Used by both acme_slamd::loadConfig() and its unit tests.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/calibration.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace thunderbird::slamd {

/// Parse the "calibration_file:" key from a YAML config file and load the
/// referenced CalibrationBundle.  Returns true on success (including when no
/// calibration_file key is present — calibration is left unchanged).
/// On failure, returns false and fills `error`.
inline bool parseCalibrationFile(const std::string& config_path,
                                 CalibrationBundle& calibration,
                                 std::string& error) {
    std::ifstream fs(config_path);
    if (!fs.is_open()) {
        error = "Cannot open config file: " + config_path;
        return false;
    }

    std::string line;
    while (std::getline(fs, line)) {
        // Strip comments.
        auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);

        // Trim leading whitespace and require key at the start of the line.
        size_t first_non_ws = line.find_first_not_of(" \t");
        if (first_non_ws == std::string::npos) continue;
        std::string trimmed = line.substr(first_non_ws);

        constexpr const char* kCalibKey    = "calibration_file:";
        constexpr size_t      kCalibKeyLen = 17;
        if (trimmed.size() < kCalibKeyLen ||
            trimmed.compare(0, kCalibKeyLen, kCalibKey) != 0)
            continue;

        std::string val = trimmed.substr(kCalibKeyLen);

        // Trim leading/trailing whitespace.
        size_t vs = val.find_first_not_of(" \t");
        if (vs == std::string::npos) {
            error = "calibration_file key present but value is empty/whitespace";
            return false;
        }
        val = val.substr(vs);
        size_t ve = val.find_last_not_of(" \t\r\n");
        if (ve != std::string::npos) val = val.substr(0, ve + 1);

        // Strip surrounding matching quotes (either '...' or "..."), if
        // present.
        if (val.size() >= 2 &&
            ((val.front() == '\'' && val.back() == '\'') ||
             (val.front() == '"' && val.back() == '"'))) {
            val = val.substr(1, val.size() - 2);
        }

        if (val.empty()) {
            error = "calibration_file key present but value is empty";
            return false;
        }

        // Resolve relative path against config file directory.
        std::error_code ec;
        auto config_dir = std::filesystem::path(config_path).parent_path();
        auto calib_path = std::filesystem::path(val);
        if (calib_path.is_relative())
            calib_path = config_dir / calib_path;
        auto canonical = std::filesystem::canonical(calib_path, ec);
        if (ec) {
            if (ec == std::errc::no_such_file_or_directory) {
                error = "Calibration file not found: " + calib_path.string();
            } else {
                error = "Failed to access calibration file '" +
                        calib_path.string() + "': " + ec.message();
            }
            return false;
        }
        bool is_file = std::filesystem::is_regular_file(canonical, ec);
        if (ec) {
            error = "Failed to stat calibration file '" + canonical.string() +
                    "': " + ec.message();
            return false;
        }
        if (!is_file) {
            error = "Calibration path is not a regular file: " +
                    canonical.string();
            return false;
        }
        if (!calibration.load_yaml(canonical.string())) {
            error = "Failed to parse calibration file: " + canonical.string();
            return false;
        }
        break;
    }

    error.clear();
    return true;
}

} // namespace thunderbird::slamd

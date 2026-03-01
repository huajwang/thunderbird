// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Decoder factory
// ─────────────────────────────────────────────────────────────────────────────
//
// Maps a model/vendor string to the appropriate IPacketDecoder implementation.
//
// Usage:
//   auto decoder = thunderbird::DecoderFactory::create("vlp16");
//   if (!decoder) { /* unknown model */ }
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/packet_decoder.h"
#include "thunderbird/packet_parser.h"
#include "thunderbird/decoders/velodyne_vlp16.h"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>

namespace thunderbird {

class DecoderFactory {
public:
    /// Create a decoder for the given model name.
    ///
    /// Recognised models:
    ///   "thunderbird"  — native Thunderbird protocol (PacketParser)
    ///   "vlp16"        — Velodyne VLP-16
    ///
    /// Returns nullptr for unrecognised models.
    static std::unique_ptr<IPacketDecoder> create(const std::string& model) {
        std::string lower = model;
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        if (lower == "thunderbird" || lower == "native" || lower.empty()) {
            return std::make_unique<PacketParser>();
        }

        if (lower == "vlp16" || lower == "vlp-16" || lower == "velodyne_vlp16") {
            return std::make_unique<VelodyneVlp16Decoder>();
        }

        return nullptr;
    }

    /// Check whether a model name is recognised.
    static bool is_supported(const std::string& model) {
        return create(model) != nullptr;
    }
};

} // namespace thunderbird

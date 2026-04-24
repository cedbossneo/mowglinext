// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mowgli_unicore_gnss
{

enum class FixSource : uint8_t
{
  kGga,
  kPvtslna,
};

enum class HeadingSource : uint8_t
{
  kHdt,
  kHpr,
};

struct FixData
{
  FixSource source{FixSource::kGga};
  bool valid_fix{false};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  int fix_quality{0};
  int satellites{-1};
  double hdop{-1.0};
  bool has_covariance{false};
  std::array<double, 9> covariance{};
};

struct HeadingData
{
  HeadingSource source{HeadingSource::kHdt};
  double heading_deg{0.0};
  std::optional<double> pitch_deg;
  std::optional<double> roll_deg;
  double variance_deg2{0.0};
};

struct VelocityData
{
  double east_mps{0.0};
  double north_mps{0.0};
  double up_mps{0.0};
  double horizontal_std_mps{0.0};
  double vertical_std_mps{0.0};
};

struct ParsedSentence
{
  std::string sentence_type;
  std::optional<FixData> fix;
  std::optional<HeadingData> heading;
  std::optional<VelocityData> velocity;
};

class Um982Parser
{
public:
  std::optional<ParsedSentence> parse_line(const std::string& line) const;

private:
  static bool validate_nmea_checksum(std::string_view line);
  static bool validate_unicore_crc(std::string_view line);
  static std::vector<std::string_view> split_fields(std::string_view payload);
  static std::string trim(std::string_view text);
  static bool parse_double(std::string_view field, double& value);
  static bool parse_int(std::string_view field, int& value);
  static bool parse_latlon(std::string_view value_field,
                           std::string_view hemi_field,
                           bool is_latitude,
                           double& degrees);

  static std::optional<ParsedSentence> parse_gga(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_hdt(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_hpr(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_pvtslna(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_bestnava(const std::vector<std::string_view>& fields);
};

}  // namespace mowgli_unicore_gnss

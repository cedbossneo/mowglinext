// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_unicore_gnss/um982_parser.hpp"

#include <cctype>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

namespace mowgli_unicore_gnss
{

namespace
{

constexpr std::size_t kPvtslnaAltitudeIndex = 10;
constexpr std::size_t kPvtslnaLatitudeIndex = 11;
constexpr std::size_t kPvtslnaLongitudeIndex = 12;
constexpr std::size_t kPvtslnaAltitudeStdIndex = 13;
constexpr std::size_t kPvtslnaLatitudeStdIndex = 14;
constexpr std::size_t kPvtslnaLongitudeStdIndex = 15;

uint32_t crc32_unicore(std::string_view text)
{
  uint32_t crc = 0U;
  for (const unsigned char ch : text)
  {
    crc ^= static_cast<uint32_t>(ch);
    for (int bit = 0; bit < 8; ++bit)
    {
      const bool lsb = (crc & 1U) != 0U;
      crc >>= 1U;
      if (lsb)
      {
        crc ^= 0xEDB88320U;
      }
    }
  }
  return crc;
}

std::string sentence_suffix(std::string_view type)
{
  if (type.size() <= 3U)
  {
    return std::string(type);
  }
  return std::string(type.substr(type.size() - 3U));
}

}  // namespace

std::optional<ParsedSentence> Um982Parser::parse_line(const std::string& line) const
{
  const std::string trimmed = trim(line);
  if (trimmed.empty())
  {
    return std::nullopt;
  }

  if (trimmed.front() == '$')
  {
    if (!validate_nmea_checksum(trimmed))
    {
      return std::nullopt;
    }

    const std::size_t star = trimmed.find('*');
    const auto fields = split_fields(std::string_view(trimmed).substr(1U, star - 1U));
    if (fields.empty())
    {
      return std::nullopt;
    }

    const std::string suffix = sentence_suffix(fields.front());
    if (suffix == "GGA")
    {
      return parse_gga(fields);
    }
    if (suffix == "HDT")
    {
      return parse_hdt(fields);
    }
    if (suffix == "HPR")
    {
      return parse_hpr(fields);
    }
    return std::nullopt;
  }

  if (trimmed.front() == '#')
  {
    if (!validate_unicore_crc(trimmed))
    {
      return std::nullopt;
    }

    const std::size_t star = trimmed.find('*');
    const auto fields = split_fields(std::string_view(trimmed).substr(1U, star - 1U));
    if (fields.empty())
    {
      return std::nullopt;
    }

    if (fields.front() == "PVTSLNA")
    {
      return parse_pvtslna(fields);
    }
    if (fields.front() == "BESTNAVA")
    {
      return parse_bestnava(fields);
    }
  }

  return std::nullopt;
}

bool Um982Parser::validate_nmea_checksum(std::string_view line)
{
  if (line.size() < 4U || line.front() != '$')
  {
    return false;
  }

  const std::size_t star = line.find('*');
  if (star == std::string_view::npos || star + 2U >= line.size())
  {
    return false;
  }

  uint8_t checksum = 0U;
  for (std::size_t i = 1U; i < star; ++i)
  {
    checksum ^= static_cast<uint8_t>(line[i]);
  }

  char* end = nullptr;
  const auto received = static_cast<unsigned long>(
    std::strtoul(std::string(line.substr(star + 1U, 2U)).c_str(), &end, 16));
  return end != nullptr && *end == '\0' && checksum == static_cast<uint8_t>(received);
}

bool Um982Parser::validate_unicore_crc(std::string_view line)
{
  if (line.size() < 11U || line.front() != '#')
  {
    return false;
  }

  const std::size_t star = line.find('*');
  if (star == std::string_view::npos || star + 8U >= line.size())
  {
    return false;
  }

  const uint32_t calculated = crc32_unicore(line.substr(1U, star - 1U));
  char* end = nullptr;
  const uint32_t received = static_cast<uint32_t>(
    std::strtoul(std::string(line.substr(star + 1U, 8U)).c_str(), &end, 16));
  return end != nullptr && *end == '\0' && calculated == received;
}

std::vector<std::string_view> Um982Parser::split_fields(std::string_view payload)
{
  std::vector<std::string_view> fields;
  std::size_t start = 0U;
  while (start <= payload.size())
  {
    const std::size_t comma = payload.find(',', start);
    if (comma == std::string_view::npos)
    {
      fields.emplace_back(payload.substr(start));
      break;
    }
    fields.emplace_back(payload.substr(start, comma - start));
    start = comma + 1U;
  }
  return fields;
}

std::string Um982Parser::trim(std::string_view text)
{
  std::size_t start = 0U;
  std::size_t end = text.size();
  while (start < end && std::isspace(static_cast<unsigned char>(text[start])) != 0)
  {
    ++start;
  }
  while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1U])) != 0)
  {
    --end;
  }
  return std::string(text.substr(start, end - start));
}

bool Um982Parser::parse_double(std::string_view field, double& value)
{
  if (field.empty())
  {
    return false;
  }

  char* end = nullptr;
  const std::string text(field);
  value = std::strtod(text.c_str(), &end);
  return end != nullptr && *end == '\0' && std::isfinite(value);
}

bool Um982Parser::parse_int(std::string_view field, int& value)
{
  if (field.empty())
  {
    return false;
  }

  char* end = nullptr;
  const std::string text(field);
  const long parsed = std::strtol(text.c_str(), &end, 10);
  if (end == nullptr || *end != '\0')
  {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool Um982Parser::parse_latlon(std::string_view value_field, std::string_view hemi_field, bool is_latitude,
                               double& degrees)
{
  double raw = 0.0;
  if (!parse_double(value_field, raw))
  {
    return false;
  }
  if (hemi_field.size() != 1U)
  {
    return false;
  }

  const double divisor = is_latitude ? 100.0 : 100.0;
  const double whole = std::floor(raw / divisor);
  const double minutes = raw - (whole * divisor);
  degrees = whole + minutes / 60.0;

  const char hemi = static_cast<char>(std::toupper(static_cast<unsigned char>(hemi_field.front())));
  if (hemi == 'S' || hemi == 'W')
  {
    degrees = -degrees;
  }
  return hemi == 'N' || hemi == 'S' || hemi == 'E' || hemi == 'W';
}

std::optional<ParsedSentence> Um982Parser::parse_gga(const std::vector<std::string_view>& fields)
{
  if (fields.size() < 10U)
  {
    return std::nullopt;
  }

  double latitude = 0.0;
  double longitude = 0.0;
  if (!parse_latlon(fields[2], fields[3], true, latitude) ||
      !parse_latlon(fields[4], fields[5], false, longitude))
  {
    return std::nullopt;
  }

  int quality = 0;
  int satellites = -1;
  double hdop = -1.0;
  double altitude_msl = 0.0;
  const bool quality_ok = parse_int(fields[6], quality);
  const bool satellites_ok = fields.size() > 7U && parse_int(fields[7], satellites);
  const bool hdop_ok = fields.size() > 8U && parse_double(fields[8], hdop);
  const bool altitude_ok = parse_double(fields[9], altitude_msl);

  double geoid_separation = 0.0;
  const bool geoid_ok = fields.size() > 11U && parse_double(fields[11], geoid_separation);

  ParsedSentence sentence;
  sentence.sentence_type = "GGA";
  sentence.fix = FixData{};
  sentence.fix->source = FixSource::kGga;
  sentence.fix->valid_fix = quality_ok && quality > 0;
  sentence.fix->latitude_deg = latitude;
  sentence.fix->longitude_deg = longitude;
  sentence.fix->altitude_m = altitude_ok ? altitude_msl + (geoid_ok ? geoid_separation : 0.0) : 0.0;
  sentence.fix->fix_quality = quality_ok ? quality : 0;
  sentence.fix->satellites = satellites_ok ? satellites : -1;
  sentence.fix->hdop = hdop_ok ? hdop : -1.0;
  sentence.fix->has_covariance = false;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_hdt(const std::vector<std::string_view>& fields)
{
  if (fields.size() < 2U)
  {
    return std::nullopt;
  }

  double heading = 0.0;
  if (!parse_double(fields[1], heading))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "HDT";
  sentence.heading = HeadingData{};
  sentence.heading->source = HeadingSource::kHdt;
  sentence.heading->heading_deg = heading;
  sentence.heading->variance_deg2 = 0.0;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_hpr(const std::vector<std::string_view>& fields)
{
  if (fields.size() < 5U)
  {
    return std::nullopt;
  }

  double heading = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  if (!parse_double(fields[2], heading) ||
      !parse_double(fields[3], pitch) ||
      !parse_double(fields[4], roll))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "HPR";
  sentence.heading = HeadingData{};
  sentence.heading->source = HeadingSource::kHpr;
  sentence.heading->heading_deg = heading;
  sentence.heading->pitch_deg = pitch;
  sentence.heading->roll_deg = roll;
  sentence.heading->variance_deg2 = 0.0;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_pvtslna(const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kPvtslnaLongitudeStdIndex)
  {
    return std::nullopt;
  }

  double altitude = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude_std = 0.0;
  double latitude_std = 0.0;
  double longitude_std = 0.0;

  if (!parse_double(fields[kPvtslnaAltitudeIndex], altitude) ||
      !parse_double(fields[kPvtslnaLatitudeIndex], latitude) ||
      !parse_double(fields[kPvtslnaLongitudeIndex], longitude) ||
      !parse_double(fields[kPvtslnaAltitudeStdIndex], altitude_std) ||
      !parse_double(fields[kPvtslnaLatitudeStdIndex], latitude_std) ||
      !parse_double(fields[kPvtslnaLongitudeStdIndex], longitude_std))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "PVTSLNA";
  sentence.fix = FixData{};
  sentence.fix->source = FixSource::kPvtslna;
  sentence.fix->valid_fix = true;
  sentence.fix->latitude_deg = latitude;
  sentence.fix->longitude_deg = longitude;
  sentence.fix->altitude_m = altitude;
  sentence.fix->has_covariance = true;
  sentence.fix->covariance.fill(0.0);
  sentence.fix->covariance[0] = longitude_std * longitude_std;
  sentence.fix->covariance[4] = latitude_std * latitude_std;
  sentence.fix->covariance[8] = altitude_std * altitude_std;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_bestnava(const std::vector<std::string_view>& fields)
{
  if (fields.size() < 6U)
  {
    return std::nullopt;
  }

  double horizontal_speed = 0.0;
  double track_deg = 0.0;
  double up_speed = 0.0;
  double vertical_std = 0.0;
  double horizontal_std = 0.0;

  if (!parse_double(fields[fields.size() - 5U], horizontal_speed) ||
      !parse_double(fields[fields.size() - 4U], track_deg) ||
      !parse_double(fields[fields.size() - 3U], up_speed) ||
      !parse_double(fields[fields.size() - 2U], vertical_std) ||
      !parse_double(fields[fields.size() - 1U], horizontal_std))
  {
    return std::nullopt;
  }

  const double track_rad = track_deg * M_PI / 180.0;

  ParsedSentence sentence;
  sentence.sentence_type = "BESTNAVA";
  sentence.velocity = VelocityData{};
  sentence.velocity->east_mps = horizontal_speed * std::sin(track_rad);
  sentence.velocity->north_mps = horizontal_speed * std::cos(track_rad);
  sentence.velocity->up_mps = up_speed;
  sentence.velocity->horizontal_std_mps = horizontal_std;
  sentence.velocity->vertical_std_mps = vertical_std;
  return sentence;
}

}  // namespace mowgli_unicore_gnss

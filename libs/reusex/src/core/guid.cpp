// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/guid.hpp"

#include <boost/uuid/name_generator_sha1.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <exception>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

namespace reusex::core {

std::string generate_guid() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 15);
  std::uniform_int_distribution<> dis2(8, 11);
  std::stringstream ss;
  ss << std::hex;
  for (int i = 0; i < 8; i++)
    ss << dis(gen);
  ss << "-";
  for (int i = 0; i < 4; i++)
    ss << dis(gen);
  ss << "-4";
  for (int i = 0; i < 3; i++)
    ss << dis(gen);
  ss << "-" << dis2(gen);
  for (int i = 0; i < 3; i++)
    ss << dis(gen);
  ss << "-";
  for (int i = 0; i < 12; i++)
    ss << dis(gen);
  return ss.str();
}

std::string deterministic_guid(std::string_view ns_guid,
                               std::string_view name) {
  boost::uuids::uuid ns;
  try {
    ns = boost::uuids::string_generator()(std::string(ns_guid));
  } catch (const std::exception &) {
    throw std::invalid_argument("deterministic_guid: namespace '" +
                                std::string(ns_guid) + "' is not a UUID");
  }
  const boost::uuids::name_generator_sha1 gen(ns);
  return boost::uuids::to_string(gen(name.data(), name.size()));
}

} // namespace reusex::core

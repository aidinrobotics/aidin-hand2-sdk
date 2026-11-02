#pragma once

#include <cstdlib>
#include <cstring>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace aidin_hand2
{

// Minimal JSON reader for the files the SDK takes, an owning DOM
// Every read is bounded by the view, so text that is not NUL terminated is safe

class JsonValue
{
 public:
  enum class Type { Null, Bool, Number, String, Array, Object };

  JsonValue() = default;

  Type type() const { return type_; }
  bool is_object() const { return type_ == Type::Object; }
  bool is_array() const { return type_ == Type::Array; }

  // Direct member of this object, nullptr when absent
  const JsonValue* find(std::string_view key) const
  {
    if (type_ != Type::Object) return nullptr;
    const auto it = members_.find(std::string(key));
    return it == members_.end() ? nullptr : &it->second;
  }

  std::optional<double> as_number() const
  {
    if (type_ == Type::Number) return number_;
    return std::nullopt;
  }
  std::optional<std::string_view> as_string() const
  {
    if (type_ == Type::String) return std::string_view(string_);
    return std::nullopt;
  }

  std::size_t size() const { return type_ == Type::Array ? elements_.size() : 0; }
  const JsonValue* at(std::size_t i) const
  {
    return (type_ == Type::Array && i < elements_.size()) ? &elements_[i] : nullptr;
  }

  // nullopt unless the whole text is one value
  static std::optional<JsonValue> parse(std::string_view text)
  {
    std::size_t pos = 0;
    JsonValue out;
    if (!parse_value(text, pos, out, 0)) return std::nullopt;
    skip_ws(text, pos);
    if (pos != text.size()) return std::nullopt;
    return out;
  }

 private:
  static constexpr int kMaxDepth = 32;

  Type type_{Type::Null};
  bool bool_{false};
  double number_{0.0};
  std::string string_;
  std::vector<JsonValue> elements_;
  std::map<std::string, JsonValue> members_;

  static void skip_ws(std::string_view s, std::size_t& p)
  {
    while (p < s.size() && (s[p] == ' ' || s[p] == '\t' || s[p] == '\n' || s[p] == '\r')) ++p;
  }

  static bool literal(std::string_view s, std::size_t& p, std::string_view word)
  {
    if (s.size() - p < word.size()) return false;
    if (s.compare(p, word.size(), word) != 0) return false;
    p += word.size();
    return true;
  }

  static bool parse_string(std::string_view s, std::size_t& p, std::string& out)
  {
    if (p >= s.size() || s[p] != '"') return false;
    ++p;
    out.clear();
    while (p < s.size()) {
      const char c = s[p];
      if (c == '"') { ++p; return true; }
      if (c == '\\') {
        ++p;
        if (p >= s.size()) return false;
        switch (s[p]) {
          case '"':  out += '"';  ++p; break;
          case '\\': out += '\\'; ++p; break;
          case '/':  out += '/';  ++p; break;
          case 'b':  out += '\b'; ++p; break;
          case 'f':  out += '\f'; ++p; break;
          case 'n':  out += '\n'; ++p; break;
          case 'r':  out += '\r'; ++p; break;
          case 't':  out += '\t'; ++p; break;
          case 'u': {
            // BMP only, a surrogate pair goes through as two code points
            if (s.size() - p < 5) return false;
            unsigned code = 0;
            for (int i = 1; i <= 4; ++i) {
              const char h = s[p + i];
              code <<= 4;
              if (h >= '0' && h <= '9') code |= static_cast<unsigned>(h - '0');
              else if (h >= 'a' && h <= 'f') code |= static_cast<unsigned>(h - 'a' + 10);
              else if (h >= 'A' && h <= 'F') code |= static_cast<unsigned>(h - 'A' + 10);
              else return false;
            }
            p += 5;
            if (code < 0x80) {
              out += static_cast<char>(code);
            } else if (code < 0x800) {
              out += static_cast<char>(0xC0 | (code >> 6));
              out += static_cast<char>(0x80 | (code & 0x3F));
            } else {
              out += static_cast<char>(0xE0 | (code >> 12));
              out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
              out += static_cast<char>(0x80 | (code & 0x3F));
            }
            break;
          }
          default: return false;
        }
        continue;
      }
      out += c;
      ++p;
    }
    return false;
  }

  // The token is copied before strtod, which would otherwise read past the view
  static bool parse_number(std::string_view s, std::size_t& p, double& out)
  {
    const std::size_t begin = p;
    if (p < s.size() && (s[p] == '-' || s[p] == '+')) ++p;
    while (p < s.size() && ((s[p] >= '0' && s[p] <= '9') || s[p] == '.' ||
                            s[p] == 'e' || s[p] == 'E' || s[p] == '-' || s[p] == '+')) {
      ++p;
    }
    const std::size_t len = p - begin;
    if (len == 0 || len >= 64) return false;

    char buffer[64];
    std::memcpy(buffer, s.data() + begin, len);
    buffer[len] = '\0';

    char* end = nullptr;
    const double parsed = std::strtod(buffer, &end);
    if (end != buffer + len) return false;
    out = parsed;
    return true;
  }

  static bool parse_value(std::string_view s, std::size_t& p, JsonValue& out, int depth)
  {
    if (depth > kMaxDepth) return false;
    skip_ws(s, p);
    if (p >= s.size()) return false;

    switch (s[p]) {
      case 'n':
        if (!literal(s, p, "null")) return false;
        out.type_ = Type::Null;
        return true;
      case 't':
        if (!literal(s, p, "true")) return false;
        out.type_ = Type::Bool; out.bool_ = true;
        return true;
      case 'f':
        if (!literal(s, p, "false")) return false;
        out.type_ = Type::Bool; out.bool_ = false;
        return true;
      case '"': {
        std::string str;
        if (!parse_string(s, p, str)) return false;
        out.type_ = Type::String; out.string_ = std::move(str);
        return true;
      }
      case '[': {
        ++p;
        out.type_ = Type::Array;
        out.elements_.clear();
        skip_ws(s, p);
        if (p < s.size() && s[p] == ']') { ++p; return true; }
        while (true) {
          JsonValue element;
          if (!parse_value(s, p, element, depth + 1)) return false;
          out.elements_.push_back(std::move(element));
          skip_ws(s, p);
          if (p >= s.size()) return false;
          if (s[p] == ',') { ++p; continue; }
          if (s[p] == ']') { ++p; return true; }
          return false;
        }
      }
      case '{': {
        ++p;
        out.type_ = Type::Object;
        out.members_.clear();
        skip_ws(s, p);
        if (p < s.size() && s[p] == '}') { ++p; return true; }
        while (true) {
          skip_ws(s, p);
          std::string key;
          if (!parse_string(s, p, key)) return false;
          skip_ws(s, p);
          if (p >= s.size() || s[p] != ':') return false;
          ++p;
          JsonValue member;
          if (!parse_value(s, p, member, depth + 1)) return false;
          out.members_[std::move(key)] = std::move(member);
          skip_ws(s, p);
          if (p >= s.size()) return false;
          if (s[p] == ',') { ++p; continue; }
          if (s[p] == '}') { ++p; return true; }
          return false;
        }
      }
      default:
        out.type_ = Type::Number;
        return parse_number(s, p, out.number_);
    }
  }
};

}  // namespace aidin_hand2

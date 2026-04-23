#include "mowgli_ntrip_client/ntrip_connection.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <utility>

namespace mowgli_ntrip_client
{

namespace
{

bool starts_with(const std::string &value, const std::string &prefix)
{
  return value.rfind(prefix, 0) == 0;
}

bool is_success_status_line(const std::string &line)
{
  return starts_with(line, "ICY 200") || starts_with(line, "HTTP/1.0 200") ||
         starts_with(line, "HTTP/1.1 200");
}

bool is_textual_error_response(const std::string &text)
{
  return starts_with(text, "SOURCETABLE") || starts_with(text, "ERROR") ||
         starts_with(text, "ICY 401") || starts_with(text, "ICY 403") ||
         starts_with(text, "ICY 404") || starts_with(text, "HTTP/1.0 401") ||
         starts_with(text, "HTTP/1.0 403") || starts_with(text, "HTTP/1.0 404") ||
         starts_with(text, "HTTP/1.1 401") || starts_with(text, "HTTP/1.1 403") ||
         starts_with(text, "HTTP/1.1 404");
}

bool looks_like_rtcm3_stream(const std::vector<std::uint8_t> &buffer)
{
  if (buffer.empty())
  {
    return false;
  }

  if (buffer.front() == 0xD3U)
  {
    return true;
  }

  const auto non_ascii_count = std::count_if(buffer.begin(),
                                             buffer.end(),
                                             [](std::uint8_t byte)
                                             {
                                               return byte == 0U || (byte < 0x09U) ||
                                                      (byte > 0x0DU && byte < 0x20U) ||
                                                      byte > 0x7EU;
                                             });

  if (non_ascii_count == 0)
  {
    return false;
  }

  std::string prefix;
  prefix.reserve(std::min<std::size_t>(buffer.size(), 32U));
  for (std::size_t i = 0; i < buffer.size() && i < 32U; ++i)
  {
    const auto byte = buffer[i];
    if (byte == '\r' || byte == '\n' || byte == '\t' || (byte >= 0x20U && byte <= 0x7EU))
    {
      prefix.push_back(static_cast<char>(byte));
    }
    else
    {
      break;
    }
  }

  return !starts_with(prefix, "HTTP/") && !starts_with(prefix, "ICY ") &&
         !starts_with(prefix, "SOURCETABLE") && !starts_with(prefix, "ERROR");
}

std::string extract_text_prefix(const std::vector<std::uint8_t> &buffer)
{
  std::string text;
  text.reserve(buffer.size());

  for (std::uint8_t byte : buffer)
  {
    if (byte == '\r' || byte == '\n' || byte == '\t' || (byte >= 0x20U && byte <= 0x7EU))
    {
      text.push_back(static_cast<char>(byte));
      continue;
    }
    break;
  }

  return text;
}

std::string extract_status_line(const std::string &text)
{
  const auto line_end = text.find_first_of("\r\n");
  if (line_end == std::string::npos)
  {
    return text;
  }
  return text.substr(0, line_end);
}

std::vector<std::uint8_t> extract_initial_payload(const std::vector<std::uint8_t> &buffer,
                                                  const std::string &text_prefix)
{
  const auto header_end = text_prefix.find("\r\n\r\n");
  if (header_end != std::string::npos)
  {
    const auto payload_offset = header_end + 4U;
    if (payload_offset < buffer.size())
    {
      return std::vector<std::uint8_t>(buffer.begin() + static_cast<std::ptrdiff_t>(payload_offset),
                                       buffer.end());
    }
    return {};
  }

  const auto line_end = text_prefix.find_first_of("\r\n");
  if (line_end != std::string::npos)
  {
    std::size_t payload_offset = line_end;
    while (payload_offset < buffer.size() &&
           (buffer[payload_offset] == '\r' || buffer[payload_offset] == '\n'))
    {
      ++payload_offset;
    }

    if (payload_offset < buffer.size())
    {
      return std::vector<std::uint8_t>(buffer.begin() + static_cast<std::ptrdiff_t>(payload_offset),
                                       buffer.end());
    }
  }

  return {};
}

}  // namespace

NtripConnection::NtripConnection(ConnectionOptions options)
    : options_(std::move(options)),
      work_guard_(boost::asio::make_work_guard(io_context_)),
      resolver_(io_context_),
      socket_(io_context_),
      reconnect_timer_(io_context_),
      connect_timer_(io_context_),
      read_watchdog_(io_context_)
{
}

NtripConnection::~NtripConnection()
{
  stop();
}

void NtripConnection::set_data_callback(DataCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  data_callback_ = std::move(callback);
}

void NtripConnection::set_state_callback(StateCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  state_callback_ = std::move(callback);
}

void NtripConnection::start()
{
  if (started_)
  {
    return;
  }

  started_ = true;
  stopped_ = false;
  io_thread_ = std::thread(
      [this]()
      {
        io_context_.run();
      });
  boost::asio::post(io_context_,
                    [this]()
                    {
                      schedule_resolve("starting", false);
                    });
}

void NtripConnection::stop()
{
  if (!started_)
  {
    return;
  }

  boost::asio::post(io_context_,
                    [this]()
                    {
                      stopped_ = true;
                      reconnect_timer_.cancel();
                      connect_timer_.cancel();
                      read_watchdog_.cancel();
                      resolver_.cancel();
                      close_socket();
                      post_state(false, "stopped");
                    });

  work_guard_.reset();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }

  started_ = false;
}

ConnectionStats NtripConnection::stats() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

void NtripConnection::post_state(bool connected, const std::string &state, const std::string &error)
{
  StateCallback callback;

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.connected = connected;
    stats_.last_state = state;
    if (!error.empty())
    {
      stats_.last_error = error;
    }
    else if (connected)
    {
      stats_.last_error.clear();
    }
  }

  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback = state_callback_;
  }

  if (callback)
  {
    callback(connected, state);
  }
}

void NtripConnection::schedule_resolve(const std::string &reason, bool count_reconnect)
{
  if (stopped_)
  {
    return;
  }

  close_socket();
  resolver_.cancel();
  connect_timer_.cancel();
  read_watchdog_.cancel();
  handshake_complete_ = false;
  response_buffer_.clear();

  if (count_reconnect)
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.reconnect_count;
  }

  post_state(false, reason);
  reconnect_timer_.expires_after(std::chrono::milliseconds(options_.reconnect_delay_ms));
  reconnect_timer_.async_wait(
      [self = shared_from_this()](const boost::system::error_code &ec)
      {
        if (!ec)
        {
          self->do_resolve();
        }
      });
}

void NtripConnection::do_resolve()
{
  if (stopped_)
  {
    return;
  }

  post_state(false, "resolving");
  resolver_.async_resolve(options_.host,
                          std::to_string(options_.port),
                          [self = shared_from_this()](const boost::system::error_code &ec,
                                                      tcp::resolver::results_type endpoints)
                          {
                            if (ec)
                            {
                              self->post_state(false, "resolve_failed", ec.message());
                              self->schedule_resolve("reconnecting", true);
                              return;
                            }

                            self->do_connect(endpoints);
                          });
}

void NtripConnection::start_connect_timeout()
{
  connect_timer_.expires_after(std::chrono::milliseconds(options_.connect_timeout_ms));
  connect_timer_.async_wait(
      [self = shared_from_this()](const boost::system::error_code &ec)
      {
        if (ec || self->stopped_ || self->handshake_complete_)
        {
          return;
        }

        self->post_state(false, "connect_timeout", "connection timed out");
        self->close_socket();
        self->schedule_resolve("reconnecting", true);
      });
}

void NtripConnection::do_connect(const tcp::resolver::results_type &endpoints)
{
  if (stopped_)
  {
    return;
  }

  post_state(false, "connecting");
  start_connect_timeout();
  boost::asio::async_connect(socket_,
                             endpoints,
                             [self = shared_from_this()](const boost::system::error_code &ec,
                                                         const tcp::endpoint &)
                             {
                               if (ec)
                               {
                                 self->post_state(false, "connect_failed", ec.message());
                                 self->schedule_resolve("reconnecting", true);
                                 return;
                               }

                               self->do_handshake();
                             });
}

void NtripConnection::do_handshake()
{
  const auto request = build_request(options_);
  post_state(false, "handshake");

  boost::asio::async_write(
      socket_,
      boost::asio::buffer(request),
      [self = shared_from_this()](const boost::system::error_code &ec, std::size_t)
      {
        if (ec)
        {
          self->post_state(false, "handshake_write_failed", ec.message());
          self->schedule_resolve("reconnecting", true);
          return;
        }

        self->socket_.async_read_some(
            boost::asio::buffer(self->read_buffer_),
            [self](const boost::system::error_code &read_ec, std::size_t bytes_read)
            {
              if (read_ec)
              {
                self->post_state(false, "handshake_read_failed", read_ec.message());
                self->schedule_resolve("reconnecting", true);
                return;
              }

              if (bytes_read == 0)
              {
                self->post_state(false, "invalid_response", "empty handshake response");
                self->schedule_resolve("reconnecting", true);
                return;
              }

              std::vector<std::uint8_t> initial_chunk(self->read_buffer_.begin(),
                                                      self->read_buffer_.begin() +
                                                          static_cast<std::ptrdiff_t>(bytes_read));

              if (looks_like_rtcm3_stream(initial_chunk))
              {
                self->handshake_complete_ = true;
                self->connect_timer_.cancel();
                self->post_state(true, "streaming");
                self->start_read_watchdog();

                {
                  std::lock_guard<std::mutex> lock(self->stats_mutex_);
                  self->stats_.bytes_received += initial_chunk.size();
                }

                DataCallback callback;
                {
                  std::lock_guard<std::mutex> lock(self->callback_mutex_);
                  callback = self->data_callback_;
                }

                if (callback)
                {
                  callback(initial_chunk);
                }

                self->do_read_stream();
                return;
              }

              const std::string text_prefix = extract_text_prefix(initial_chunk);
              const std::string status_line = extract_status_line(text_prefix);

              if (is_textual_error_response(text_prefix) || is_textual_error_response(status_line))
              {
                self->post_state(false,
                                 "caster_rejected",
                                 status_line.empty() ? text_prefix : status_line);
                self->schedule_resolve("reconnecting", true);
                return;
              }

              if (!is_success_status_line(status_line))
              {
                self->post_state(false,
                                 "invalid_response",
                                 status_line.empty() ? "unrecognized handshake response"
                                                     : status_line);
                self->schedule_resolve("reconnecting", true);
                return;
              }

              self->handshake_complete_ = true;
              self->connect_timer_.cancel();
              self->post_state(true, "streaming");
              self->start_read_watchdog();

              auto payload = extract_initial_payload(initial_chunk, text_prefix);
              if (!payload.empty())
              {
                {
                  std::lock_guard<std::mutex> lock(self->stats_mutex_);
                  self->stats_.bytes_received += payload.size();
                }

                DataCallback callback;
                {
                  std::lock_guard<std::mutex> lock(self->callback_mutex_);
                  callback = self->data_callback_;
                }

                if (callback)
                {
                  callback(payload);
                }
              }

              self->do_read_stream();
            });
      });
}

void NtripConnection::start_read_watchdog()
{
  read_watchdog_.expires_after(std::chrono::milliseconds(options_.read_timeout_ms));
  read_watchdog_.async_wait(
      [self = shared_from_this()](const boost::system::error_code &ec)
      {
        if (ec || self->stopped_ || !self->handshake_complete_)
        {
          return;
        }

        self->post_state(false, "read_timeout", "RTCM stream timed out");
        self->schedule_resolve("reconnecting", true);
      });
}

void NtripConnection::do_read_stream()
{
  socket_.async_read_some(
      boost::asio::buffer(read_buffer_),
      [self = shared_from_this()](const boost::system::error_code &ec, std::size_t bytes_read)
      {
        if (ec)
        {
          self->post_state(false, "stream_failed", ec.message());
          self->schedule_resolve("reconnecting", true);
          return;
        }

        self->start_read_watchdog();

        {
          std::lock_guard<std::mutex> lock(self->stats_mutex_);
          self->stats_.bytes_received += bytes_read;
        }

        DataCallback callback;
        {
          std::lock_guard<std::mutex> lock(self->callback_mutex_);
          callback = self->data_callback_;
        }

        if (callback && bytes_read > 0)
        {
          callback(std::vector<std::uint8_t>(self->read_buffer_.begin(),
                                             self->read_buffer_.begin() +
                                                 static_cast<std::ptrdiff_t>(bytes_read)));
        }

        self->do_read_stream();
      });
}

void NtripConnection::close_socket()
{
  boost::system::error_code ignored_ec;
  if (socket_.is_open())
  {
    socket_.shutdown(tcp::socket::shutdown_both, ignored_ec);
    socket_.close(ignored_ec);
  }
}

std::string NtripConnection::build_request(const ConnectionOptions &options)
{
  std::ostringstream stream;
  stream << "GET " << normalize_mountpoint(options.mountpoint) << " HTTP/1.0\r\n";
  stream << "Host: " << options.host << ":" << options.port << "\r\n";
  stream << "User-Agent: NTRIP " << options.user_agent << "\r\n";
  stream << "Ntrip-Version: Ntrip/2.0\r\n";
  stream << "Accept: */*\r\n";

  if (!options.username.empty() || !options.password.empty())
  {
    stream << "Authorization: Basic " << base64_encode(options.username + ":" + options.password)
           << "\r\n";
  }

  stream << "\r\n";
  return stream.str();
}

std::string NtripConnection::normalize_mountpoint(const std::string &mountpoint)
{
  if (mountpoint.empty())
  {
    return "/";
  }

  if (mountpoint.front() == '/')
  {
    return mountpoint;
  }

  return "/" + mountpoint;
}

std::string NtripConnection::base64_encode(const std::string &input)
{
  static constexpr char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string output;
  output.reserve(((input.size() + 2U) / 3U) * 4U);

  std::uint32_t value = 0;
  int value_bits = -6;
  for (unsigned char c : input)
  {
    value = (value << 8) + c;
    value_bits += 8;
    while (value_bits >= 0)
    {
      output.push_back(table[(value >> value_bits) & 0x3F]);
      value_bits -= 6;
    }
  }

  if (value_bits > -6)
  {
    output.push_back(table[((value << 8) >> (value_bits + 8)) & 0x3F]);
  }

  while (output.size() % 4U != 0U)
  {
    output.push_back('=');
  }

  return output;
}

}  // namespace mowgli_ntrip_client

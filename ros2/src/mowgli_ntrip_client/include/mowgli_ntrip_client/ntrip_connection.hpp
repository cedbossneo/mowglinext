#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace mowgli_ntrip_client
{

struct ConnectionOptions
{
  std::string host;
  std::uint16_t port{2101};
  std::string mountpoint;
  std::string username;
  std::string password;
  std::string user_agent{"mowgli_ntrip_client/0.1"};
  std::size_t reconnect_delay_ms{5000};
  std::size_t connect_timeout_ms{5000};
  std::size_t read_timeout_ms{15000};
};

struct ConnectionStats
{
  bool connected{false};
  std::uint64_t bytes_received{0};
  std::uint64_t reconnect_count{0};
  std::string last_state{"idle"};
  std::string last_error;
};

class NtripConnection : public std::enable_shared_from_this<NtripConnection>
{
public:
  using DataCallback = std::function<void(const std::vector<std::uint8_t> &)>;
  using StateCallback = std::function<void(bool, const std::string &)>;

  explicit NtripConnection(ConnectionOptions options);
  ~NtripConnection();

  void set_data_callback(DataCallback callback);
  void set_state_callback(StateCallback callback);

  void start();
  void stop();

  ConnectionStats stats() const;

private:
  using tcp = boost::asio::ip::tcp;

  void post_state(bool connected, const std::string &state, const std::string &error = "");
  void schedule_resolve(const std::string &reason, bool count_reconnect);
  void do_resolve();
  void start_connect_timeout();
  void do_connect(const tcp::resolver::results_type &endpoints);
  void do_handshake();
  void start_read_watchdog();
  void do_read_stream();
  void close_socket();

  static std::string build_request(const ConnectionOptions &options);
  static std::string normalize_mountpoint(const std::string &mountpoint);
  static std::string base64_encode(const std::string &input);

  ConnectionOptions options_;

  mutable std::mutex callback_mutex_;
  DataCallback data_callback_;
  StateCallback state_callback_;

  mutable std::mutex stats_mutex_;
  ConnectionStats stats_;

  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  tcp::resolver resolver_;
  tcp::socket socket_;
  boost::asio::steady_timer reconnect_timer_;
  boost::asio::steady_timer connect_timer_;
  boost::asio::steady_timer read_watchdog_;
  std::thread io_thread_;

  std::array<std::uint8_t, 4096> read_buffer_{};
  std::string response_buffer_;

  bool started_{false};
  bool stopped_{false};
  bool handshake_complete_{false};
};

}  // namespace mowgli_ntrip_client

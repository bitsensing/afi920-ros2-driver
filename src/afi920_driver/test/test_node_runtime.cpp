// Copyright (c) 2026 bitsensing Inc. All rights reserved.
// Runtime behavior that needs no sensor: parameter changes and TCP reconnect.
// Everything here talks to 127.0.0.1 only.
#include <gtest/gtest.h>

#include "afi920_driver/afi920_node.hpp"
#include "afi920_driver/transport/tcp_stream_client.hpp"

#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

using afi920_driver::Afi920Node;
using afi920_driver::TcpStreamClient;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

namespace {

// Loopback listener on a free port. Listen() can be called after construction
// so a client sees the port refused first.
class LoopbackListener {
public:
    LoopbackListener()
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        socklen_t len = sizeof(addr);
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
        port_ = ntohs(addr.sin_port);
    }
    ~LoopbackListener()
    {
        if (peer_ >= 0) ::close(peer_);
        if (fd_ >= 0) ::close(fd_);
    }
    uint16_t port() const { return port_; }
    void Listen() { ::listen(fd_, 1); }
    bool Accept()
    {
        peer_ = ::accept(fd_, nullptr, nullptr);
        return peer_ >= 0;
    }
    void Send(const std::vector<uint8_t>& bytes)
    {
        ::send(peer_, bytes.data(), bytes.size(), 0);
    }

private:
    int fd_ = -1;
    int peer_ = -1;
    uint16_t port_ = 0;
};

// SOME/IP-like frame: MessageID (4) + Length (4) + Length bytes.
std::vector<uint8_t> Frame(uint32_t length)
{
    std::vector<uint8_t> f(8 + length, 0x5A);
    f[4] = static_cast<uint8_t>(length >> 24);
    f[5] = static_cast<uint8_t>(length >> 16);
    f[6] = static_cast<uint8_t>(length >> 8);
    f[7] = static_cast<uint8_t>(length);
    return f;
}

}  // namespace

TEST(TcpStreamClientRuntime, FirstConnectFailureIsRetriedUntilTheServerAppears)
{
    LoopbackListener server;  // bound, not listening: connects are refused
    TcpStreamClient client("127.0.0.1", server.port(), 4096, 500, 20);
    ASSERT_LT(client.Connect(), 0);
    EXPECT_FALSE(client.IsOpen());

    std::vector<uint8_t> buf(4096);
    // First retry after 1 s fails and doubles the backoff.
    EXPECT_LT(client.Receive(buf.data(), buf.size()), 0);
    EXPECT_EQ(client.NextReconnectDelayMs(), 2000);

    server.Listen();
    std::thread accept_thread([&server] { server.Accept(); });
    EXPECT_EQ(client.Receive(buf.data(), buf.size()), 0);  // reconnected
    accept_thread.join();
    EXPECT_TRUE(client.IsOpen());
    EXPECT_EQ(client.NextReconnectDelayMs(), 1000);

    server.Send(Frame(4));
    int n = 0;
    for (int i = 0; i < 50 && n == 0; ++i) n = client.Receive(buf.data(), buf.size());
    EXPECT_EQ(n, 12);
}

TEST(TcpStreamClientRuntime, CloseInterruptsTheBackoffWait)
{
    LoopbackListener server;
    TcpStreamClient client("127.0.0.1", server.port(), 4096, 500, 20);
    ASSERT_LT(client.Connect(), 0);

    std::atomic<bool> returned{false};
    std::thread rx([&] {
        std::vector<uint8_t> buf(4096);
        client.Receive(buf.data(), buf.size());  // sleeps the 1 s backoff
        returned = true;
    });
    std::this_thread::sleep_for(milliseconds(100));
    const auto t0 = steady_clock::now();
    client.Close();
    rx.join();
    const auto waited = std::chrono::duration_cast<milliseconds>(steady_clock::now() - t0);
    EXPECT_TRUE(returned);
    EXPECT_LT(waited.count(), 500);
}

class NodeParameterTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(NodeParameterTest, OnlyDynamicParametersChangeAtRuntime)
{
    // Loopback sensor: Config API and streams are refused and never leave the host.
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("sensor_ip", "127.0.0.1"),
        rclcpp::Parameter("host_ip", "127.0.0.1"),
        rclcpp::Parameter("min_range", 2.5),
    });
    auto node = std::make_shared<Afi920Node>(options);
    EXPECT_DOUBLE_EQ(node->get_parameter("min_range").as_double(), 2.5);

    auto ok = node->set_parameter(rclcpp::Parameter("min_range", 1.0));
    EXPECT_TRUE(ok.successful);
    EXPECT_DOUBLE_EQ(node->get_parameter("min_range").as_double(), 1.0);

    auto rejected = node->set_parameter(rclcpp::Parameter("sensor_ip", "127.0.0.2"));
    EXPECT_FALSE(rejected.successful);
    EXPECT_NE(rejected.reason.find("sensor_ip cannot be changed at runtime"), std::string::npos);
    EXPECT_EQ(node->get_parameter("sensor_ip").as_string(), "127.0.0.1");

    // A request mixing both kinds is rejected as a whole.
    auto mixed = node->set_parameters_atomically({
        rclcpp::Parameter("max_range", 50.0),
        rclcpp::Parameter("transport_mode", "udp"),
    });
    EXPECT_FALSE(mixed.successful);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_range").as_double(), 300.0);

    const auto t0 = steady_clock::now();
    node.reset();
    const auto waited = std::chrono::duration_cast<milliseconds>(steady_clock::now() - t0);
    EXPECT_LT(waited.count(), 1000);  // retry threads stop promptly
}

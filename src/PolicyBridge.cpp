#include "PolicyBridge.h"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/imgproc.hpp>

namespace uuv
{
namespace
{
constexpr uint32_t kStateFloats = 29;
constexpr uint32_t kImageW = 320;
constexpr uint32_t kImageH = 240;
constexpr uint32_t kImageBytes = kImageW * kImageH * 3;

void appendBytes(std::vector<uint8_t>& buf, const void* src, size_t n)
{
    const size_t off = buf.size();
    buf.resize(off + n);
    if (n > 0)
        std::memcpy(buf.data() + off, src, n);
}

void writeU32(std::vector<uint8_t>& buf, uint32_t v)
{
    appendBytes(buf, &v, sizeof(v));
}

void writeFloats(std::vector<uint8_t>& buf, const float* v, size_t n)
{
    appendBytes(buf, v, n * sizeof(float));
}
}  // namespace

PolicyBridge::PolicyBridge(int port)
{
    listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd_ < 0)
        throw std::runtime_error("policy bridge: socket() failed");

    int one = 1;
    ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(listenFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("policy bridge: bind() failed on port " + std::to_string(port));
    if (::listen(listenFd_, 1) < 0)
        throw std::runtime_error("policy bridge: listen() failed");
    ::fcntl(listenFd_, F_SETFL, ::fcntl(listenFd_, F_GETFL, 0) | O_NONBLOCK);
    std::printf("[policy] listening on 127.0.0.1:%d\n", port);
    std::fflush(stdout);
}

PolicyBridge::~PolicyBridge()
{
    closeClient();
    if (listenFd_ >= 0)
        ::close(listenFd_);
}

void PolicyBridge::closeClient()
{
    if (client_ >= 0)
    {
        ::close(client_);
        client_ = -1;
        awaitingAck_ = false;
        std::printf("[policy] client disconnected\n");
        std::fflush(stdout);
    }
}

void PolicyBridge::tryAccept()
{
    if (client_ >= 0 || listenFd_ < 0)
        return;
    sockaddr_in addr{};
    socklen_t len = sizeof(addr);
    const int fd = ::accept(listenFd_, reinterpret_cast<sockaddr*>(&addr), &len);
    if (fd < 0)
        return;
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    timeval snd{};
    snd.tv_sec = 0;
    snd.tv_usec = 100000;  // 100 ms send timeout; the send loop retries on EAGAIN
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
    timeval rcv{};
    rcv.tv_sec = 0;
    rcv.tv_usec = 1000;  // non-blocking-ish reads so the control loop never stalls
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rcv, sizeof(rcv));
    client_ = fd;
    rxBuffer_.clear();
    std::printf("[policy] client connected\n");
    std::fflush(stdout);
}

bool PolicyBridge::sendFrame(const std::array<float, 29>& state, const cv::Mat& egoBgr,
                             const cv::Mat& wristBgr, double goalDistance,
                             const std::array<double, 3>& position, double goalBearing,
                             double routeDistance, double routeBearing)
{
    tryAccept();
    if (client_ < 0)
        return false;
    // Backpressure: the policy is slow (~0.5 s per inference), so do not queue more
    // frames than it consumes. One frame in flight at a time.
    if (awaitingAck_)
        return true;

    cv::Mat ego;
    cv::Mat wrist;
    if (!egoBgr.empty())
        cv::resize(egoBgr, ego, cv::Size(kImageW, kImageH), 0, 0, cv::INTER_LINEAR);
    if (!wristBgr.empty())
        cv::resize(wristBgr, wrist, cv::Size(kImageW, kImageH), 0, 0, cv::INTER_LINEAR);

    std::vector<uint8_t> payload;
    payload.resize(kStateFloats * sizeof(float));
    std::memcpy(payload.data(), state.data(), kStateFloats * sizeof(float));
    writeU32(payload, ego.empty() ? 0u : kImageBytes);
    if (!ego.empty())
    {
        std::vector<uint8_t> rgb(kImageBytes);
        // Stonefish cameras hand us BGR mats; the dataset videos are RGB.
        cv::Mat rgbMat(kImageH, kImageW, CV_8UC3, rgb.data());
        cv::cvtColor(ego, rgbMat, cv::COLOR_BGR2RGB);
        payload.insert(payload.end(), rgb.begin(), rgb.end());
    }
    writeU32(payload, wrist.empty() ? 0u : kImageBytes);
    if (!wrist.empty())
    {
        std::vector<uint8_t> rgb(kImageBytes);
        cv::Mat rgbMat(kImageH, kImageW, CV_8UC3, rgb.data());
        cv::cvtColor(wrist, rgbMat, cv::COLOR_BGR2RGB);
        payload.insert(payload.end(), rgb.begin(), rgb.end());
    }

    const float dist = static_cast<float>(goalDistance);
    const float dists[3] = {dist, dist, dist};
    writeFloats(payload, dists, 3);
    const float pos[3] = {static_cast<float>(position[0]), static_cast<float>(position[1]),
                          static_cast<float>(position[2])};
    writeFloats(payload, pos, 3);
    // Goal direction relative to the robot heading (rad, positive = right). Appended after the
    // USIMNavTest trailer so older clients can ignore it.
    const float tail[3] = {static_cast<float>(goalBearing), static_cast<float>(routeDistance),
                           static_cast<float>(routeBearing)};
    writeFloats(payload, tail, 3);

    const uint32_t total = static_cast<uint32_t>(payload.size());
    std::vector<uint8_t> packet;
    packet.reserve(sizeof(total) + payload.size());
    writeU32(packet, total);
    packet.insert(packet.end(), payload.begin(), payload.end());

    size_t off = 0;
    int stalls = 0;
    while (off < packet.size())
    {
        const ssize_t n = ::send(client_, packet.data() + off, packet.size() - off, MSG_NOSIGNAL);
        if (n > 0)
        {
            off += static_cast<size_t>(n);
            stalls = 0;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            // The policy is busy (inference) and not draining the socket. Keep the frame
            // intact and retry, otherwise the client's length-prefixed stream desyncs.
            if (++stalls > 50)
            {
                closeClient();
                return false;
            }
            continue;
        }
        closeClient();
        return false;
    }
    awaitingAck_ = true;
    return true;
}

void PolicyBridge::drain()
{
    if (client_ < 0)
        return;
    uint8_t tmp[256];
    for (;;)
    {
        const ssize_t n = ::recv(client_, tmp, sizeof(tmp), 0);
        if (n > 0)
        {
            rxBuffer_.insert(rxBuffer_.end(), tmp, tmp + n);
            continue;
        }
        if (n == 0)
        {
            closeClient();
            return;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            break;
        if (errno == EINTR)
            continue;
        closeClient();
        return;
    }

    const size_t cmdBytes = 8 * sizeof(float);
    while (rxBuffer_.size() >= cmdBytes)
    {
        std::array<double, 8> pwm{};
        float values[8];
        std::memcpy(values, rxBuffer_.data(), sizeof(values));
        for (int i = 0; i < 8; ++i)
            pwm[i] = static_cast<double>(values[i]);
        rxBuffer_.erase(rxBuffer_.begin(), rxBuffer_.begin() + cmdBytes);
        commands_.push_back(pwm);
        awaitingAck_ = false;
        if (commands_.size() > 50)
            commands_.pop_front();
    }
}

bool PolicyBridge::popCommand(std::array<double, 8>& pwm)
{
    if (client_ >= 0)
        drain();
    if (commands_.empty())
        return false;
    pwm = commands_.front();
    commands_.pop_front();
    return true;
}
}  // namespace uuv

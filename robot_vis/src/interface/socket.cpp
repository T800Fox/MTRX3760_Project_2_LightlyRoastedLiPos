#include "socket.hpp"
#include <iostream>
#include <chrono>
#include <thread>

TCP_Connection::TCP_Connection() {
    server = nullptr;
    client = nullptr;
}

TCP_Connection::~TCP_Connection() {
    if (server) {
        server->stop();  // safely stop the server first
    }
    if (client) {
        client->stop();
    }
    if (loop) {
        loop->stop();    // stop the event loop
    }
    // No need to manually delete anything – smart pointers handle it
}


void TCP_Connection::start_loop() {
    if (!loop) loop = std::make_shared<hv::EventLoop>();
    std::thread([this] {
        loop->run();
    }).detach();
}

// Helper map for accumulating partial messages per connection
std::unordered_map<int, std::string> buffer_accum_map;

int TCP_Connection::create_server(int port) {
    server = std::make_shared<hv::TcpServer>();

    int listenfd = server->createsocket(port);
    if (listenfd < 0) {
        std::cerr << "Failed to create server socket on port " << port << std::endl;
        return 0;
    }

    server->onConnection = [this](const hv::SocketChannelPtr& ch) {
        std::string peer = ch->peeraddr();
        if (ch->isConnected()) {
            std::cout << peer << " connected! fd=" << ch->fd() << "\n";
            channel = ch;
            buffer_accum_map[ch->fd()] = "";
        } else {
            std::cout << peer << " disconnected! fd=" << ch->fd() << "\n";
            buffer_accum_map.erase(ch->fd());
        }
    };

    server->onMessage = [this](const hv::SocketChannelPtr& ch, hv::Buffer* buf) {
        std::string& buffer_accum = buffer_accum_map[ch->fd()];
        buffer_accum.append((char*)buf->data(), buf->size());

        size_t pos;
        while ((pos = buffer_accum.find('\n')) != std::string::npos) {
            std::string msg = buffer_accum.substr(0, pos);
            buffer_accum.erase(0, pos + 1);

            try {
                json j = json::parse(msg);
                std::lock_guard<std::mutex> lock(map_mutex);
                last_packets[j["type"]] = j;
            } catch (...) {
                std::cerr << "Invalid JSON received (partial or corrupted)\n";
            }
        }
    };

    server->setThreadNum(2);
    server->start();

    std::cout << "Server listening on port " << port << "\n";
    start_loop();
    return 1;
}

int TCP_Connection::connect_to_server(const std::string& ip, int port) {
    client = std::make_shared<hv::TcpClient>();

    int connfd = client->createsocket(port);
    if (connfd < 0) {
        std::cerr << "Failed to create client socket to port " << port << std::endl;
        return 0;
    }

    client->onConnection = [this](const hv::SocketChannelPtr& ch) {
        std::string peer = ch->peeraddr();
        if (ch->isConnected()) {
            std::cout << "Connected to " << peer << " fd=" << ch->fd() << "\n";
            channel = ch;
            buffer_accum_map[ch->fd()] = "";
        } else {
            std::cout << "Disconnected from " << peer << "\n";
            buffer_accum_map.erase(ch->fd());
        }
    };

    client->onMessage = [this](const hv::SocketChannelPtr& ch, hv::Buffer* buf) {
        std::string& buffer_accum = buffer_accum_map[ch->fd()];
        buffer_accum.append((char*)buf->data(), buf->size());

        size_t pos;
        while ((pos = buffer_accum.find('\n')) != std::string::npos) {
            std::string msg = buffer_accum.substr(0, pos);
            buffer_accum.erase(0, pos + 1);

            try {
                json j = json::parse(msg);
                std::lock_guard<std::mutex> lock(map_mutex);
                last_packets[j["type"]] = j;
            } catch (...) {
                std::cerr << "Invalid JSON received (partial or corrupted)\n";
            }
        }
    };

    client->start();
    std::cout << "Client started.\n";

    start_loop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return 1;
}

int TCP_Connection::send_json(const json& j) {
    if (!channel) {
        std::cerr << "No active connection.\n";
        return 0;
    }

    std::string data = j.dump() + "\n";
    channel->write(data);
    return 1;
}

json TCP_Connection::query_prev_packet(const std::string& key, bool clear) {
    std::lock_guard<std::mutex> lock(map_mutex);
    if (last_packets.find(key) == last_packets.end()) return json();
    json temp = last_packets[key];
    if (clear) last_packets.erase(key);
    return temp;
}

#pragma once
#include <TcpServer.h>
#include <TcpClient.h>
#include <EventLoop.h>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <mutex>
#include <string>
#include <thread>
#include <iostream>

using json = nlohmann::json;

class TCP_Connection {
public:
    TCP_Connection();
    ~TCP_Connection();

    int create_server(int port);
    int connect_to_server(const std::string& ip, int port);
    int send_json(const json& j);
    json query_prev_packet(const std::string& key, bool clear=false);

private:
    std::shared_ptr<hv::TcpServer> server;
    std::shared_ptr<hv::TcpClient> client;
    std::shared_ptr<hv::SocketChannel> channel;  // shared because libhv may also hold references
    std::shared_ptr<hv::EventLoop> loop;

    std::unordered_map<std::string, json> last_packets;
    std::mutex map_mutex;

    void start_loop();
};

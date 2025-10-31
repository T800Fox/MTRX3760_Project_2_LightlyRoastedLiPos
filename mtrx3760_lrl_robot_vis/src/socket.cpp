#include "mtrx3760_lrl_robot_vis/socket.hpp"


std::mutex data_mutex;
std::string latest_response;
int sock;

// background receiver
void recv_thread() {
    std::string buffer;
    char buf[512];
    while (true) {
        int n = recv(sock, buf, sizeof(buf)-1, 0);
        if (n <= 0) break; // closed or error
        buf[n] = 0;
        buffer += buf;

        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string msg = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);
            std::lock_guard<std::mutex> lock(data_mutex);
            latest_response = msg; // store last received JSON
        }
    }
}

// send helper
void send_json(const std::string& msg) {
    std::string payload = msg + "\n";
    send(sock, payload.c_str(), payload.size(), 0);
}
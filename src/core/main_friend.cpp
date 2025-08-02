// src/core/main_friend.cpp - Arkadaşınız için
#include "engine.h"
#include "../common/logger.h"
#include <stdexcept>
#include <iostream>

int main() {
    try {
        // Arkadaşınızın IP'si: 192.168.1.5
        // Sizin IP'niz: 192.168.1.254
        const std::string local_ip = "192.168.1.5";
        const std::string remote_ip = "192.168.1.254";
        const uint16_t base_port = 60000;
        const size_t num_tunnels = 5;

        std::cout << "Nova Engine V3 - UDP Video Chat (Friend)" << std::endl;
        std::cout << "Local IP: " << local_ip << std::endl;
        std::cout << "Remote IP: " << remote_ip << std::endl;
        std::cout << "Base Port: " << base_port << std::endl;
        std::cout << "Tunnels: " << num_tunnels << std::endl;
        std::cout << "Starting engine..." << std::endl;

        Engine nova_engine(remote_ip, base_port, num_tunnels);
        nova_engine.run();

    } catch (const std::runtime_error& e) {
        LOG_ERROR("A critical error occurred: ", e.what());
        return 1;
    } catch (...) {
        LOG_ERROR("An unknown critical error occurred.");
        return 1;
    }

    return 0;
} 
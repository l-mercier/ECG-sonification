#include "NetworkManager.h"

NetworkManager::NetworkManager() : destIP(nullptr), destPort(0) {
}

bool NetworkManager::connectToWiFi(const char* ssid, const char* password,
                                    IPAddress localIP, IPAddress gateway,
                                    IPAddress subnet, IPAddress dns1, IPAddress dns2) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    
    WiFi.config(localIP, gateway, subnet, dns1, dns2);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nFailed to connect to WiFi.");
        return false;
    }
}

bool NetworkManager::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

IPAddress NetworkManager::getLocalIP() {
    return WiFi.localIP();
}

void NetworkManager::setDestination(const char* ip, int port) {
    destIP = ip;
    destPort = port;
}

void NetworkManager::beginOSC(int localPort) {
    udp.begin(localPort);
    Serial.print("OSC listening on port: ");
    Serial.println(localPort);
}

OSCMessage* NetworkManager::createOSCMessage(const char* address) {
    return new OSCMessage(address);
}

void NetworkManager::sendOSCMessage(OSCMessage* msg) {
    if (!destIP || destPort == 0) {
        Serial.println("OSC destination not set!");
        delete msg;
        return;
    }
    
    udp.beginPacket(destIP, destPort);
    msg->send(udp);
    udp.endPacket();
    msg->empty();
    delete msg;
}

void NetworkManager::sendOSC(const char* address, int32_t value1, 
                              float value2, float value3, int32_t value4) {
    if (!destIP || destPort == 0) {
        Serial.println("OSC destination not set!");
        return;
    }
    
    OSCMessage msg(address);
    msg.add(value1);
    msg.add(value2);
    msg.add(value3);
    msg.add(value4);
    
    udp.beginPacket(destIP, destPort);
    msg.send(udp);
    udp.endPacket();
    msg.empty();
}

// NetworkManagerConfigurator implementation
void NetworkManagerConfigurator::setOSCDestination(NetworkManager& nm, 
                                                    const char* ip, int port) {
    nm.setDestination(ip, port);
}
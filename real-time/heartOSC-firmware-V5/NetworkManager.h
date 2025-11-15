#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <IPAddress.h>

class NetworkManager {
public:
    NetworkManager();
    
    // WiFi methods
    bool connectToWiFi(const char* ssid, const char* password,
                       IPAddress localIP, IPAddress gateway, 
                       IPAddress subnet, IPAddress dns1, IPAddress dns2);
    bool isConnected();
    IPAddress getLocalIP();
    
    // OSC methods
    void beginOSC(int localPort);
    
    // Flexible OSC message builder - USE THIS FOR DYNAMIC MESSAGES
    OSCMessage* createOSCMessage(const char* address);
    void sendOSCMessage(OSCMessage* msg);
    
    // Legacy method for backward compatibility (fixed 4 parameters)
    void sendOSC(const char* address, int32_t value1, float value2, 
                 float value3, int32_t value4);
    
private:
    WiFiUDP udp;
    const char* destIP;
    int destPort;
    
    void setDestination(const char* ip, int port);
    
    friend class NetworkManagerConfigurator;
};

// Helper class to configure destination after construction
class NetworkManagerConfigurator {
public:
    static void setOSCDestination(NetworkManager& nm, const char* ip, int port);
};

#endif // NETWORK_MANAGER_H
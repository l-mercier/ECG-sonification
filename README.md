# Heartrate-OSC project

This is the firmware for an Arduino Nano-based project that uses an infrared heartrate DIY sensor (MAX3015 by Sparkfun), sends the data via OSC (wifi) to a network and can be read by a software. The application here was to modulate sound in real-time with the heart signal.

### quick start

* Download and install Arduino IDE from [energia.nu/download/](http://energia.nu/download/).
* install required libraries (<WiFi.h>, <WiFiUdp.h>, <OSCMessage.h>, <Wire.h>, "MAX30105.h")


#### Network config

Inside the .ino file, you'll find a series of parameters that are used to set the network to which to ESP-32 will connect, as well as OSC ports and destination.

// WiFi configuration

const char* ssid = "your-ssid";
const char* password = "your-password";

IPAddress local_IP(10, 10, 0, 150);     // Must be inside DHCP range and different from destIP
IPAddress gateway(10, 10, 0, 254);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(1, 1, 1, 1);

// OSC destination
const char* destIP = "10.10.0.1";       // Receiving device IP (adjust if needed)
const int destPort = 8000;              // Receiving port
const int localPort = 12345;            // Port to send from

#### upload

* Upload code to your ESP-32 board


### going further

You can receive the data in a wide range of OSC enabled apps : 
Max/MSP, MAx4live, TouchDesigner, to only name those...

You'll find example MAX & M4L patches attached.

For more information, please visit : 


### credits

by Léo Mercier
heavily inspired by Emmanuel Flety's Riot, developped at Ircam.

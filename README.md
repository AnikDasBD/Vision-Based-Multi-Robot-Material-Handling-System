# Vision-Based Multi-Robot Material Handling

A centralized vision-based multi-robot system for coordinated material handling using a handheld camera, ArUco-based localization, A* path planning, and electromagnetic object handling.

## Features

* Two differential-drive mobile robots
* Vision-based localization using ArUco markers
* Handheld camera used as the sole sensing source
* A* path planning
* Greedy nearest-object task allocation
* Geometric inter-robot collision detection and avoidance
* P-controller-based motion control
* HTTP POST and JSON communication between server and robots
* Electromagnetic object handling
* Watchdog-based motor safety
* Software and physical emergency stop

## Hardware

Each robot uses:

* ESP8266 NodeMCU
* TB6612FNG motor driver
* 2 × N20 DC geared motors
* P20/15 electromagnet
* N-channel MOSFET
* 2 × 18650 Li-ion cells in series (7.4 V nominal)
* 5 V buck converter
* Physical emergency stop switch

## Software

**Server:** Python, OpenCV, ArUco, A*, HTTP/JSON

**Robot:** C/C++ on ESP8266

All sensing is performed through the camera. No additional sensors are installed on the robots.

## Network Configuration

Each ESP8266 has a unique MAC address. Before uploading the main firmware:

1. First upload the MAC-address checking program on ESP8266 to obtain the MAC address.
2. Assign that MAC address to a static IP address in the Wi-Fi router.
3. Set the corresponding static IP address in the server configuration file.
4. Upload the firmware to the robot.

Both robots use the same firmware, with their network configuration determined by their assigned IP addresses.

## Team Members

* Anik Das
* Md. Abu Sian

## Presentation

The project presentation file is included in this repository as a PDF.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

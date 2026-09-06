# Firmware Instructions

1. Install the **ArduinoJson** library.
2. Upload the **MAC address code** to each ESP.
3. Get the MAC address from the Serial Monitor and note it down.
4. Set the MAC address as a **static IP** in the Wi-Fi router and note the assigned IP.
5. Upload the **main modular firmware** to each ESP. No modification is required; use the same firmware for every ESP.
6. Add the respective static IPs to `server/config.py`.


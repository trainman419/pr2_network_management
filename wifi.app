display: Wifi
description: PR2 Wifi Management
platform: pr2
launch: pr2_network_management/wifi.launch
interface: pr2_network_management/wifi.interface
icon: pr2_network_management/wireless.png
clients:
   - type: android
     manager:
       intent-action: ros.android.pr2wifi.PR2WifiActivity

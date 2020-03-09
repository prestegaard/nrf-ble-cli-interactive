# nrf-ble-cli-interactive
Modified the BLE CLI interactive example from the nRF SDK 16.0.0, found here https://infocenter.nordicsemi.com/topic/sdk_nrf5_v16.0.0/ble_sdk_app_interactive.html?cp=7_1_4_2_1_1


## Changes from SDK example
* Changed format of advertising packet
** Use short name of 6 bytes instead of long name
** Include TX power
** Only use 13 byte custom data

* Added command to update TX power both physically and in advertisement packet
* Added command to update comapny ID in advertisement packet
* Added command to update flags in advertisement packet
* Added command to update UUID in advertisement packet
* Added command to update potential custom data in advertisement packet

* Added command to display advertisment packets instad of only addresses

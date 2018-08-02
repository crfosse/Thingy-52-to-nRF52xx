# nRF52xx-to-Thingy-52
An example of how you can get environment sensor data from Nordic Thingy:52 to a nRF52xx development kit(DK). This readme is only a brief description of the solution, please feel free to contact me if you have any questions.

Before reading I recommend taking a look at the nRF52 multi link example(https://github.com/NordicPlayground/nrf52-ble-multi-link-multi-role). This is a great example of the nRF52's capabilities and it has a good description of how you can connect your nRF52 DK to a Thingy:52.

For this example I have used:
- 1x nRF52832 DK
- 1x Thingy:52
- SDK 15.0.0
- Softdevice 6.0.0 for nRF52

Programming of the Thingy is not required at all.

The code takes base in the `ble_app_blinky_c` example found in the SDK. This uses the BLE LED button service to demonstrate how two DKs can toggle eachother's LEDs through transmitting button state. The first iteration of this project was to enable our DK's LEDs to be toggled by a Thingy button press. This can be done in two steps:

1. In main: change the `m_target_periph_name[]` to your Thingy's name(i.e. `"Thingy"`).
2. In ble_lbs_c.h: replace the LBS_UUID values with the corrosponding ones for the Thingy: 
```
#define LBS_UUID_BASE        {0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x00, 0x68, 0xEF}
#define LBS_UUID_SERVICE     0x0300
#define LBS_UUID_BUTTON_CHAR 0x0302
#define LBS_UUID_LED_CHAR    0x0301
```

This should enable your development kit to react on button presses from the Thingy. 

Achieving access to all the environment sensor data from the Thingy requires more work(and I have done it for you). In short I have taken the BLE LED button service code and rewritten it for handling Environment data from the Thingy. I have called this: Thingy Environment Service(TES). `main.c` is also ammended to support this. 

In order to try this out yourself:
1. Add the `ble_app_thingy_to_52_dk_c` project folder to `<Your SDK path>\examples\ble_central\`. This folder contains the `main.c` file and the µvision Keil project(Segger support is on its way). 
2. Move the folders in src to `<Your SDK path>\components\ble\ble_services\`. The folder `ble_tes_c` includes the Thingy Environment service files and the `ble_lbs_c` folder includes the original LED button service code amended for use with the Thingy(as described earlier). 
3. Build and flash it to you nrf52 DK. 

Short usage guide:
In `main.c` you'll find the function `tes_c_evt_handler` ehere you can choose which sensors you want to receive data from. Just call the respective sensor's `ble_tes_c_<sensor type>_notif_enable` function. The program then writes the received values over UART to you terminal.  

Note 1: It is possible to get config data from the Thingy, but I haven't looked into how that works. Use on your own risk. 

Note 2: CO2 data from the Thingy is as of now not working. But I'm trying to find out how to get it. 

Note 3: The necessary defines to add in the `sdk_config` are: 
```
// <q> BLE_TES_C_ENABLED  - ble_tes_c - Nordic Thingy Environment Service Client

#ifndef BLE_TES_C_ENABLED
#define BLE_TES_C_ENABLED 1
#endif

// <o> BLE_TES_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Thingy Environment Service Client.

#ifndef BLE_TES_C_BLE_OBSERVER_PRIO
#define BLE_TES_C_BLE_OBSERVER_PRIO 2
#endif

```

Note 4: This is only tested with a nRF52832 DK. Some amendments may be necessary for use with the nRF52840. 

Good luck!

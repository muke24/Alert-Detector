Made for fun.

----------------------------------------
Description:

This project was inspired by the original Need for Speed Most Wanted (2005) which includes a police detection device which points towards the closest police car. A really cool project which scrapes Waze API alert data (currently only POLICE alerts) and displays an arrow pointing towards the closest alert. It also shows the heat level, determined by how many police alerts are in the area. Along with that, the icon for the alert which was detected is also displayed, so if there are 3 police cars in the vicinity then 3 police icons will be shown.
The distance of the closest alert is also shown. Alert submission back to Waze does not exist, but if someone figured out how to, that would be cool.

This used to work perfectly, and if you use the Waze URL found within the code and paste it in to your search bar, you'll still see the JSON containing all of the alerts.
I think Waze may have patched this for devices which doesn't send cookie data, like this ESP32, but with some hacking around I think someone could get past this.
This code can be found in "Code/1_75 Inch/AlertFinder/AlertRetriever.cpp". Keep in mind this is a Waze backdoor API URL, use at your own risk. This could be changed at any time which it seems that some sort of security with it has changed and has not been working recently with the ESP32's I have tested recently. It still works within a Unity application as of currently. I will add that project to my GitHub soon.

Uses DIY knob as inspiration found at https://github.com/scottbez1/smartknob. The youtube video I followed was: https://www.youtube.com/watch?v=ip641WmY4pA.
I used this as I did not want a cheap feel to the device, and makes it more premium and is stronger than a regular rotary encoder like an EC35 rotary encoder.
I also did not want to explicitly use touch, as touch screens and cars don't work well together in my humble opinion, so I wanted to use the touch screen as a sensor to know when the knob is pushed down, which would in theory make a haptic like vibration to replicate a button press. This has not been completed, but that was the idea.

I'd love to add a dashcam functionality and even AI detection of alerts, however the current device runs at about 15 frames per second, so this is not viable with this setup.
A Jetson Nano should do that job well, but that would get expensive. Definitely something I want to make though.

----------------------------------------

Data Source Warning: 
The device currently uses a reverse-engineered, non-public Waze URL. 
This method is fragile and likely violates Waze's Terms of Service. The URL is currently broken for the device (it only works with cookies), 
so the first challenge for the community is to fix the access method (or find a new, legal API!). All code is provided under the MIT license."

----------------------------------------

PCB Warning:
PCB is untested. It is used to connect the ESP32 to the modules and power management. The ESP32 connects to this PCB by feeding 28AUG enameled copper wires through the hollow shaft motor. Do not hot glue the enamelled wires directly to the ESP32, but solder them onto the end of a 8 pin male header and push that into the device, seems to work pretty well.

----------------------------------------

Files for Creation

Code: "Code/1_75 Inch/AlertFinder.ino"
Models: "Models/13" (Read the README file in Models folder)
PCB Files: PCB/1_75/kiCadWiringDiagram2025aug29/kiCadWiringDiagram
Parts: 
- ESP32: Waveshare 1.75Inch AMOLED ESP32-S3 (with GPS and Speaker) - https://www.waveshare.com/esp32-s3-touch-amoled-1.75.htm
- LED: (48leds used) 160 LEDs/m WS2812B 2.7mm width - https://www.aliexpress.com/item/1005006697141669.html?spm=a2g0o.order_list.order_list_main.172.643b18023ERucf
- Motor: 2804 iFlight Ipower - https://www.aliexpress.com/item/1005008389835260.html?spm=a2g0o.order_list.order_list_main.67.7c741802dJIMvc
(will test cheaper model eventually, I received 3 different types, there was a cheaper one that would have worked with some modeling work to add an encoder {https://www.aliexpress.com/item/4001123251754.html?spm=a2g0o.order_list.order_list_main.77.7c741802dJIMvc} but I received the iFlight first)
- PCB (To be sent to manufacturer JLCPCB)
- 28AUG Enamelled Wire
- 8 pin male header
- 10 Pin JST XH male header

On PCB:
- Magnetometer: MMC5883 (on PCB)
- Motor controller: TMC6300-LA (Might be changed on PCB if no stock is available)

M2 screws used.

----------------------------------------

TODO:

- Remove wire tray from "15" model and rename that to "16". (It was added to test things before I got a PCB manufactured, but even the tray was never used, so will need to fix that up)
- Code haptics and knob control for the motor
- Code LED's
- Code the magnetometer (merge the MMC5883 with the built-in 6 axis IMU on the Waveshare device to create a 9-axis IMU so that compass will work accurately)
- Code other Waze alerts and subtypes
- Code user settings so that brightness / selected alerts / ect can be modified

Things I have not personally tested on this model yet:
I have not yet manufactured a PCB.
I have not tested or coded the motor functionality. The motor will be used for haptics, and knob rotation feedback and selecting options such as changing alerts, brightness ect (not coded yet but you get the idea).
LED's have not yet been tested.

I do not want touch functionality, touch only exists to detect button presses. A mini touch screen makes no sense on a device made for use in a car to specifically reduce the need to even look at the Waze app for alert info.

I did make a different version with different parts. It was ugly and no model was made for it, but functionally it worked entirely (encoder worked, LED's worked and would flash at a speed dependant on how far the closest alert was, GUI was functional but showed no colour and was ugly, the arrow pointed at the closest police alert, speaker made OG NFSMW radar sounds which also played repeatedly dependant on how far the closest alert was). This can be found somewhere in the "Old" folder. I've provided Google Drive links with videos of the old version working with real alerts.

----------------------------------------

https://github.com/user-attachments/assets/8ef88b90-4c08-46e0-89e6-efae050536bf

![IMG_3583](https://github.com/user-attachments/assets/8a708f85-1d08-458c-ad3b-f7323527e11f)
![IMG_3606](https://github.com/user-attachments/assets/19f7778e-d1c3-4d5b-a04c-d6e3c4ec1828)
![IMG_3410](https://github.com/user-attachments/assets/ec7811a2-9f51-4a33-84d0-fb7bfc86974c)
![IMG_3420](https://github.com/user-attachments/assets/7a383259-eb33-4d4e-96cc-ea5484fc7472)
![IMG_3421](https://github.com/user-attachments/assets/65d46f26-b007-4ea7-8e20-91a6984d283a)
![IMG_3485](https://github.com/user-attachments/assets/6766496d-a223-43fb-81a0-0540633a9ef1)
![IMG_3553](https://github.com/user-attachments/assets/d6f2a744-af22-4f6d-b79e-90dc605bd09d)

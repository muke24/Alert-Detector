Made for fun. Sorry in advance, the organisation is pure executive dysfunction and the last time I worked on this project was months ago and I've forgotten stuff. I'll update this when I go through the files again to rejot my memory and provide better organised info.

----------------------------------------

A really cool project which scrapes Waze API alert data (currently only POLICE alerts) and displays an arrow pointing towards the closest alert. 
It also shows the heat level, determined by how many police alerts are in the area.
Along with that, the icon for the alert which was detected is also displayed, so if there are 3 police cars in the vicinity then 3 police icons will be shown.
The distance of the closest alert is also shown.

This used to work perfectly, and if you use the Waze URL found within the code and paste it in to your search bar, you'll still see the JSON containing all of the alerts.
I think Waze may have patched this for devices which doesn't send cookie data, like this ESP32, but with some hacking around I think someone could get past this.

Alert submission back to Waze does not exist, but if someone figured out how to, that would be cool.

This project was inspired by the original Need for Speed Most Wanted (2005) which includes a police detection device which points towards the closest police car.

I've had too much of a hard time with this project and unable to learn PCB design, but I really want to see it in existence.

I'd love to add a dashcam functionality and even AI detection of alerts, however the current device runs at about 15 frames per second, so this is probably not viable with this setup.
A Jetson Nano should do that job well, but that would get expensive.

----------------------------------------

All parts can be found on AliExpress, totalling around $200 AUD from memory. Uses 3D printed model for casing with model files included. 
Uses DIY knob as inspiration found at https://github.com/scottbez1/smartknob. The youtube video I followed was: https://www.youtube.com/watch?v=ip641WmY4pA.
I used this as I did not want a cheap feel to the device, and makes it more premium and is stronger than a regular rotary encoder like an EC35 rotary encoder.
I also did not want to explicitly use touch, as touch screens and cars don't work well together in my humble opinion, so I wanted to use the touch screen as a sensor to know when the knob is pushed down, which would in theory make a haptic like vibration to replicate a button press. This has not been completed, but that was the idea.

----------------------------------------

Some random details:

M2 screws used. Don't use M2.5 or M3 as they are too phat.
The 1.75 inch version is the most up to date, however older versions did have the LED blinking functionality for a different ESP32.
Just use the 1_75 as I cannot even remember what the others were like.
Feel free to enquire about anything, I just don't have the time as of the moment to do a full deep dive of this repository yet.

----------------------------------------

Relevant Information (Work in Progress)

There's a lot of old, random and unfinished stuff.
There's some random AI generated stuff.

There's 3 different attempts of making this device.
(Thats why its all messy). I will name them by revision. 

- 1.75 Inch Waveshare (As seen in pictures | Revision 3): Only able to test with real alerts before the GUI and compass was working, as Waze seems to have blocked their URL from ESP's sometime in July??? idk but it once worked with the barebones device as proof
AI told me it probably has something to do with cookies as this JSON data can still be retrieved via a web browser on PC.
LED's dont work, as I havent wired them up nor coded it for this device yet.
Don't think compass will work as I haven't hooked up a magnetometer. The Waveshare only has a 6-axis IMU without a magnetometer which is needed. I will use a HMC5883
I had a freelancer make a PCB, but they exceeded budget before completion. Some things were not up to spec.
The PCB includes power management, connections for ESP32 and modules, and the magnetometer. I have not been able to send to manufacture yet so I am not sure if it works. I was going to use pcbway or jlcpcb.
Has a demo mode if the Waze URL or Wifi isn't working.

- 1.28 Inch ESP32C3 Knob + LilyGo T-SIM7000G (Revision 2): Never got LTE working for network access (I'd have to code for every carrier), 
however this worked. LED's light up. 1.28 Inch ESP32C3 Knob Screen was for GUI and rotary encoder data. Lilygo was for receiving the Alerts via LTE or Wifi (couldn't get LTE working) and connecting together modules (LEDs, IMU ect)
Also attempted getting a waveshare 1.28 screen (with no ESP32) connected, but I don't think it worked. I moved onto the 1.75inch after this.

- ESP-WROOM-32D (Revision 1): Used for testing IMU data and Waze data retrieval.

FILES

Revision 3
Code: Code/1_75 Inch/New/AlertFinder.ino
Models: 
PCB Files: 
Parts: 
- ESP32: Waveshare 1.75Inch AMOLED
- LED: 48leds WS2812B 3mm width
- Motor: 2804 iFlight Ipower (will test cheaper model eventually)
- Magnetometer: MMC5883

Other:
Motor controller (Might be changed on PCB if no stock is available): 

----------------------------------------

Revision 3.

https://github.com/user-attachments/assets/8ef88b90-4c08-46e0-89e6-efae050536bf

![IMG_3583](https://github.com/user-attachments/assets/8a708f85-1d08-458c-ad3b-f7323527e11f)
![IMG_3606](https://github.com/user-attachments/assets/19f7778e-d1c3-4d5b-a04c-d6e3c4ec1828)
![IMG_3410](https://github.com/user-attachments/assets/ec7811a2-9f51-4a33-84d0-fb7bfc86974c)
![IMG_3420](https://github.com/user-attachments/assets/7a383259-eb33-4d4e-96cc-ea5484fc7472)
![IMG_3421](https://github.com/user-attachments/assets/65d46f26-b007-4ea7-8e20-91a6984d283a)
![IMG_3485](https://github.com/user-attachments/assets/6766496d-a223-43fb-81a0-0540633a9ef1)
![IMG_3553](https://github.com/user-attachments/assets/d6f2a744-af22-4f6d-b79e-90dc605bd09d)

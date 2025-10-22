A really cool project which scapes Waze API alert data (currently only POLICE alerts) and displays an arrow pointing towards the closest alert. 
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
I used this as I did not want a cheap feel to the device, and makes it more premium and is likely stronger than a regular rotary encoder like an EC35 rotary encoder.
I also did not want to explicetly use touch, as touch screens and cars don't work well together in my humble opinion, so I wanted to use the touch screen as a sensor to know when the knob is pushed down, which would in theory make a haptic like vibration to replicate a button press. This has not been completed, but that was the idea.

----------------------------------------



https://github.com/user-attachments/assets/8ef88b90-4c08-46e0-89e6-efae050536bf

![IMG_3583](https://github.com/user-attachments/assets/8a708f85-1d08-458c-ad3b-f7323527e11f)
![IMG_3606](https://github.com/user-attachments/assets/19f7778e-d1c3-4d5b-a04c-d6e3c4ec1828)
![IMG_3410](https://github.com/user-attachments/assets/ec7811a2-9f51-4a33-84d0-fb7bfc86974c)
![IMG_3420](https://github.com/user-attachments/assets/7a383259-eb33-4d4e-96cc-ea5484fc7472)
![IMG_3421](https://github.com/user-attachments/assets/65d46f26-b007-4ea7-8e20-91a6984d283a)
![IMG_3485](https://github.com/user-attachments/assets/6766496d-a223-43fb-81a0-0540633a9ef1)
![IMG_3553](https://github.com/user-attachments/assets/d6f2a744-af22-4f6d-b79e-90dc605bd09d)

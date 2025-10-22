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

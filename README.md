# RoamingBot4WD
### Roaming and obstacle avoiding 4WD robot.

Bot has a 4WD system and 2 ultrasonic sensors - one facing forward and the other pointing down. Based on the common arduino project you will find floating around the internet that uses two wheels and one ultrasonic sensor to move a bot around a room. The goal here is to expand upon the original idea so the bot operates until the battery runs out and use available equipment. Extra functions help the bot stay out of corners, detect stalls, and avoid drops. 

Bugs:
* Ping not frequent enough and bot might hit obstacle before detecting. Wasn't an issue in earlier versions.
* Stall routine occasionally not triggered.

To Do:
- [ ] Multithreading for concurrency. Project would greatly benefit from having Ping function constantly working. 
- [ ] Use a more object oriented style with classes? There seems to be debate on whether this actually benefits microcontroller programs, but if this allows for a more universal obstacle avoiding bot or better avoidance routines I'll look into it.

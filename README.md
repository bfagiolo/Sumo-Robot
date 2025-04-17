# Sumo-Robot
This Arduino code runs our competitive sumo robot that made it to round 3 in the wrestling competition. The robot uses a clever scanning system with an ultrasonic sensor mounted on a servo that continuously moves between five positions to find opponents anywhere in the arena. We added three infrared sensors underneath to detect the white boundary line and prevent ring-outs.

The robot's brain uses different modes depending on what's happening - it can scan for enemies, track and charge when it spots one, and quickly turn around if it detects the boundary. The coolest part is when it finds an opponent - it aligns itself based on where the enemy was detected, then charges at different speeds depending on how close the target is.

Having the continuously moving sensor gave us a big advantage since most other robots only had fixed sensors with limited detection angles. Our approach allowed us to detect opponents from multiple directions without having to constantly rotate the entire robot, making our strategy more efficient and responsive during battles.
![sumorobot1](https://github.com/user-attachments/assets/8c25d8c7-e198-40e3-a5d4-ce23755c2127)

# Self-Balancing-Mobile-Bot
- This repository holds the code for a 2-wheeled 1-axle mobile robot.

## Media
- [Body_model.jpg](https://github.com/sparsh-b/Self-Balancing-Mobile-Bot/blob/master/body_model.jpg) shows the body of the robot built.
- [Self_balancing_bot.mp4](https://github.com/sparsh-b/Self-Balancing-Mobile-Bot/blob/master/Self_balancing_bot.mp4) contains the demo of the bot in action.

## Implementation details
- An Arduino Uno was used to implement this project & an MPU-6050 was used to sense the orientation of the bot. The gyrosensor data was denoised and fed to the PID controller. The gains of the controller were tuned using Ziegler-Nichols' method.
- Later, partial direction control was implemented by controlling the direction of the bot when its orientation is close to upright position(approximately 0.5 degrees on both sides of the Unstable equilibrium position). A HC-05 was integrated into the system & the direction of the bot was controlled by an android app over bluetooth with HC-05's help.

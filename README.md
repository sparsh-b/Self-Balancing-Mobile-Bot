# Self-Balancing-Mobile-Bot
A 2-wheeled 1-axle mobile bot

An Arduino Uno was used to implement the project & an MPU-6050 was used to sense the orientation of the bot. The libraries written by jrowberg were used to read the data from MPU-6050.

In the first phase, a 2-wheeled 1-axle mobile bot was built by using PID control.

In the second phase, partial direction control was implemented by controlling the direction of the bot when its orientation is close to upright position(approximately 0.5 degrees on both sides of the Unstable equilibrium position). A HC-05 was integrated into the system & the direction of the bot was controlled by an android app over bluetooth with HC-05's help.

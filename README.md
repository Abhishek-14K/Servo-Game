# Servo-Game

Using STM32L476RG design a game using two servos. One servo motor will randomly move to one of 6 positions with a random delay of 1 to 4 seconds. The user should first calibrate another servo to determine its minimum and maximum signal, then every time the first servo moves into a new position, the user should move their servo using button 1 and 3 on the MFS. The button 2 is used to select the positions during calibration and start the game or restart it once it ends after 5 rounds. The time taken to move to the new position should be counted and displayed on the 7- segment display.

Refer to the src and inc folders for the source code.

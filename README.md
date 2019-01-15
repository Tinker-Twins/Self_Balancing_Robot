# Self_Balancing_Robot
Self-Balancing Robot is a robot that balances itself on just two wheels; it uses the principle of an inverted pendulum for balancing.  The robot is powered using a DC power supply and uses the Digital Motion Processing Unit (DMPU) on accelerometer and a gyroscope combined breakout board (GY-521) to stably calculate its tilt angle. It then sends this data to the microcontroller (Arduino Nano), which processes the incoming data and decides the direction and speed (using PWM) of the motors in order to balance the robot (by moving it in the direction of fall).  A control algorithm called ‘PID’ (Proportional-Integral-Derivative) is utilized to control the output signals to the motors. The output signal is passed to a motor driver circuit (L298N), which in turn drives the motors accordingly. Finally, the motors drive the robot in the direction it is falling in, with a speed adequate to keep it upright.  This robotics project won the First Prize at "TechKnow 2018 (First Edition)", a physics concept based project demonstration competition at SRM Institute of Science and Technology.
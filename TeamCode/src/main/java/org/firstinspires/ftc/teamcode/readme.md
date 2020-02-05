All test code is for the blue side, foundation side

Using internal IMU in Rev Hub to correct direction and orientation, as well as turning

IMU set to Adafruit IMU in an I2C named imu

Front left and right motors called "front left" and "front right" respectively

Back left and right motors called "rear left" and "rear right" respectively

To push code to GitHub type in terminal: git push origin master


Robot Configuration:
    Expantion Hub 3:
        Motors:
            port 0: front left
            port 1: front right
            port 2: rear left
            port 3: rear right
        IMU:
            I2C port 0: imu
    Expantion Hub 2:
        Motors:
            port 0: right intake
            port 1: left intake
        Servos:
            port 0: right servo
            port 1: left servo


DriveAvoidIMU is continuously moving forward using IMU for correction

FoundationAuto is final code for Fountation Autonomous

IsaacCode is code that Isaac makes and wants to test

ParkingAuto is moving forward enough to park underneath bridge, for close parking have line in
            center of robot, for far have line in center of front wheel closest to the wall

TestCodeAuto is moving forward test with while loop and runtime

TestCodeAuto2 is guess and check 2 point turn for Foundation

TestCodeAuto3 is foundation code using while loop and runtime and correction code with hard turn

TikkiCode is code that Tikki makes and wants to test

TrobotixTeleOp is  final code for teleop
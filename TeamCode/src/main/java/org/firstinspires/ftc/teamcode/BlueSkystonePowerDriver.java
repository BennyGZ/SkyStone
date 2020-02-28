/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name="Blue Side: Skystone Driver Side", group="Autonomous")
public class BlueSkystonePowerDriver extends LinearOpMode {

    private double counter       =  0;
    private double counter2      =  0;
    private double time          =  7;
    final double TARGET_DISTANCE =  225.0;    // Hold robot's center 400 mm from target

    /* Declare OpMode members. */
    Robot_OmniDrive     robot    = new Robot_OmniDrive();   // Use Omni-Directional drive system
    Robot_Navigation    nav      = new Robot_Navigation();  // Use Image Tracking library

    final double SPEED           =  0.4;

    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.initDrive(this);
        nav.initVuforia(this, robot);

        // Activate Vuforia (this takes a few seconds)
        nav.activateTracking();

        robot.setServo(0.5);
        // Wait for the game to start (driver presses PLAY)
        /*while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");

            // Display any Nav Targets while we wait for the match to start
            nav.targetsAreVisible();
            nav.addNavTelemetry();
            telemetry.update();
        }*/
        waitForStart();
        // run until the end of the match (driver presses STOP)
        robot.strafeRight(SPEED);
        sleep(1200);
        robot.stopMotor();
        sleep(500);
        telemetry.addData("Add", robot.getAngle());
        telemetry.addData("Last", robot.lastAngles.firstAngle);
        telemetry.update();
        robot.correctOrientation();
        robot.stopMotor();
        sleep(200);
        while (opModeIsActive()) {

            // auto drive or manual drive?
            // In auto drive, the robot will approach any target it can see and then press against it
            // In manual drive the robot responds to the Joystick.

//            if(nav.targetsAreVisible() && (nav.getRelativeBearing() > 5 && nav.getRelativeBearing() < -5)){
//                telemetry.addLine("Correct");
//                if(nav.getRelativeBearing() > 5){
//                    robot.turnMotor(SPEED);
//                }
//                if(nav.getRelativeBearing() < -5){
//                    robot.turnMotor(-SPEED);
//                }
//            }
            if (nav.targetsAreVisible() && (nav.getRobotY() >= 60)){
//                telemetry.addLine("Right");
//                telemetry.addData("RobotY:", "%5.0dmm", nav.getRobotY());
//                telemetry.update();
                robot.driveBackward(0.15);
                sleep(200);
                robot.stopMotor();
                sleep(500);
            }
            else if (nav.targetsAreVisible() && (nav.getRobotY() <= 30)){
//                telemetry.addLine("Left");
//                telemetry.addData("RobotY:", "%5.0dmm", nav.getRobotY());
//                telemetry.update();
                robot.driveForward(0.15);
                sleep(200);
                robot.stopMotor();
                sleep(500);
            }
            else if (nav.targetsAreVisible() && (nav.getRobotY() < 60 && nav.getRobotY() > 30)) {
                telemetry.addData("Add", robot.getAngle());
                telemetry.addData("Last", robot.lastAngles.firstAngle);
                telemetry.update();
                robot.correctOrientation();
                robot.stopMotor();
                sleep(400);
                robot.strafeRight(SPEED);
                sleep(600);
                robot.setServo(0);
                robot.stopMotor();
                sleep(200);
                robot.strafeLeft(SPEED);
                sleep(1200);
                robot.stopMotor();
                sleep(200);
                telemetry.addData("Add", robot.getAngle());
                telemetry.addData("Last", robot.lastAngles.firstAngle);
                telemetry.update();
                robot.correctOrientation();
                robot.stopMotor();
                sleep(200);
                robot.driveForward(SPEED);
                sleep((long)(1400 + counter));
                robot.stopMotor();
                sleep(1000);
                robot.driveBackward(SPEED);
                sleep(600);
                robot.stopMotor();
                sleep(30000);

            } else {
                robot.driveBackward(0.2);
                sleep(650);
                robot.stopMotor();
                sleep(1000);
                counter += 300;
                counter2 ++;

            }

            // Build telemetry messages with Navigation Information;
            nav.addNavTelemetry();

            //  Move the robot according to the pre-determined axis motions
//            if (!robot.checkAlignment())  {
//                robot.align();
//            }
//            robot.moveRobot();
            telemetry.update();
        }

        telemetry.addData(">", "Shutting Down. Bye!");
        telemetry.update();
    }
}
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BJ & Vivien TeleOp", group="Linear Opmode")

public class BJVivienTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor elevatorMotor = null;
    private DcMotor clawMotor = null;

    private double mode = 1;
    private double mode2 = 1;

    private Servo autoServo = null;
    private Servo leftServo = null;
    private Servo rightServo = null;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rear_right");
        leftDrive2 = hardwareMap.get(DcMotor.class, "rear_left");
        autoServo = hardwareMap.get(Servo.class, "auto_servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");
        clawMotor = hardwareMap.get(DcMotor.class, "claw");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Path0",  "Starting at %7d",
                elevatorMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            autoServo.setPosition(0.5);

            // Setup a variable for each drive wheel to save power level for telemetry
            boolean DPadLeft = gamepad1.dpad_left;
            boolean DPadRight = gamepad1.dpad_right;
            boolean DPadUp = gamepad1.dpad_up;
            boolean DPadDown = gamepad1.dpad_down;

            double triggerL = gamepad1.left_trigger;
            double triggerR = gamepad1.right_trigger;

            boolean bumperL = gamepad1.left_bumper;
            boolean bumperR = gamepad1.right_bumper;

            boolean buttonA = gamepad1.a;
            boolean buttonB = gamepad1.b;
            boolean buttonX = gamepad1.x;
            boolean buttonY = gamepad1.y;

            double leftStick = -gamepad1.left_stick_y;
            double rightStick = gamepad1.right_stick_y;

            boolean DPadLeft2 = gamepad2.dpad_left;
            boolean DPadRight2 = gamepad2.dpad_right;
            boolean DPadUp2 = gamepad2.dpad_up;
            boolean DPadDown2 = gamepad2.dpad_down;

            double triggerL2 = gamepad2.left_trigger;
            double triggerR2 = gamepad2.right_trigger;

            boolean bumperL2 = gamepad2.left_bumper;
            boolean bumperR2 = gamepad2.right_bumper;

            boolean buttonA2 = gamepad2.a;
            boolean buttonB2 = gamepad2.b;
            boolean buttonX2 = gamepad2.x;
            boolean buttonY2 = gamepad2.y;

            double leftStick2 = -gamepad2.left_stick_y;
            double rightStick2 = gamepad2.right_stick_y;


            //Set mode
            if (buttonA) {
                mode = 1;
            }
            if (buttonB){
                mode = 0.5;
            }

            //Set mode
            if (buttonA2) {
                mode2 = 1;
            }
            if (buttonB2){
                mode2 = 0.5;
            }


            if(Math.abs(leftStick) >= 0.1 || Math.abs(rightStick) >= 0.1){
                leftDrive.setPower(leftStick * mode);
                rightDrive.setPower(rightStick * mode);
                leftDrive2.setPower(leftStick * mode);
                rightDrive2.setPower(rightStick * mode);
            }
            else if(Math.abs(leftStick2) >= 0.1 || Math.abs(rightStick2) >= 0.1) {
                leftDrive.setPower(leftStick2 * mode2);
                rightDrive.setPower(rightStick2 * mode2);
                leftDrive2.setPower(leftStick2 * mode2);
                rightDrive2.setPower(rightStick2 * mode2);
            }
            else{
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftDrive2.setPower(0);
                rightDrive2.setPower(0);
            }

            //Elevator Claw
            if(bumperR || bumperL2){
                clawMotor.setPower(1);
            }
            else if(bumperL || triggerL2 >= 0.1){
                clawMotor.setPower(-0.5);
            }
            else{
                clawMotor.setPower(0);
            }


//            if (bumperL) {
//                clawMotor.setPower(-1);
//            }
//            else{
//                clawMotor.setPower(0);
//            }
//
//            if (bumperR) {
//                clawMotor.setPower(1);
//            }
//            else{
//                clawMotor.setPower(0);
//            }
//
//            if (bumperL2) {
//                clawMotor.setPower(1);
//            }
//            else{
//                clawMotor.setPower(0);
//            }
//
//            if(triggerL2 >= 0.1){
//                clawMotor.setPower(-triggerL2);
//            }
//            else{
//                clawMotor.setPower(0);
//            }




            //Elevator
            if(bumperR2 || triggerR >= 0.1){
                elevatorMotor.setPower(1);
            }
            else if(triggerL >= 0.1 || triggerR2 >= 0.1){
                elevatorMotor.setPower(-1);
            }
            else{
                elevatorMotor.setPower(0);
            }



//            if(triggerL >= 0.1){
//                elevatorMotor.setPower(-triggerL);
//            }
//            else{
//                elevatorMotor.setPower(0);
//            }
//
//            if(triggerR >= 0.1){
//                elevatorMotor.setPower(triggerR);
//            }
//            else {
//                elevatorMotor.setPower(0);
//            }
//
//            if (bumperR2) {
//                elevatorMotor.setPower(1);
//            }
//            else{
//                elevatorMotor.setPower(0);
//            }
//
//            if(triggerR2 >= 0.1){
//                elevatorMotor.setPower(-triggerR2);
//            }
//            else {
//                elevatorMotor.setPower(0);
//            }





            //elevator lift
            if (buttonX){
                leftServo.setPosition(0);
                rightServo.setPosition(0);
                telemetry.addData("Servos", "UNLATCHED");
                telemetry.update();
            }
            if (buttonY){
                leftServo.setPosition(1);
                rightServo.setPosition(1);
                telemetry.addData("Servos", "LATCHED");
                telemetry.update();
            }
            //elevator lift
            if (buttonX2){
                leftServo.setPosition(0);     //inside
                rightServo.setPosition(0);    //inside
                telemetry.addData("Servos", "UNLATCHED");
                telemetry.update();
            }
            if (buttonY2){
                leftServo.setPosition(1);
                rightServo.setPosition(1);    //down
                telemetry.addData("Servos", "LATCHED");
                telemetry.update();
            }




            //DPad
            if (DPadLeft) {
                leftDrive.setPower(-1 * mode);
                rightDrive.setPower(-1 * mode);
                rightDrive2.setPower(1 * mode);
                leftDrive2.setPower(1 * mode);
            } else if (DPadRight) {
                leftDrive.setPower(1 * mode);
                rightDrive.setPower(1 * mode);
                rightDrive2.setPower(-1 * mode);
                leftDrive2.setPower(-1 * mode);
            } else if (DPadUp) {
                leftDrive.setPower(1 * mode);
                rightDrive.setPower(-1 * mode);
                rightDrive2.setPower(-1 * mode);
                leftDrive2.setPower(1 * mode);
            } else if (DPadDown){
                leftDrive.setPower(-1 * mode);
                rightDrive.setPower(1 * mode);
                rightDrive2.setPower(1 * mode);
                leftDrive2.setPower(-1 * mode);
            } else if (DPadLeft2) {
                leftDrive.setPower(-1 * mode2);
                rightDrive.setPower(-1 * mode2);
                rightDrive2.setPower(1 * mode2);
                leftDrive2.setPower(1 * mode2);
            } else if (DPadRight2) {
                leftDrive.setPower(1 * mode2);
                rightDrive.setPower(1 * mode2);
                rightDrive2.setPower(-1 * mode2);
                leftDrive2.setPower(-1 * mode2);
            } else if (DPadUp2) {
                leftDrive.setPower(1 * mode2);
                rightDrive.setPower(-1 * mode2);
                rightDrive2.setPower(-1 * mode2);
                leftDrive2.setPower(1 * mode2);
            } else if (DPadDown2) {
                leftDrive.setPower(-1 * mode2);
                rightDrive.setPower(1 * mode2);
                rightDrive2.setPower(1 * mode2);
                leftDrive2.setPower(-1 * mode2);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                rightDrive2.setPower(0);
                leftDrive2.setPower(0);
            }








            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftStick, rightStick);
            telemetry.update();

        }
    }
}

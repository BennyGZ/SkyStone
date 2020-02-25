package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Foundation Red", group="Autonomous")
public class FoundationRed extends LinearOpMode {

    Robot_OmniDrive     robot    = new Robot_OmniDrive();
    private ElapsedTime runtime = new ElapsedTime();

    private Servo leftServo = null;
    private Servo rightServo = null;

    @Override
    public void runOpMode() {
        final long START = 1000;
        final long END = 10000;
        robot.initDrive(this);
        robot.setServo(0.5);
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        waitForStart();
        sleep(START);
        robot.encoderDrive(.06, -10, -10, 10, 10, 1);           //Strafe Right
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(0.06, -10, 10, -10, 10, 1.3);       //Drive Forward
        robot.stopMotor();
        sleep(1000);

        rightServo.setPosition(1);                                                                              //Lower Servos
        leftServo.setPosition(1);
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(.06, 10, -10, 10, -10, 1.9);           //Drive Backwards
        robot.stopMotor();
        sleep(1000);

//        robot.encoderDrive(0.06, -10, -10, -10, -10, 1);        //Turn Left
//        robot.stopMotor();
//        sleep(1000);

        rightServo.setPosition(0);                                                                              //Lower Servos
        leftServo.setPosition(0);
        robot.stopMotor();
        sleep(1000);
        sleep(END);
        robot.encoderDrive(.06, 10, 10, -10, -10, 2.2);           //Strafe Left
//        robot.stopMotor();
//        sleep(1000);
//        robot.encoderDrive(.06, 10, -10, 10, -10, 2);           //Drive Backwards
//        robot.stopMotor();
        sleep(1500);
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(120);  //Turn left
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(110);  //Turn left
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(90);  //Turn left
        robot.encoderDrive(0.06, 10, -10, 10, -10, .5);
        sleep(1000);
    }


}

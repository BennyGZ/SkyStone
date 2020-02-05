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


@Autonomous(name="Foundation Blue", group="Autonomous")
public class FoundationBlue extends LinearOpMode {

    Robot_OmniDrive     robot    = new Robot_OmniDrive();
    private ElapsedTime runtime = new ElapsedTime();

    private Servo leftServo = null;
    private Servo rightServo = null;

    @Override
    public void runOpMode() {

        robot.initDrive(this);
        robot.setServo(0.5);
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        waitForStart();

        robot.encoderDrive(.06, 10, 10, -10, -10, 1);           //Strafe Left
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(0.06, -10, 10, -10, 10, 1.25);       //Drive Forward
        robot.stopMotor();
        sleep(1000);

        rightServo.setPosition(1);                                                                              //Lower Servos
        leftServo.setPosition(1);
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(.06, 10, -10, 10, -10, 0.75);           //Drive Backwards
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(0.06, -10, -10, -10, -10, 1);        //Turn Left
        robot.stopMotor();
        sleep(1000);

        rightServo.setPosition(0);                                                                              //Lower Servos
        leftServo.setPosition(0);
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(.06, 10, -10, 10, -10, 1);           //Drive Backwards
        robot.stopMotor();
        sleep(30000);
    }


}
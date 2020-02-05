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


@Autonomous(name="Omkar Code", group="Autonomous")
public class OmkarCode extends LinearOpMode {
    Robot_OmniDrive     robot    = new Robot_OmniDrive();
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
        robot.initDrive(this);
        leftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rear_right");
        leftDrive2 = hardwareMap.get(DcMotor.class, "rear_left");
        autoServo = hardwareMap.get(Servo.class, "auto_servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");
        clawMotor = hardwareMap.get(DcMotor.class, "claw");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        waitForStart();

//        public void forward(double time) {
//            robot.encoderDrive(0.06, 10, 10, -10, -10, time);
//        }
//        public void backword(int time) {
//            robot.encoderDrive(0.06, -10, -10, -10, -10, time);
//        }
//        public void right(int time) {
//            robot.encoderDrive(0.06, 10, -10, 10, -10, time);
//        }
//        public void left(int time) {
//            robot.encoderDrive(0.06, -10, 10, -10, 10, time);
//        }
//        public void moveServos(int rotation) {
//            rightServo.setPosition(rotation);
//            leftServo.setPosition(rotation);
//        }
        /* move forward until it's on the foundation; then move the servos to gain control of the robot
        then pull back out; then */
//        backword(4);
//        left(1);
//        moveServos(0.5);
        robot.encoderDrive(.06, 10, 10, -10, -10, 1);
        sleep(1000);
        robot.encoderDrive(0.06, -10, 10, -10, 10, 1.25);
        sleep(1000);

        rightServo.setPosition(2);
        leftServo.setPosition(2);
        sleep(1000);
        robot.encoderDrive(.06, 10, -10, 10, -10, 1);
        sleep(1000);
        robot.encoderDrive(0.06, -10, 10, 20, -10, 2);
    }


}
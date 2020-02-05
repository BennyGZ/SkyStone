package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/**
 * This is NOT an opmode.
 *
 * This class defines all the specific hardware for a three wheel omni-bot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left drive"
 * Motor channel:  Right drive motor:        "right drive"
 * Motor channel:  Rear  drive motor:        "back drive"
 *
 * These motors correspond to three drive locations spaced 120 degrees around a circular robot.
 * Each motor is attached to an omni-wheel. Two wheels are in front, and one is at the rear of the robot.
 *
 * Robot motion is defined in three different axis motions:
 * - Axial    Forward/Backwards      +ve = Forward
 * - Lateral  Side to Side strafing  +ve = Right
 * - Yaw      Rotating               +ve = CCW
 */


public class Robot_OmniDrive
{
    // Private Members
    private LinearOpMode myOpMode;

    private DcMotor  leftDrive      = null;
    private DcMotor  rightDrive     = null;
    private DcMotor  leftDrive2      = null;
    private DcMotor  rightDrive2      = null;
    private Servo   autoServo = null;
    private double  driveAxial      = 0 ;   // Positive is forward
    private double  driveLateral    = 0 ;   // Positive is right
    private double  driveYaw        = 0 ;   // Positive is CCW

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.9 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Constructor */
    public Robot_OmniDrive(){

    }


    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {

        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        leftDrive        = myOpMode.hardwareMap.get(DcMotor.class, "front_left");
        rightDrive       = myOpMode.hardwareMap.get(DcMotor.class, "front_right");
        leftDrive2       = myOpMode.hardwareMap.get(DcMotor.class, "rear_left");
        rightDrive2      = myOpMode.hardwareMap.get(DcMotor.class, "rear_right");

        autoServo  = myOpMode.hardwareMap.get(Servo.class, "auto_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        //use RUN_USING_ENCODERS because encoders are installed.
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop all robot motion by setting each axis value to zero
        moveRobot(0,0,0) ;
    }

    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(-myOpMode.gamepad1.right_stick_x);
    }
    public void driveMotor(double motorPower){
        leftDrive.setPower(-motorPower);
        rightDrive.setPower(motorPower);
        leftDrive2.setPower(-motorPower);
        rightDrive2.setPower(motorPower);

    }
    public void turnMotor(double turnPower){    //ccw
        leftDrive.setPower(turnPower/5);
        rightDrive.setPower(turnPower/5);
        leftDrive2.setPower(turnPower/5);
        rightDrive2.setPower(turnPower/5);
    }
    public void strafeMotor(double motorPower){
        leftDrive.setPower(motorPower);
        rightDrive.setPower(motorPower);
        leftDrive2.setPower(-motorPower);
        rightDrive2.setPower(-motorPower);
    }
    public void setFrontLeft(double power){
        leftDrive.setPower(power);
    }
    public void setFrontRight(double power){
        rightDrive.setPower(power);
    }
    public void setRearLeft(double power){
        leftDrive2.setPower(power);
    }
    public void setRearRight(double power){
        rightDrive2.setPower(power);
    }
    public void stopMotor(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);
    }

    public void setServo(double servoPower){
        autoServo.setPosition(servoPower);
    }

    public void encoderDrive(double speed,
                             double frontLeft, double frontRight, double rearLeft, double rearRight,
                             double timeoutS) {
        int newFrontLeft;
        int newFrontRight;
        int newRearLeft;
        int newRearRight;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newFrontLeft = leftDrive.getCurrentPosition() + (int)(frontLeft * COUNTS_PER_INCH);
        newFrontRight = rightDrive.getCurrentPosition() + (int)(frontRight * COUNTS_PER_INCH);
        newRearLeft = leftDrive2.getCurrentPosition() + (int)(rearLeft * COUNTS_PER_INCH);
        newRearRight = rightDrive2.getCurrentPosition() + (int)(rearRight * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newFrontLeft);
        rightDrive.setTargetPosition(newFrontRight);
        leftDrive2.setTargetPosition(newRearLeft);
        rightDrive2.setTargetPosition(newRearRight);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));
        leftDrive2.setPower(Math.abs(speed));
        rightDrive2.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while  ((leftDrive.isBusy() && rightDrive.isBusy() && rightDrive2.isBusy() && leftDrive2.isBusy()) && (runtime.seconds() < timeoutS)) {
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    /***
     * void moveRobot(double axial, double lateral, double yaw)
     * Set speed levels to motors based on axes requests
     * @param axial     Speed in Fwd Direction
     * @param lateral   Speed in lateral direction (+ve to right)
     * @param yaw       Speed of Yaw rotation.  (+ve is CCW)
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /***
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public boolean checkAlignment() {
        if (driveLateral < 2 && driveLateral > - 2) return true;
        else return false;
    }
    public void align() {
        while (true) {
            leftDrive.setPower(.2);
            rightDrive.setPower(.2);
            leftDrive2.setPower(.2);
            rightDrive2.setPower(.2);
            if (driveLateral < 2 && driveLateral > -2) break;

    }
    }
    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions
        /*double back = driveYaw + driveLateral;
        double left = driveYaw - driveAxial - (driveLateral * 0.5);
        double right = driveYaw + driveAxial - (driveLateral * 0.5);*/

        double frontLeft = (driveAxial + driveLateral);
        double frontRight = (driveAxial + driveLateral);
        double rearLeft = (-driveAxial + driveLateral);
        double rearRight = (-driveAxial + driveLateral);

        /*double frontLeft = driveAxial - (driveLateral*0.5);
        double frontRight = -driveAxial - (driveLateral*0.5);
        double rearLeft = -driveAxial - (driveLateral*0.5);
        double rearRight = driveAxial - (driveLateral*0.5);*/

        /*double frontLeft = -driveYaw + driveAxial + driveLateral;
        double frontRight = driveYaw + driveAxial - driveLateral;
        double rearLeft = -driveYaw + driveAxial - driveLateral;
        double rearRight = driveYaw + driveAxial + driveLateral;*/

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        max = Math.max(max, Math.abs(rearLeft));
        max = Math.max(max, Math.abs(rearRight));
        if (max > 1)
        {
            frontLeft /= (max);
            frontRight /= (max);
            rearLeft /= (max);
            rearRight /= (max);
        }

        // Set drive motor power levels.
        leftDrive.setPower(frontLeft/7);
        rightDrive.setPower(frontRight/7);
        leftDrive2.setPower(rearLeft/7);
        rightDrive2.setPower(rearRight/7);

        // Display Telemetry
        myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f], B[%+5.2f]", frontLeft, frontRight, rearLeft, rearRight);
    }


    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
        leftDrive2.setMode(mode);
        rightDrive2.setMode(mode);
    }
}
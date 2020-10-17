package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a LegitPushbot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 */

public class LegitmapV1 {


    /* Public OpMode members. */

    public DcMotor FrontRight = null;
    public DcMotor FrontLeft = null;
    public DcMotor BackRight = null;
    public DcMotor BackLeft = null;




    public DcMotor Wheeloutake = null;

    public DcMotor Intake = null;
    public Servo Wgoalservo = null;
    public Servo Backservo = null;

    public ColorSensor colorSensor = null;


    //public ColorSensor colorSensor = null;

    public static final double redvalue = 200;
    public static final double bluevalue = 200;





    public static boolean isStone = true;

    float[] hsvValues = new float[3];
    final float[] values = hsvValues;


    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.37727198;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.85;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double Circumference = 62.1;

    static final double FR = 1;
    static final double FL = 1;
    static final double BR = 1;
    static final double BL = 1;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();



    /* Constructor */


    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FrontRight = hwMap.get(DcMotor.class, "FR");
        FrontLeft = hwMap.get(DcMotor.class, "FL");
        BackRight = hwMap.get(DcMotor.class, "BR");
        BackLeft = hwMap.get(DcMotor.class, "BL");
        Intake = hwMap.get(DcMotor.class,"I");
        Wheeloutake = hwMap.get(DcMotor.class, "WO");







        // Define and initialize ALL installed servos.

        Wgoalservo = hwMap.get(Servo.class, "WS");
        Backservo = hwMap.get(Servo.class, "BS");



        // Define all sensors

        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");


        // Set all motors to zero power
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.


        //Tells robot is using encoders
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set the directino Reverse for the Right side motors so we can get all positive value inputs
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);


    }

    public void stop() throws InterruptedException {
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
    }

    public void driveForward(double speed, double inches) throws InterruptedException {

        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Determine new target position, and pass to motor controller
        newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newfrontRightTarget = FrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newbackLeftTarget = BackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newbackRightTarget = BackRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        FrontLeft.setTargetPosition(newfrontLeftTarget);
        FrontRight.setTargetPosition(newfrontRightTarget);
        BackLeft.setTargetPosition(newbackLeftTarget);
        BackRight.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        FrontLeft.setPower(Math.abs(speed*FL));
        FrontRight.setPower(Math.abs(speed*FR));
        BackRight.setPower(Math.abs(speed*BR));
        BackLeft.setPower(Math.abs(speed*BL));
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        while (FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()) {
        }


        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);



        // Turn off RUN_TO_POSITION
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void driveBack(double speed, double inches) throws InterruptedException {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Determine new target position, and pass to motor controller
        newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        newfrontRightTarget = FrontRight.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        newbackLeftTarget = BackLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        newbackRightTarget = BackRight.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        FrontLeft.setTargetPosition(newfrontLeftTarget);
        FrontRight.setTargetPosition(newfrontRightTarget);
        BackLeft.setTargetPosition(newbackLeftTarget);
        BackRight.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        FrontLeft.setPower(Math.abs(-speed*FL));
        FrontRight.setPower(Math.abs(-speed*FR));
        BackRight.setPower(Math.abs(-speed*BR));
        BackLeft.setPower(Math.abs(-speed*BL));
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()) {
        }

        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);



        // Turn off RUN_TO_POSITION
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*public void driveTillTouch(String direction, double speed) {
        while (opMode.opModeIsActive()) {

            while (FrontTouch.getState() == true) {

                if (direction == "forward") {
                    FrontLeft.setPower(Math.abs(speed*FL));
                    FrontRight.setPower(Math.abs(speed*FR));
                    BackRight.setPower(Math.abs(speed*BR));
                    BackLeft.setPower(Math.abs(speed*BL));
                }
                if (direction == "back") {
                    FrontLeft.setPower(Math.abs(-speed*FL));
                    FrontRight.setPower(Math.abs(-speed*FR));
                    BackRight.setPower(Math.abs(-speed*BR));
                    BackLeft.setPower(Math.abs(-speed*BL));
                }
                if (direction == "right") {
                    BackLeft.setPower(-speed*BL);
                    BackRight.setPower(speed*BR);
                    FrontLeft.setPower(speed*FL);
                    FrontRight.setPower(-speed*FR);
                }
                if (direction == "left") {
                    BackLeft.setPower(speed*BL);
                    BackRight.setPower(-speed*BR);
                    FrontLeft.setPower(-speed*FL);
                    FrontRight.setPower(speed*FR);
                }
            }

            BackLeft.setPower(0);
            BackRight.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
        }
    }*/

    public void strafeLeft(double speed, double inches) throws InterruptedException {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Determine new target position, and pass to motor controller
        newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        newfrontRightTarget = FrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newbackLeftTarget = BackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newbackRightTarget = BackRight.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        FrontLeft.setTargetPosition(newfrontLeftTarget);
        FrontRight.setTargetPosition(newfrontRightTarget);
        BackLeft.setTargetPosition(newbackLeftTarget);
        BackRight.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        FrontLeft.setPower(Math.abs(-speed*FL));
        FrontRight.setPower(Math.abs(speed*FR));
        BackRight.setPower(Math.abs(-speed*BR));
        BackLeft.setPower(Math.abs(speed*BL));
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()) {
        }
        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);


        // Turn off RUN_TO_POSITION
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void strafeRight(double speed, double inches) throws InterruptedException {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Determine new target position, and pass to motor controller
        newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newfrontRightTarget = FrontRight.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        newbackLeftTarget = BackLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        newbackRightTarget = BackRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        FrontLeft.setTargetPosition(newfrontLeftTarget);
        FrontRight.setTargetPosition(newfrontRightTarget);
        BackLeft.setTargetPosition(newbackLeftTarget);
        BackRight.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        FrontLeft.setPower(Math.abs(speed*FL));
        FrontRight.setPower(Math.abs(-speed*FR));
        BackRight.setPower(Math.abs(speed*BR));
        BackLeft.setPower(Math.abs(-speed*BL));
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()) {
        }
        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);


        // Turn off RUN_TO_POSITION
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*public void stopAtRed(String direction, double speed) throws InterruptedException {
        while (colorSensor.red() < redvalue) {
            if (direction == "forward") {
                BackLeft.setPower(speed*BL);
                BackRight.setPower(speed*BR);
                FrontLeft.setPower(speed*FL);
                FrontRight.setPower(speed*FR);
            }
            if (direction == "back") {
                BackLeft.setPower(-speed);
                BackRight.setPower(-speed);
                FrontLeft.setPower(-speed);
                FrontRight.setPower(-speed);
            }
            if (direction == "right") {
                BackLeft.setPower(-speed*BL);
                BackRight.setPower(speed*BR);
                FrontLeft.setPower(speed*FL);
                FrontRight.setPower(-speed*FR);
            }
            if (direction == "left") {
                BackLeft.setPower(speed*BL);
                BackRight.setPower(-speed*BR);
                FrontLeft.setPower(-speed*FL);
                FrontRight.setPower(speed*FR);
            }

        }

        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
    }

    public void stopAtBlue(String direction, double speed) throws InterruptedException {
        while (colorSensor.blue() < bluevalue) {
            if (direction == "forward") {
                BackLeft.setPower(speed*BL);
                BackRight.setPower(speed*BR);
                FrontLeft.setPower(speed*FL);
                FrontRight.setPower(speed*FR);
            }
            if (direction == "back") {
                BackLeft.setPower(-speed*BL);
                BackRight.setPower(-speed*BR);
                FrontLeft.setPower(-speed*FL);
                FrontRight.setPower(-speed*FR);
            }
            if (direction == "right") {
                BackLeft.setPower(-speed*BL);
                BackRight.setPower(speed*BR);
                FrontLeft.setPower(speed*FL);
                FrontRight.setPower(-speed*FR);
            }
            if (direction == "left") {
                BackLeft.setPower(speed*BL);
                BackRight.setPower(-speed*BR);
                FrontLeft.setPower(-speed*FL);
                FrontRight.setPower(speed*FR);
            }

        }

        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
    }
*/
    public void turn(String direction, double speed, double degrees) throws InterruptedException {
        double inches = (degrees / 360) * Circumference;
        if (direction == "right") {
            inches = -inches;

        }
        if (direction == "left") {
            inches = inches;

        }
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Determine new target position, and pass to motor controller
        newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        newfrontRightTarget = FrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newbackLeftTarget = BackLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
        newbackRightTarget = BackRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        FrontLeft.setTargetPosition(newfrontLeftTarget);
        FrontRight.setTargetPosition(newfrontRightTarget);
        BackLeft.setTargetPosition(newbackLeftTarget);
        BackRight.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        FrontLeft.setPower(Math.abs(-speed*FL));
        FrontRight.setPower(Math.abs(-speed*FR));
        BackRight.setPower(Math.abs(-speed*BR));
        BackLeft.setPower(Math.abs(-speed*BL));
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()) {
        }

        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);


        // Turn off RUN_TO_POSITION
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


}

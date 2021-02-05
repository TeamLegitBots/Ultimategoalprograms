
package org.firstinspires.ftc.teamcode.HardwareMap;

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

public class LegitbotV1 {
    /*public static final double FR = 1;
    public  static final double FL = 1;
    public static final double BR = 1;
    public static final double BL = 1;

    public static final int horizontalMultiplier = 1;
    public static final int verticalLeftMultiplier = -1;
    public static final int verticalRightMultiplier = -1;


*/
    /* Public OpMode members. */
/*
    public DcMotor FrontRight = null;
    public DcMotor FrontLeft = null;
    public DcMotor BackRight = null;
    public DcMotor BackLeft = null;

 */
    public DcMotor WheelOutake = null;
    public DcMotor Pulley = null;
    public DcMotor Intake = null;
    public DcMotor Wgoalarm = null;
    public Servo Ring_gate = null;
    public Servo Wgoalservo = null;
    public Servo Backservo = null;




    public ColorSensor colorSensor = null;


    //public ColorSensor colorSensor = null;

    public static final double redvalue = 200;
    public static final double bluevalue = 200;





    public static boolean isStone = true;

    float[] hsvValues = new float[3];
    final float[] values = hsvValues;


    static final double COUNTS_PER_MOTOR_REV = 1300;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 1.49606;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double Circumference = 40.53;




    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public LegitbotV1() {

    }

    /* Constructor */


    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        this.hwMap = ahwMap;
/*
        // Define and Initialize Motors
        FrontRight = hwMap.get(DcMotor.class, "FR");
        FrontLeft = hwMap.get(DcMotor.class, "FL");
        BackRight = hwMap.get(DcMotor.class, "BR");
        BackLeft = hwMap.get(DcMotor.class, "BL");

 */
        WheelOutake = hwMap.get(DcMotor.class, "WO");
        Pulley = hwMap.get(DcMotor.class, "P");
        Intake = hwMap.get(DcMotor.class, "I");
        Wgoalarm = hwMap.get(DcMotor.class, "WGA");





        // Define and initialize ALL installed servos.

        Wgoalservo = hwMap.servo.get("WS");
        Backservo = hwMap.servo.get("BS");
        Ring_gate = hwMap.servo.get("gate");



        // Define all sensors

        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");


        // Set all motors to zero power
        /*
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontRight.setPower(0);

         */
        Pulley.setPower(0);
        Intake.setPower(0);
        Wgoalarm.setPower(0);
        WheelOutake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.


        //Tells robot is using encoders
        /*
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         */
        WheelOutake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Set the directino Reverse for the Right side motors so we can get all positive value inputs
        /*
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

         */

        //Wgoalarm.setDirection(DcMotor.Direction.REVERSE);

/*
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

 */



    }

    public void stop() throws InterruptedException {
        /*
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
         */
    }

    /* for reference:
        front left motor = vertical right encoder
        back right motor = vertical left encoder
        back left motor = horizontal encoder
     */

    /*
    public void driveForward(double speed, double inches) throws InterruptedException {

        double average = ((FrontLeft.getCurrentPosition() * verticalRightMultiplier) + (BackRight.getCurrentPosition() * verticalLeftMultiplier))/2;
        double targetPos = average + (inches * COUNTS_PER_INCH);

        while (average < targetPos){
            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);
            average = ((FrontLeft.getCurrentPosition() * verticalRightMultiplier) + (BackRight.getCurrentPosition() * verticalLeftMultiplier))/2;


        }
        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

    }

    public void driveBack(double speed, double inches) throws InterruptedException {

        double average = ((FrontLeft.getCurrentPosition() * verticalRightMultiplier) + (BackRight.getCurrentPosition() * verticalLeftMultiplier))/2;
        double targetPos = average - (inches * COUNTS_PER_INCH);

        while (average > targetPos){
            FrontLeft.setPower(-speed);
            FrontRight.setPower(-speed);
            BackLeft.setPower(-speed);
            BackRight.setPower(-speed);
            average = ((FrontLeft.getCurrentPosition() * verticalRightMultiplier) + (BackRight.getCurrentPosition() * verticalLeftMultiplier))/2;
        }
        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);


    }


    public void strafeLeft(double speed, double inches) throws InterruptedException {

        double targetPos = BackLeft.getCurrentPosition() * horizontalMultiplier - (inches * COUNTS_PER_INCH);

        while (BackLeft.getCurrentPosition() * horizontalMultiplier > targetPos){
            FrontLeft.setPower(-speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(-speed);
        }

        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);


    }

    public void strafeRight(double speed, double inches) throws InterruptedException {
        double targetPos = BackLeft.getCurrentPosition() * horizontalMultiplier+ (inches * COUNTS_PER_INCH);

        while (BackLeft.getCurrentPosition() * horizontalMultiplier< targetPos){
            FrontLeft.setPower(speed);
            FrontRight.setPower(-speed);
            BackLeft.setPower(-speed);
            BackRight.setPower(speed);
        }

        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

    }

    public void turnLeft(double speed, double degrees) throws InterruptedException {
        double inches = (degrees / 360) * Circumference;
        double targetPos = inches * COUNTS_PER_INCH;

        double originalRight = FrontLeft.getCurrentPosition() * verticalRightMultiplier;
        double originalLeft = BackRight.getCurrentPosition() * verticalLeftMultiplier;

        double deltaRight = (FrontLeft.getCurrentPosition() * verticalRightMultiplier) - originalRight;
        double deltaLeft = originalLeft- (BackRight.getCurrentPosition() * verticalLeftMultiplier);

        double averageDifference = (deltaRight + deltaLeft)/2;

        while (averageDifference<targetPos){
            FrontLeft.setPower(-speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(-speed);
            BackRight.setPower(speed);

            deltaRight = (FrontLeft.getCurrentPosition() * verticalRightMultiplier) - originalRight;
            deltaLeft = originalLeft- (BackRight.getCurrentPosition() * verticalLeftMultiplier);

            averageDifference = (deltaRight + deltaLeft)/2;
        }


        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);


    }

    public void turnRight(double speed, double degrees) throws InterruptedException {
        double inches = (degrees / 360) * Circumference;
        double targetPos = inches * COUNTS_PER_INCH;

        double originalRight = FrontLeft.getCurrentPosition() * verticalRightMultiplier;
        double originalLeft = BackRight.getCurrentPosition() * verticalLeftMultiplier;

        double deltaRight = originalRight- (FrontLeft.getCurrentPosition()* verticalRightMultiplier);
        double deltaLeft = (BackRight.getCurrentPosition() * verticalLeftMultiplier) - originalLeft;

        double averageDifference = (deltaRight + deltaLeft)/2;

        while (averageDifference<targetPos){
            FrontLeft.setPower(speed);
            FrontRight.setPower(-speed);
            BackLeft.setPower(speed);
            BackRight.setPower(-speed);

            deltaRight = originalRight- (FrontLeft.getCurrentPosition() * verticalRightMultiplier);
            deltaLeft = (BackRight.getCurrentPosition() * verticalLeftMultiplier) - originalLeft;

            averageDifference = (deltaRight + deltaLeft)/2;
        }


        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);


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



}

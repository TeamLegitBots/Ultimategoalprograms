package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMap.LegitbotV1;


@TeleOp(name = "ASMyTeleop")
//@Disabled
public class TeleopV1 extends LinearOpMode {

    LegitbotV1 robot = new LegitbotV1();

    double sensitivity = 1;



    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        robot.WheelOutake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.Wgoalarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int original_WA_pos = robot.Wgoalarm.getCurrentPosition();

        int WA_pos_1 = original_WA_pos;
        int WA_pos_2 = original_WA_pos + 450;
        int WA_pos_3 = original_WA_pos + 1500;

        double wobble_goal_arm_pos = 1;
        double WAerror=0;
        int WAservo_pos =0;
        double shooterspeed = 1;
        double powershotspeed = .87;


        waitForStart();


        while (opModeIsActive()) {

            // Code for the four drive motors
            /*
            float drive = -gamepad1.right_stick_x;
            float strafe = gamepad1.left_stick_x;
            float turn = gamepad1.left_stick_y;
            */

            float intake = gamepad2.right_stick_x;
            float wheelouttake = gamepad2.right_stick_y;
            float outtake = gamepad2.left_stick_y;

            /*
            float FR = drive - turn - strafe;
            float FL = -drive - turn  + strafe;
            float BR = drive - turn + strafe;
            float BL = -drive  - turn - strafe;
            */



            robot.Intake.setPower(-intake);
            //robot.WheelOutake.setPower(0.80 * (outtake));

            robot.Pulley.setPower(-intake);
            /*
            robot.FrontRight.setPower(sensitivity * FR);
            robot.FrontLeft.setPower(sensitivity * FL );
            robot.BackRight.setPower(sensitivity * BR);
            robot.BackLeft.setPower(sensitivity * BL);
            */
            setDriveMotorPower();

            telemetry.addData("2: left bumper", gamepad2.left_bumper);
            telemetry.update();
            if (gamepad2.left_bumper && outtake<-.5){
                robot.WheelOutake.setPower(powershotspeed);
            } else if (outtake<-.5){
                robot.WheelOutake.setPower(shooterspeed);
            } else{
                robot.WheelOutake.setPower(0);
            }
            //automatic ringgate
            /*if ((outtake<-.1 && intake>.1) || (intake<-.1 )){
                robot.Ring_gate.setPosition(1);
            } else{
                robot.Ring_gate.setPosition(0);

            }
        
            
            */

            //manual ringgate
            if (gamepad2.x){
                robot.Ring_gate.setPosition(.55);
            }else{
                robot.Ring_gate.setPosition(.8);

            }

            //move wobble goal arm servo
            if (gamepad2.a  && wobble_goal_arm_pos !=1){
                robot.Wgoalservo.setPosition(0);
            }
            if (gamepad2.b){
                robot.Wgoalservo.setPosition(1);
            }

            //drop intake
            if (gamepad2.y){
                robot.Backservo.setPosition(.75);
            } else{
                robot.Backservo.setPosition(.25);

            }



            // Drag the sparkmini system down


            // Stop if not linear motion kit input
            /*
            if(gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {


            }

*/





            //wobble goal arm

            if(gamepad2.dpad_up && wobble_goal_arm_pos<3){
                wobble_goal_arm_pos = wobble_goal_arm_pos+1;
                sleep(500);
            }
            if(gamepad2.dpad_down && wobble_goal_arm_pos>1){
                wobble_goal_arm_pos = wobble_goal_arm_pos-1;
                sleep(500);

            }
            //telemetry.addData("target position",wobble_goal_arm_pos);
            //telemetry.addData("current position",robot.Wgoalarm.getCurrentPosition());
            //telemetry.update();

            //move Wgoal arm to position 1
            if (wobble_goal_arm_pos == 1 && (robot.Wgoalarm.getCurrentPosition() < WA_pos_1 - WAerror || robot.Wgoalarm.getCurrentPosition() > WA_pos_1 + WAerror )){
                robot.Wgoalarm.setTargetPosition(WA_pos_1);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (robot.Wgoalarm.getCurrentPosition()>WA_pos_1){
                    robot.Wgoalarm.setPower(-1);
                } else {
                    robot.Wgoalarm.setPower(1);
                }
                while(robot.Wgoalarm.isBusy()&&opModeIsActive()){
                    setDriveMotorPower();
                    if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.a || gamepad2.b){
                        break;
                    }
                }
                robot.Wgoalarm.setPower(0);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            //telemetry.addData("target position",wobble_goal_arm_pos);
            //telemetry.addData("current position",robot.Wgoalarm.getCurrentPosition());
            //telemetry.update();
            //move Wgoal arm to position 2
            if (wobble_goal_arm_pos == 2 && (robot.Wgoalarm.getCurrentPosition() < WA_pos_2 - WAerror || robot.Wgoalarm.getCurrentPosition() > WA_pos_2 + WAerror )){
                robot.Wgoalarm.setTargetPosition(WA_pos_2);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                if (robot.Wgoalarm.getCurrentPosition()>WA_pos_2){
                    robot.Wgoalarm.setPower(-1);
                } else {
                    robot.Wgoalarm.setPower(1);
                }

                while(robot.Wgoalarm.isBusy() && opModeIsActive()){
                    setDriveMotorPower();
                    if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.a || gamepad2.b){
                        break;
                    }
                }
                robot.Wgoalarm.setPower(0);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            //move Wgoal arm to position 3
            if (wobble_goal_arm_pos == 3 && (robot.Wgoalarm.getCurrentPosition() < WA_pos_3 - WAerror || robot.Wgoalarm.getCurrentPosition() > WA_pos_3 + WAerror) ){
                robot.Wgoalarm.setTargetPosition(WA_pos_3);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (robot.Wgoalarm.getCurrentPosition()>WA_pos_3){
                    robot.Wgoalarm.setPower(-.75);
                } else {
                    robot.Wgoalarm.setPower(.75);
                }
                while(robot.Wgoalarm.isBusy()&&opModeIsActive()){
                    setDriveMotorPower();
                    if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.a || gamepad2.b){
                        break;
                    }
                }
                robot.Wgoalarm.setPower(0);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //sensitivity
            if(gamepad1.dpad_up && sensitivity<1){
                while(gamepad1.dpad_up){}
                sensitivity = sensitivity + .1;

            }
            if(gamepad1.dpad_down && sensitivity>.1){
                while(gamepad1.dpad_down){}

                sensitivity = sensitivity - .1;
            }
            telemetry.addData("sensitivity now: ", sensitivity);
            telemetry.update();


        }

    }
    public void setDriveMotorPower(){
        float drive = -gamepad1.right_stick_x;
        float strafe = gamepad1.left_stick_x;
        float turn = gamepad1.left_stick_y;

        float FR = drive - turn - strafe;
        float FL = -drive - turn  + strafe;
        float BR = drive - turn + strafe;
        float BL = -drive  - turn - strafe;

        robot.FrontRight.setPower(sensitivity * FR);
        robot.FrontLeft.setPower(sensitivity * FL );
        robot.BackRight.setPower(sensitivity * BR);
        robot.BackLeft.setPower(sensitivity * BL);
    }
}

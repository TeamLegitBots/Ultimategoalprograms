package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMap.LegitbotV1;


@TeleOp(name = "ASTeleopOneP")
//@Disabled
public class TeleopOneP extends LinearOpMode {

    LegitbotV1 robot = new LegitbotV1();
    
    double sensitivity = 2;

    

  
    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        telemetry.addData("right bumper1", gamepad1.right_bumper);
        telemetry.update();

        robot.WheelOutake.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

            telemetry.addData("horizontal encoder", robot.BackLeft.getCurrentPosition());
            telemetry.update();
            
            float intake = gamepad1.left_trigger;
            float wheelouttake = gamepad1.right_trigger;
            float outtake = gamepad1.right_trigger;

            if (gamepad1.left_bumper){
                intake = -1;
            }

            // Code for the four drive motors
            /*float drive = -gamepad1.right_stick_x;
            float strafe = gamepad1.left_stick_x;
            float turn = gamepad1.left_stick_y;



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

            telemetry.addData("right bumper", gamepad1.right_bumper);
            telemetry.update();
            if (gamepad1.right_bumper){
                robot.WheelOutake.setPower(powershotspeed);
            } else if (outtake>.5){
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
            if (gamepad1.x){
                robot.Ring_gate.setPosition(.55);
            }else{
                robot.Ring_gate.setPosition(.8);

            }
            //move wobble goal arm servo
            if (gamepad1.a  && wobble_goal_arm_pos !=1){
                robot.Wgoalservo.setPosition(0);
            }
            if (gamepad1.b){
                robot.Wgoalservo.setPosition(1);
            }
            
            //drop intake
            if (gamepad1.y){
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
            
            if(gamepad1.dpad_up && wobble_goal_arm_pos<3){
                wobble_goal_arm_pos = wobble_goal_arm_pos+1;
                sleep(500);
            }
            if(gamepad1.dpad_down && wobble_goal_arm_pos>1){
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

                }
                robot.Wgoalarm.setPower(0);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } 
            //move Wgoal arm to position 3
            if (wobble_goal_arm_pos == 3 && (robot.Wgoalarm.getCurrentPosition() < WA_pos_3 - WAerror || robot.Wgoalarm.getCurrentPosition() > WA_pos_3 + WAerror) ){
                robot.Wgoalarm.setTargetPosition(WA_pos_3);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (robot.Wgoalarm.getCurrentPosition()>WA_pos_3){
                    robot.Wgoalarm.setPower(-1);
                } else {
                    robot.Wgoalarm.setPower(1);
                }
                while(robot.Wgoalarm.isBusy()&&opModeIsActive()){
                    setDriveMotorPower();
                }
                robot.Wgoalarm.setPower(0);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            
            
            

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


    


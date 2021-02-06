package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMap.LegitbotV1;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;


@TeleOp(name = "ASMyTeleop")
//@Disabled
public class TeleopV1 extends LinearOpMode {

    LegitbotV1 robot = new LegitbotV1();




    //double sensitivity = 1;



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.init(hardwareMap);

        robot.WheelOutake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


/*
        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

 */

        robot.Wgoalarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int original_WA_pos = robot.Wgoalarm.getCurrentPosition();

        /*int WA_pos_1 = original_WA_pos - 1000;
        int WA_pos_2 = original_WA_pos;
        int WA_pos_3 = original_WA_pos + 2000;

        double wobble_goal_arm_pos = 2;
        double WAerror=0;
         */
        int WAservo_pos =0;
        double shooterspeed = 1;
        double powershotspeed = .87;


        waitForStart();


        while (opModeIsActive()) {

            //auto aim:
/*
            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();

            if (gamepad1.b) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            }

            if (gamepad1.a) {
                robot.WheelOutake.setPower(shooterspeed);

                robot.Intake.setPower(-1);
                robot.Pulley.setPower(-1);

                drive.setMotorPowers(0, 0, 0, 0);
                Trajectory myTrajectory = drive.trajectoryBuilder(myPose)
                        .lineToLinearHeading(new Pose2d(-48, 0, Math.toRadians(90)))
                        .build();

                drive.followTrajectory(myTrajectory);

                while (opModeIsActive()) {
                    if (gamepad1.a) {

                        robot.Ring_gate.setPosition(.55);
                        sleep(500);
                        robot.Ring_gate.setPosition(.8);
                        sleep(500);

                        robot.Ring_gate.setPosition(.55);
                        sleep(500);
                        robot.Ring_gate.setPosition(.8);
                        sleep(500);

                        robot.Ring_gate.setPosition(.55);
                        sleep(500);
                        robot.Ring_gate.setPosition(.8);
                        break;
                    }

                    if (gamepad1.y){
                        break;
                    }
                }




            }
            */

            drive.update();

            if (gamepad1.x) {
                drive.setMotorPowers(0,0,0,0);
                robot.WheelOutake.setPower(powershotspeed);

                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                Pose2d myPose = drive.getPoseEstimate();


                Trajectory powershot1 = drive.trajectoryBuilder(myPose)
                    .strafeTo(new Vector2d(0,23),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                        .build();
                Trajectory powershot2 = drive.trajectoryBuilder(powershot1.end())
                        .strafeTo(new Vector2d(0,29),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                Trajectory powershot3 = drive.trajectoryBuilder(powershot2.end())
                        .strafeTo(new Vector2d(0,35.5),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                drive.followTrajectory(powershot1);
                robot.Ring_gate.setPosition(.55);
                sleep(500);
                robot.Ring_gate.setPosition(.8);

                drive.followTrajectory(powershot2);
                robot.Ring_gate.setPosition(.55);
                sleep(500);
                robot.Ring_gate.setPosition(.8);

                drive.followTrajectory(powershot3);
                robot.Ring_gate.setPosition(.55);
                sleep(500);
                robot.Ring_gate.setPosition(.8);






            }


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
            drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

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
            if (gamepad2.a){
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
            if(gamepad2.dpad_up){
                robot.Wgoalarm.setPower(1);
            } else if (gamepad2.dpad_down){
                robot.Wgoalarm.setPower(-1);
            } else{
                robot.Wgoalarm.setPower(0);
            }

/*
            if(gamepad2.dpad_up && wobble_goal_arm_pos<3){
                wobble_goal_arm_pos = wobble_goal_arm_pos+1;
                //sleep(500);
            }
            if(gamepad2.dpad_down && wobble_goal_arm_pos>1){
                wobble_goal_arm_pos = wobble_goal_arm_pos-1;
                //sleep(500);

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
                    drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
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
                    drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
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
                    drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
                    if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.a || gamepad2.b){
                        break;
                    }
                }
                robot.Wgoalarm.setPower(0);
                robot.Wgoalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
*/
            //sensitivity

            /*
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
*/

        }

    }
    /*
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

     */
}

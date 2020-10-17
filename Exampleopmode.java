package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Red Quarry Wall")
//@Disabled
public class Exampleopmode extends LinearOpMode implements Autousingintakeinterface{

    public static double SkystonePlacement = 0;
    LegitmapV1 robot = new LegitmapV1();   // Use a Pushbot's hardware
    double robotSpeed = .6;
    double strafeSpeed = .6;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //code goes here






    }
    @Override
    public void startIntake(int speed, int time) {
        robot.Intake.setPower(speed);
        sleep(time);
    }

    @Override
    public void startOutake(int speed, int time) {
        robot.Wheeloutake.setPower(speed);
        sleep(time);

    }
}
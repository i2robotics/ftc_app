package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.RobotControl;

/**
 * Created by Nathanael on 10/29/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTest2", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class AutoTest2 extends LinearOpMode{

    //RobotControl robot;
    public ElapsedTime runtime;
    RobotControl robot;
    public void runOpMode() throws InterruptedException {
        robot = new RobotControl(this);
        waitForStart();
        runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.milliseconds() < 10000){
            if(!opModeIsActive()) return;
            telemetry.addData("Blue:", robot.colorEast.blue());
            telemetry.update();
        }


        //robot = new RobotControl(hardwareMap);
        /*
        int encoderError = robot.nw.getCurrentPosition();
        while(robot.nw.getCurrentPosition() < encoderError+250){
            robot.drive(0, .5, 0);
        }

        robot.stop();
        robot.hood.setPosition(.3725);
        robot.startFlyWheel(1);
        //wait(1000);
        robot.ballFeeder.setPower(1);
        //wait(1000);
        robot.ballFeeder.setPower(0);
        //wait(5000);
        robot.ballFeeder.setPower(1);
        //wait(1000);
        robot.ballFeeder.setPower(0);
        encoderError = robot.nw.getCurrentPosition();
        while(robot.nw.getCurrentPosition() < encoderError+500){
            robot.drive(0, .5, 0);
        }*/
        //robot.stop();



    }
}

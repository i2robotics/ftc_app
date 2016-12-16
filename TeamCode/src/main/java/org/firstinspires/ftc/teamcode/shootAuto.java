package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helpers.RobotControl;

/**
 * Created by Nathanael on 12/10/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Shoot Auto", group = "Linear Opmode")

public class shootAuto extends LinearOpMode {
    RobotControl robot;

    public void runOpMode() throws InterruptedException {
        robot = new RobotControl(this);
        robot.gyro.calibrate();

        //telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
        // robot.se.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        //System.out.println("Starting! Woo Hoo!");
        //Start up the flywheel and position the hood for the firing of the particles


        robot.startFlyWheel(8900);
        robot.hood.setPosition(.49);

        //Fire the ball
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 2000) {
            if (!opModeIsActive()) return;
            telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
            telemetry.update();
        }
        ;
        telemetry.update();
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 1500) {
            if (!opModeIsActive()) return;
            robot.ballFeeder.setPower(1);
            telemetry.addData("Switch", robot.SaddleSwitch.getVoltage());
        }
        while (robot.SaddleSwitch.getVoltage() < .1) {
            if (!opModeIsActive()) return;
            robot.ballFeeder.setPower(-1);
        }
        robot.ballFeeder.setPower(0);
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 1500) {
            if (!opModeIsActive()) return;
            robot.harvester.setPower(-1);
            if (robot.SaddleSwitch.getVoltage() < .1) {
                robot.ballFeeder.setPower(-1);
            } else {
                robot.ballFeeder.setPower(0);
            }
        }
        robot.harvester.setPower(0);
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 500) if (!opModeIsActive()) return;
        ;
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 2000) {
            if (!opModeIsActive()) return;
            robot.ballFeeder.setPower(1);
            telemetry.addData("Switch", robot.SaddleSwitch.getVoltage());
        }
        while (robot.SaddleSwitch.getVoltage() < .1) {
            if (!opModeIsActive()) return;
            robot.ballFeeder.setPower(-1);
        }
        robot.ballFeeder.setPower(0);
        robot.stopFlyWheel();

        robot.runtime.reset();
        robot.stop();

    }

}

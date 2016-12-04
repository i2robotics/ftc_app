package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Helpers.RobotControl;

/**
 * Created by Nathanael on 12/1/2016.
 */

public class RedAuto1 extends LinearOpMode {
    RobotControl robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotControl(this);
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating()) {
            telemetry.addData("Gyro heading", robot.gyro.getHeading());
            telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
            telemetry.update();

        }
        telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());

        waitForStart();
        //System.out.println("Starting! Woo Hoo!");
        //Start up the flywheel and position the hood for the firing of the particles
        /*robot.startFlyWheel(8715);
        robot.hood.setPosition(.875);

        //Fire the ball
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 2000){
            if(!opModeIsActive()) return;
            telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
            telemetry.update();
        };
        telemetry.clearAll();
        telemetry.update();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 1500){
            if(!opModeIsActive()) return;
            robot.ballFeeder.setPower(1);
            telemetry.addData("Switch", robot.SaddleSwitch.getVoltage());
        }
        while(robot.SaddleSwitch.getVoltage() < .1){
            if(!opModeIsActive()) return;
            robot.ballFeeder.setPower(-1);
        }
        robot.ballFeeder.setPower(0);
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 1500){
            if(!opModeIsActive()) return;
            robot.harvester.setPower(-1);
            if (robot.SaddleSwitch.getVoltage() < .1) {
                robot.ballFeeder.setPower(-1);
            } else {
                robot.ballFeeder.setPower(0);
            }
        }
        robot.harvester.setPower(0);
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 500) if(!opModeIsActive()) return;;
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 2000){
            if(!opModeIsActive()) return;
            robot.ballFeeder.setPower(1);
            telemetry.addData("Switch", robot.SaddleSwitch.getVoltage());
        }
        while(robot.SaddleSwitch.getVoltage() < .1){
            if(!opModeIsActive()) return;
            robot.ballFeeder.setPower(-1);
        }
        robot.ballFeeder.setPower(0);
        robot.stopFlyWheel();*/



        while(robot.wallSensorWest.getVoltage() > .097) {
            //System.out.println("looping");
            if(!opModeIsActive()) return;
            robot.drive(-47, 1, (float) robot.gyroRot());
            // Drive it at a 45 degree angle from the starting position
            System.out.println(robot.wallSensorWest.getVoltage());
            telemetry.addData("Gyro Heading", robot.gyro.getHeading());
            telemetry.addData("Ultra Sonic Sensor", robot.wallSensorWest.getVoltage());
            telemetry.addData("Time", robot.runtime.milliseconds());
            telemetry.update();
        }

        robot.stop();
        lineCheck(1, 1000);
        brake(75);

        robot.runtime.reset();
        beaconCheckRed();

        int encoder = robot.se.getCurrentPosition();
       /* while(Math.abs(encoder)+250 > Math.abs(robot.se.getCurrentPosition())){
            if(!opModeIsActive()) return;
            robot.drive(180, .75, robot.gyroRot());
        }*/
        brake(150);
        lineCheck(-1.5, 3000);
        brake(75);
        beaconCheckRed();
        robot.stop();
        encoder = robot.se.getCurrentPosition();
        while(Math.abs(encoder)+700 > Math.abs(robot.se.getCurrentPosition())){
            if(!opModeIsActive()) return;
            robot.drive(90,1, robot.gyroRot());
        }
        robot.stop();






    }
    public void beaconCheckRed(){
        //

        robot.ne.setPower(-.75);
        robot.nw.setPower(-.75);
        robot.se.setPower(.75);
        robot.sw.setPower(.75);
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 1500) {
//            robot.drive(90, .75, 0);
            if (!opModeIsActive()) return;
            telemetry.addData("Sensor", robot.colorWest.blue());
            telemetry.update();


            if (robot.colorWest.blue() >= 1) {
                if (!opModeIsActive()) return;
                robot.buttonPressWest.setPosition(.627);
                telemetry.addData("Sensor", robot.colorWest.blue());
                telemetry.update();
            }
            else if(robot.colorWest.blue() < 1){
                robot.buttonPressWest.setPosition(.9);

            }
            //while(robot.runtime.milliseconds() - 2000 < 200){}q




        }
        robot.stop();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 400){
            if(!opModeIsActive()) return;
            robot.drive(270, .8, 0);
        }
        robot.stop();
    }
    public void lineCheck(double direction, int time){
        robot.runtime.reset();
        while((robot.wLineSensor.getVoltage() < 1.5 || robot.wLineSensor.getVoltage()  < 1.5) && robot.runtime.milliseconds() <= time) {
            if (!opModeIsActive()) return;
            double power = .1;


            if (robot.wLineSensor.getVoltage() < 1.5) {
                robot.nw.setPower(power*direction);
                robot.sw.setPower(power*direction);

            } else {
                robot.nw.setPower(-6*power*direction);
                robot.sw.setPower(-6*power*direction);

            }
            if (robot.eLineSensor.getVoltage() < 1.5) {
                robot.ne.setPower(-power*direction);
                robot.se.setPower(-power*direction);
            } else {
                robot.ne.setPower(6*power*direction);
                robot.se.setPower(6*power*direction);

            }


        }
        /*int encoder = robot.se.getCurrentPosition();
        while(Math.abs(encoder)+25 > Math.abs(robot.se.getCurrentPosition())){
            if(!opModeIsActive()) return;
            robot.drive(180, .75, robot.gyroRot());
        }*/
        robot.runtime.reset();

        robot.stop();
    }
    public void brake(int time){
        robot.runtime.reset();
        robot.ne.setPower(Range.clip(-robot.ne.getPower()*100,-1,1));
        robot.se.setPower(Range.clip(-robot.se.getPower()*100,-1,1));
        robot.nw.setPower(Range.clip(-robot.nw.getPower()*100,-1,1));
        robot.sw.setPower(Range.clip(-robot.sw.getPower()*100,-1,1));
        while (robot.runtime.milliseconds() <= time) {
            if(!opModeIsActive()) return;

            //Just to add some extra time
        }
        robot.ne.setPower(0);
        robot.se.setPower(0);
        robot.sw.setPower(0);
        robot.nw.setPower(0);
    }
    }


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Helpers.RobotControl;

/**
 * Created by Nathanael on 12/1/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedAuto1", group = "Linear Opmode")
public class RedAuto1 extends LinearOpMode {
    RobotControl robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotControl(this);
        robot.gyro.calibrate();

        //telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
        // robot.se.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        //System.out.println("Starting! Woo Hoo!");
        //Start up the flywheel and position the hood for the firing of the particles

        /*
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
        */














        while(robot.wallSensorWest.getVoltage() > .097) {
            System.out.println("Driving at the angle");
            if(!opModeIsActive()) return;
            robot.drive(360-47, 1, robot.gyroRot()*2);
            telemetry.addData("Gyro Rot", robot.gyroRot()*2);
            telemetry.addData("Gyro: ", robot.gyro.getHeading());
            telemetry.addData("Wall:", robot.wallSensorWest.getVoltage());
            telemetry.update();
            // Drive it at a 45 degree angle from the starting position
        }
        lineCheck(1);
        while(robot.runtime.milliseconds() < 500) if(!opModeIsActive()) return;
        robot.stop();

        beaconCheckRed();


        //beaconCheckBlue();

        //int encoder = robot.se.getCurrentPosition();
        /*while(Math.abs(encoder)+250 > Math.abs(robot.se.getCurrentPosition())){
            if(!opModeIsActive()) return;
            robot.drive(180, .75, robot.gyroRot());
        }*/

        /*
        brake(150);
        robot.runtime.reset();
        robot.drive(180,.75,robot.gyroRot());
        while (robot.runtime.milliseconds() < 250) if(!opModeIsActive()) return;
        lineCheck(-1, 3000*2);
        brake(75);
        beaconCheckBlue();
        robot.stop();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 1000){
            if(!opModeIsActive()) return;
            robot.drive(-90,1, robot.gyroRot());
        }
       */

        robot.stop();






    }
    public void beaconCheckRed(){
        double rot;
        double ang;
        robot.runtime.reset();
        robot.drive(-90, .75, 0);
        boolean setPos = false;
        while (robot.runtime.milliseconds() < 1500) {

            if (robot.colorWest.blue() > 1 && !setPos) {
                if (!opModeIsActive()) return;

                robot.buttonPressWest.setPosition(1);
                setPos = true;

                telemetry.addData("Sensor", robot.colorWest.blue());
                telemetry.update();
            }
            else if(robot.colorWest.blue() <= 1 && !setPos){
                robot.buttonPressWest.setPosition(.627);
                setPos = false;

            }
            /*if(robot.colorWest.blue() > 1){
                rot = -.05;
                ang = 93;
            }
            else{
                ang = 85;
                rot = .05;
            }*/

            /*if (!opModeIsActive()) return;
            telemetry.addData("Sensor", robot.colorWest.blue());
            telemetry.update();
            if (robot.colorWest.blue() > 1) {
                if (!opModeIsActive()) return;
                robot.buttonPressWest.setPosition(.627);
                telemetry.addData("Sensor", robot.colorWest.blue());
                telemetry.update();87
            }
            else if(robot.colorWest.blue() <= 1) {
                robot.buttonPressWest.setPosition(.9);


            }*/
        }
        robot.stop();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 400){
            if(!opModeIsActive()) return;
            robot.drive(90, .8, 0);
        }
        robot.stop();
    }



    public void lineCheck(double direction){
        double eLineVoltage;
        double wLineVoltage;
        double power = .38;

        while ((robot.eLineSensor.getVoltage() < 1.5 && robot.wLineSensor.getVoltage() < 1.5)) {

            eLineVoltage = robot.eLineSensor.getVoltage();
            wLineVoltage = robot.wLineSensor.getVoltage();

            if (!opModeIsActive()) return;
            robot.drive(0, power, robot.gyroRot());
        }

        robot.stop();

        robot.runtime.reset();

        /*while(robot.runtime.milliseconds() < 375);

        while((robot.eLineSensor.getVoltage() < 1.5 && robot.wLineSensor.getVoltage() < 1.5)) {

            if(!opModeIsActive()) return;
            robot.drive(180, power/1.05, robot.gyroRot());

        }
        robot.stop();
        while(robot.runtime.milliseconds() < 375);
        while(robot.eLineSensor.getVoltage() < 1.5 && robot.wLineSensor.getVoltage() < 1.5){
            if(!opModeIsActive()) return;
            robot.drive(0, power/1.05, robot.gyroRot());
        }*/


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




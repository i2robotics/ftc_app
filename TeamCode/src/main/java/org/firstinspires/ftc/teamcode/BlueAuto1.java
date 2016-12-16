package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.Helpers.MechNav;

import org.firstinspires.ftc.teamcode.Helpers.RobotControl;

import java.io.OptionalDataException;

/**
 * Created by Nathanael on 11/29/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueAuto1", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
class BlueAuto1 extends LinearOpMode {
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













        while(robot.wallSensor.getVoltage() > .097) {
            System.out.println("Driving at the angle");
            if(!opModeIsActive()) return;
            robot.drive(47, 1, robot.gyroRot());
            telemetry.addData("Gyro: ", robot.gyro.getHeading());
            // Drive it at a 45 degree angle from the starting position
        }
        lineCheck(1);
        while(robot.runtime.milliseconds() < 500) if(!opModeIsActive()) return;
        robot.stop();

        beaconCheckBlue();


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
    public void beaconCheckBlue(){
        double rot;
        double ang;
        robot.runtime.reset();

        boolean setPos = false;
        while (robot.runtime.milliseconds() < 1500) {
            robot.drive(90, .75, 0);
            if (robot.colorEast.blue() > 1 && !setPos) {
                if (!opModeIsActive()) return;
                robot.buttonPressEast.setPosition(.9);

                setPos = true;


                telemetry.addData("Sensor", robot.colorEast.blue());
                telemetry.update();
            }
            else if(robot.colorEast.blue() <= 1){
                robot.buttonPressEast.setPosition(.627);
                setPos = true;

            }
            /*if(robot.colorEast.blue() > 1){
                rot = -.05;
                ang = 93;
            }
            else{
                ang = 85;
                rot = .05;
            }*/

            if (!opModeIsActive()) return;
            /*
            telemetry.addData("Sensor", robot.colorEast.blue());
            telemetry.update();
            if (robot.colorEast.blue() > 1) {
                if (!opModeIsActive()) return;
                robot.buttonPressEast.setPosition(.9);
                telemetry.addData("Sensor", robot.colorEast.blue());
                telemetry.update();
            }
            else if(robot.colorEast.blue() <= 1) {
                robot.buttonPressEast.setPosition(.627);
            }*/
        }
        robot.stop();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 400){
            if(!opModeIsActive()) return;
            robot.drive(270, .8, 0);
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

        while(robot.runtime.milliseconds() < 375);

            while((robot.eLineSensor.getVoltage() < 1.5 && robot.wLineSensor.getVoltage() < 1.5)) {

                if(!opModeIsActive()) return;
                robot.drive(180, power/1.05, robot.gyroRot());

            }


        robot.stop();
    }
    public void lineAlign(double power, double direction){
        double eLineVoltage;
        double wLineVoltage;
        while(robot.eLineSensor.getVoltage() < 1.5 && robot.wLineSensor.getVoltage() < 1.5){
            eLineVoltage = robot.eLineSensor.getVoltage();
            wLineVoltage = robot.wLineSensor.getVoltage();
            if (!opModeIsActive()) return;
            if (wLineVoltage< 1.5) {
                robot.nw.setPower(-power * direction);
                robot.sw.setPower(-power * direction);

            } else {
                robot.nw.setPower(power/2 * direction);
                robot.sw.setPower(power/2 * direction);
            }
            if (eLineVoltage < 1.5) {
                robot.ne.setPower(power/2 * direction);
                robot.se.setPower(power/2 * direction);
            } else {
                robot.ne.setPower(-power/2 * direction);
                robot.se.setPower(-power/2 * direction);

            }
        }
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 500);
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



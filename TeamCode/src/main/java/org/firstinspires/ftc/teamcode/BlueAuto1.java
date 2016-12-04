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
        while (robot.gyro.isCalibrating()) {
            telemetry.addData("Gyro heading", robot.gyro.getHeading());
            telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
            telemetry.update();

        }
        telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
       // robot.se.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        //System.out.println("Starting! Woo Hoo!");
        //Start up the flywheel and position the hood for the firing of the particles
        /*robot.startFlyWheel(8900);
        robot.hood.setPosition(.82);

        //Fire the ball
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 2000){
            if(!opModeIsActive()) return;
            telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
            telemetry.update();
        };
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



        while(robot.wallSensor.getVoltage() > .097) {
            System.out.println("Driving at the angle");
            if(!opModeIsActive()) return;
            robot.drive(45, 1, robot.gyroRot());
            // Drive it at a 45 degree angle from the starting position
            System.out.println(robot.wallSensor.getVoltage());
            telemetry.addData("Gyro Heading", robot.gyro.getHeading());
            telemetry.addData("Ultra Sonic Sensor", robot.wallSensor.getVoltage());
            telemetry.addData("Time", robot.runtime.milliseconds());
            telemetry.update();
        }

        robot.stop();

        System.out.println("Line Checking");
        lineCheck(1, 1000*2);
        brake(75);

        robot.runtime.reset();
        System.out.println("Checking for beacon");
        beaconCheckBlue();

        int encoder = robot.se.getCurrentPosition();
        /*while(Math.abs(encoder)+250 > Math.abs(robot.se.getCurrentPosition())){
            if(!opModeIsActive()) return;
            robot.drive(180, .75, robot.gyroRot());
        }*/

        brake(150);
        robot.runtime.reset();
        robot.drive(180,1,robot.gyroRot());
        while (robot.runtime.milliseconds() < 500) if(!opModeIsActive()) return;
        lineCheck(-.5, 3000*2);
        brake(75);
        beaconCheckBlue();
        robot.stop();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 1000){
            if(!opModeIsActive()) return;
            robot.drive(-90,1, robot.gyroRot());
        }
        robot.stop();






    }
    public void beaconCheckBlue(){
        //

        robot.ne.setPower(.5);
        robot.nw.setPower(.5);
        robot.se.setPower(-.5);
        robot.sw.setPower(-.5);
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 1500) {
//            robot.drive(90, .75, 0);
            if (!opModeIsActive()) return;
            telemetry.addData("Sensor", robot.colorEast.blue());
            telemetry.update();


            if (robot.colorEast.blue() >= 1) {
                if (!opModeIsActive()) return;
                robot.buttonPressEast.setPosition(.9);
                telemetry.addData("Sensor", robot.colorEast.blue());
                telemetry.update();
            }
            else if(robot.colorEast.blue() < 1){
                robot.buttonPressEast.setPosition(.627);
            }
            //while(robot.runtime.milliseconds() - 2000 < 200){}
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
        if (direction > 0) {
            while ((robot.eLineSensor.getVoltage() < 1.5 || robot.wLineSensor.getVoltage() < 1.5) && robot.runtime.milliseconds() <= time) {
                if (!opModeIsActive()) return;
                double power = .1;


                if (robot.wLineSensor.getVoltage() < 1.5) {
                    robot.nw.setPower(power * direction);
                    robot.sw.setPower(power * direction);

                } else {
                    robot.nw.setPower(-1.5 * power * direction);
                    robot.sw.setPower(-1.5 * power * direction);

                }
                if (robot.eLineSensor.getVoltage() < 1.5) {
                    robot.ne.setPower(-power * direction);
                    robot.se.setPower(-power * direction);
                } else {
                    robot.ne.setPower(1.5 * power * direction);
                    robot.se.setPower(1.5 * power * direction);

                }
            }
        } else {
            while ((robot.eLineSensor.getVoltage() < 1.5 || robot.wLineSensor.getVoltage() < 1.5) && robot.runtime.milliseconds() <= time) {
                if (!opModeIsActive()) return;
                double power = .1;


                if (robot.wLineSensor.getVoltage() < 1.5) {
                    robot.nw.setPower(power * direction);
                    robot.sw.setPower(power * direction);

                } else {
                    robot.nw.setPower(-6 * power * direction);
                    robot.sw.setPower(-6 * power * direction);

                }
                if (robot.eLineSensor.getVoltage() < 1.5) {
                    robot.ne.setPower(-power * direction);
                    robot.se.setPower(-power * direction);
                } else {
                    robot.ne.setPower(6 * power * direction);
                    robot.se.setPower(6 * power * direction);
                }
            }
        }
        robot.stop();
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 500) if (!opModeIsActive()) return;
        while((robot.eLineSensor.getVoltage() < 1.5 || robot.wLineSensor.getVoltage()  < 1.5) && robot.runtime.milliseconds() <= time+500) {
            if (!opModeIsActive()) return;
            double power = .1;


            if (robot.wLineSensor.getVoltage() < 1.5) {
                robot.nw.setPower(-(power+.05)*direction);
                robot.sw.setPower(-(power+.05)*direction);

            } else {
                robot.nw.setPower(1.5*(power+.05)*direction);
                robot.sw.setPower(1.5*(power+.05)*direction);

            }
            if (robot.eLineSensor.getVoltage() < 1.5) {
                robot.ne.setPower((power+.05)*direction);
                robot.se.setPower((power+.05)*direction);
            } else {
                robot.ne.setPower(-1.5*(power+.05)*direction);
                robot.se.setPower(-1.5*(power+.05)*direction);

            }
        }
        robot.runtime.reset();

    /*    if(direction > 0){
            while(robot.runtime.milliseconds() < ) {
                robot.drive(180, 1, 0);
            }
        }
        else{
            while(robot.runtime.milliseconds() < 20) {
                robot.drive(0, 1, 0);
            }
        }*/
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



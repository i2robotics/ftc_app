/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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



@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTest1", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled

public class AutoTest1 extends LinearOpMode {
    RobotControl robot;
    int encoder;

    @Override
    public void runOpMode() {
        robot = new RobotControl(this);
            robot.gyro.calibrate();
            while (robot.gyro.isCalibrating()) {


            }
        telemetry.addData("Gyro heading", robot.gyro.getHeading());
        telemetry.addData("Color", robot.colorEast.blue());
        telemetry.update();
        robot.buttonPressEast.setPosition(.9);


        waitForStart();
 /*
        if (robot.gyro.getHeading() != 0) {
            robot.gyro.calibrate();
            while (robot.gyro.isCalibrating()) {
                if(!opModeIsActive()) return;
                telemetry.addData("Gyro heading (POST INIT)", robot.gyro.getHeading());
                telemetry.update();
            }
        }

            while(robot.eLineSensor.getVoltage() < 1.75 && robot.wallSensor.getVoltage() > .1) {
                if(!opModeIsActive()) return;
                robot.drive(41.8, 1, robot.gyroRot());
                // Drive it at a 45 degree angle from the starting position
                System.out.println(robot.wallSensor.getVoltage());

                telemetry.addData("Gyro Heading", robot.gyro.getHeading());
                telemetry.update();
                //robot.setMotors(1,(float) -.4,0);

            }
            brake();
            lineCheck(1);*/
            beaconCheckBlue();
        /*
        encoder =  robot.se.getCurrentPosition();
        while(Math.abs(encoder)+500 > Math.abs(robot.se.getCurrentPosition())) {
            if(!opModeIsActive()) return;
            robot.drive(180, 1, robot.gyroRot());
        }

        lineCheck(-1);
        beaconCheckBlue();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 2000){
            if (!opModeIsActive()) return;
            robot.drive(-90, 1, robot.gyroRot());
        }
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 1000){
            if (!opModeIsActive()) return;

            //robot.drive(-90, 1, robot.gyroRot());
            robot.ne.setPower(.75);
            robot.se.setPower(.75);
        }
*/
           /* robot.runtime.reset();
            while(robot.runtime.milliseconds() < 100){
            }
            beaconCheckBlue();
            robot.stop();

        encoder = robot.se.getCurrentPosition();
        while(Math.abs(encoder)+1000 > Math.abs(robot.se.getCurrentPosition())){
            if(!opModeIsActive()) break;
            robot.drive(180,.5,robot.gyroRot());
        }
        lineCheck();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 100){
        }
        beaconCheckBlue();
        robot.stop();
        *//*encoder = robot.ne.getCurrentPosition();
        while(Math.abs(encoder)+500 > Math.abs(robot.ne.getCurrentPosition())) {
            if(!opModeIsActive()) break;
            robot.drive(-120, 1, robot.gyroRot());
            telemetry.addData("gyro", robot.gyro.getHeading());
            telemetry.update();
            telemetry.addData("enc", robot.ne.getCurrentPosition());
        }*/
        robot.stop();

        }




    public void brake(){
        robot.runtime.reset();
        robot.ne.setPower(Range.clip(-robot.ne.getPower()*100,-1,1));
        robot.se.setPower(Range.clip(-robot.se.getPower()*100,-1,1));
        robot.nw.setPower(Range.clip(-robot.nw.getPower()*100,-1,1));
        robot.sw.setPower(Range.clip(-robot.sw.getPower()*100,-1,1));
        while (robot.runtime.milliseconds() <= 150) {
            if(!opModeIsActive()) return;

            //Just to add some extra time
        }
        robot.ne.setPower(0);
        robot.se.setPower(0);
        robot.sw.setPower(0);
        robot.nw.setPower(0);
    }
    public void lineCheck(int direction){
        while(robot.eLineSensor.getVoltage() < 1.5 || robot.wLineSensor.getVoltage()  < 1.5) {
            if (!opModeIsActive()) return;


                if (robot.eLineSensor.getVoltage() < 1.5) {
                    robot.nw.setPower(.12*direction);
                    robot.sw.setPower(.12*direction);

                } else {
                    robot.nw.setPower(-.12*direction);
                    robot.sw.setPower(-.12*direction);

                }
                if (robot.wLineSensor.getVoltage() < 1.5) {
                    robot.ne.setPower(-.12*direction);
                    robot.se.setPower(-.12*direction);
                } else {
                    robot.ne.setPower(.12*direction);
                    robot.se.setPower(.12*direction);

                }


        }
        robot.runtime.reset();

        robot.stop();
    }
    public void beaconCheckBlue2(){
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 2000){
            if(!opModeIsActive()) return;
            //robot.drive(100, .75, 0.15);
        }

        while(robot.wallSensor.getVoltage() > .2){
            //robot.drive(-90, .5, 0);
        }
        if(robot.colorEast.blue() >=2 && robot.colorEast.red() <= 1){
            robot.buttonPressEast.setPosition(235/255);
        }
        else if(robot.colorEast.blue() <=1 && robot.colorEast.red() >= 2){
            robot.buttonPressEast.setPosition(155/255);
        }
        else{
            robot.drive(90, .25, 0);
        }
        robot.stop();

    }
    public void beaconCheckBlue(){
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 1500) {
            if (!opModeIsActive()) return;



            if (robot.colorEast.blue() >= 2) {
                if (!opModeIsActive()) return;
                robot.buttonPressEast.setPosition(.9);

            }
            else if(robot.colorEast.blue() <= 1){
                robot.buttonPressEast.setPosition(.627);
            }
            //while(robot.runtime.milliseconds() - 2000 < 200){}
            robot.drive(90, .75, 0);



        }
        robot.runtime.reset();/*
        while(robot.runtime.milliseconds() < 200){
            robot.drive(270, .8, 0);
        }*/
        }
        }




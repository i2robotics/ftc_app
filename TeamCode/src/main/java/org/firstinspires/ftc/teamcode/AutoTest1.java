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

public class AutoTest1 extends LinearOpMode {
    //private final HardwareMap hardwareMap;

    double speed;
    RobotControl robot;
    int encoder;

    /* Declare OpMode members. */

    @Override
    public void runOpMode() {
        robot = new RobotControl(this);
        // Wait for the game to start (driver presses PLAY)

        if (robot.gyro.getHeading() != 0) {
            robot.gyro.calibrate();
            while (robot.gyro.isCalibrating()) {
                    if (!opModeIsActive()) return;

                telemetry.addData("gyro", robot.gyro.getHeading());
                telemetry.update();

            }
        }
        waitForStart();
        robot.colorEast.enableLed(true);
        robot.runtime.reset();
        speed = 0.5;
        // run until the end of the match (driver presses STOP

           // double wEopdVal = robot.wEopd.getVoltage();
        if (robot.gyro.getHeading() != 0) {
            robot.gyro.calibrate();
            while (robot.gyro.isCalibrating()) {
                if(!opModeIsActive()) break;
                telemetry.addData("gyro", robot.gyro.getHeading());
                telemetry.update();
                telemetry.addData("enc", robot.ne.getCurrentPosition());
            }
        }
            encoder = robot.ne.getCurrentPosition();
            /*while(Math.abs(encoder)+900> Math.abs(robot.ne.getCurrentPosition())){
                robot.drive(90, .75, robot.gyroRot());
                telemetry.addData("enc", robot.ne.getCurrentPosition());
                telemetry.update();

            }*/
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 1000){
            if(!opModeIsActive()) return;
            telemetry.addData("EOPD", robot.wallSensor.getVoltage());
            telemetry.addData("Both Floor Sensors", "East: " + robot.eLineSensor.getVoltage() + ", West: " + robot.wLineSensor.getVoltage());
            telemetry.addData("color", robot.colorEast.alpha());

            telemetry.update();
            //robot.drive(72.5, .75, robot.gyroRot());
        }

        robot.stop();

            while(robot.eLineSensor.getVoltage() < 1.75 && robot.wallSensor.getVoltage() < .39) {
                if(!opModeIsActive()) return;
                robot.drive(35, 1, robot.gyroRot());
                // Drive it at a 45 degree angle from the starting position
                System.out.println(robot.wallSensor.getVoltage());
                telemetry.addData("Status", "Run Time: " + robot.runtime.toString());
                telemetry.addData("Wall Sensor Status", "Voltage " + robot.wallSensor.getVoltage());
                telemetry.addData("Floor Sensor", robot.eLineSensor.getVoltage());
                telemetry.addData("gyro", robot.gyro.getHeading());
                telemetry.update();
                //robot.setMotors(1,(float) -.4,0);

            }
            brake();
            lineCheck();
            robot.runtime.reset();
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
        /*encoder = robot.ne.getCurrentPosition();
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
    public void lineCheck(){
        while(robot.eLineSensor.getVoltage() > 1.5 || robot.wLineSensor.getVoltage() > 1.5){
            robot.drive(0, .65, 0);
        }
        while(robot.eLineSensor.getVoltage() < 1.5 || robot.wLineSensor.getVoltage()  < 1.5){
            if(!opModeIsActive()) return;


            if(robot.eLineSensor.getVoltage() < 1.5){
                robot.ne.setPower(-.1);
                robot.se.setPower(-.1);
            }
            else{
                robot.ne.setPower(.1);
                robot.se.setPower(.1);
            }
            if(robot.wLineSensor.getVoltage() < 1.5){
                robot.nw.setPower(.1);
                robot.sw.setPower(.1);
            }
            else {
                robot.nw.setPower(-.1);
                robot.sw.setPower(-.1);
            }
        }
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
        if(robot.colorEast.blue() >=2){
            robot.buttonPressEast.setPosition(235/255);
        }
        else{
            robot.buttonPressEast.setPosition(155/255);
        }
        robot.stop();

    }
    public void beaconCheckBlue(){
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 1000){
            if(!opModeIsActive()) return;
            robot.drive(90, .65, 0);
        }
        robot.stop();
        while(robot.runtime.milliseconds() > 2000){
            if(opModeIsActive()) return;
            robot.drive(-90, .75, 0);
        }
        if(robot.colorEast.blue() >=2){
            robot.buttonPressEast.setPosition(235/255);
        }
        else{
            robot.buttonPressEast.setPosition(155/255);
        }

    }
        }




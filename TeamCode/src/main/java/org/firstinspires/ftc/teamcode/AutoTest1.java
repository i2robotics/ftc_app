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

//import org.firstinspires.ftc.teamcode.Helpers.MechNav;

import org.firstinspires.ftc.teamcode.Helpers.RobotControl;

import java.io.OptionalDataException;



@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTest1", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class AutoTest1 extends LinearOpMode {
    //private final HardwareMap hardwareMap;

    double speed;
    RobotControl robot;

    /* Declare OpMode members. */

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotControl(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.runtime.reset();
        speed = 0.5;
        // run until the end of the match (driver presses STOP

           // double wEopdVal = robot.wEopd.getVoltage();
        if (robot.gyro.getHeading() != 0) {
            robot.gyro.calibrate();
            while (robot.gyro.isCalibrating()) {
                telemetry.addData("gyro", robot.gyro.getHeading());
                telemetry.update();
            }
        }

            while(robot.eLineSensor.getVoltage() < 1.5 && robot.wallSensor.getVoltage() < .4) {

                // Drive it at a 45 degree angle from the starting position
                telemetry.addData("Status", "Run Time: " + robot.runtime.toString());
                telemetry.addData("Wall Sensor Status", "Voltage " + robot.wallSensor.getVoltage());
                telemetry.addData("gyro",robot.gyro.getHeading());
                telemetry.update();
                //robot.setMotors(1,(float) -.4,0);
                robot.drive(77, .75, robot.gyroRot());
            }
            robot.brake();
            robot.lineCheck();
            robot.runtime.reset();
            while(robot.runtime.milliseconds() < 100){
            }
            robot.beaconCheckBlue();
            robot.stop();
        int encoder1 = robot.se.getCurrentPosition();
        while(Math.abs(encoder1)+1000 > Math.abs(robot.se.getCurrentPosition())){
            robot.drive(0,.5,robot.gyroRot());
        }
        robot.lineCheck();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 100){
        }
        robot.beaconCheckBlue2();
        robot.stop();
        robot.runtime.reset();
        while(robot.runtime.milliseconds() < 7000) {
            robot.drive(-120, 1, 0);
        }
        }

            // Possibly add safeguard, so it wont hit the wall.





/*
            if(robot.eEopd.getVoltage() > 1.5 || robot.wEopd.getVoltage() > 1.5){
                //Align the robot here with he line
            }
            while(eEopdVal > 1.5 && wEopdVal > 1.5){

                
            }*/






             // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }




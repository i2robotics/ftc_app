package org.firstinspires.ftc.teamcode.Helpers;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;




import org.firstinspires.ftc.teamcode.Helpers.HW;
import org.firstinspires.ftc.teamcode.TeleOpTest1;

/**
 * Driving functions for a Mecanum drive train in autonomous.
 */
public class RobotControl{
    private final HardwareMap hardwareMap;
    private LinearOpMode linearOpMode = null;
    public DcMotor ne, se, sw, nw, harvester, flyWheelEast, flyWheelWest, capBall;
    public ModernRoboticsI2cColorSensor colorEast, colorWest;
    public CRServo ballFeeder, capBallG;
    public Servo hood, buttonPressEast, liftStopEast, liftStopWest, buttonPressWest;
    public AnalogInput eLineSensor,wLineSensor, wallSensor, feedSwitch, SaddleSwitch, wallSensorWest;
    public ElapsedTime runtime;
    public GyroSensor gyro;
    public ElapsedTime counter;

    double eastCount;
    double westCount;
    //wEopd;

    public RobotControl(LinearOpMode linearOpMode) {
        hardwareMap = linearOpMode.hardwareMap;
        this.ne = this.hardwareMap.dcMotor.get("ne");
        this.se = this.hardwareMap.dcMotor.get("se");
        this.sw = this.hardwareMap.dcMotor.get("sw");
        this.nw = this.hardwareMap.dcMotor.get("nw");
        this.flyWheelEast = this.hardwareMap.dcMotor.get("flyWheelEast");
        this.flyWheelWest = this.hardwareMap.dcMotor.get("flyWheelWest");
        this.harvester = this.hardwareMap.dcMotor.get("harvester");
        this.hood = this.hardwareMap.servo.get("hood");
        this.buttonPressEast = this.hardwareMap.servo.get("buttonPressEast");
        this.buttonPressWest = this.hardwareMap.servo.get("bpw");
        this.ballFeeder = this.hardwareMap.crservo.get("ballFeeder");
        this.colorEast = (ModernRoboticsI2cColorSensor) this.hardwareMap.colorSensor.get("colorEast");
        this.gyro = this.hardwareMap.gyroSensor.get("gyro");
        this.colorWest = (ModernRoboticsI2cColorSensor) this.hardwareMap.colorSensor.get("colorWest");
        this.wallSensor = this.hardwareMap.analogInput.get("ultra");
        this.capBallG = this.hardwareMap.crservo.get("cbg");
        this.wallSensorWest = this.hardwareMap.analogInput.get("ultraWest");
        this.SaddleSwitch = this.hardwareMap.analogInput.get("ss");
        this.capBall = this.hardwareMap.dcMotor.get("cb");
        this.eLineSensor = this.hardwareMap.analogInput.get("eLineSensor");
        this.wLineSensor = this.hardwareMap.analogInput.get("wLineSensor");
        this.liftStopEast = this.hardwareMap.servo.get("lse");
        this.liftStopWest = this.hardwareMap.servo.get("lsw");

        this.runtime = new ElapsedTime();
        this.counter = new ElapsedTime();
        this.flyWheelWest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.flyWheelEast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //gyro.calibrate();
        this.liftStopWest.setPosition(0.42);
        this.liftStopEast.setPosition(.5);
        this.buttonPressEast.setPosition(.9);
        this.buttonPressWest.setPosition(.672);
        this.colorEast.setI2cAddress(I2cAddr.create8bit(0x4c));
        this.colorWest.setI2cAddress(I2cAddr.create8bit(0x5c));
        colorEast.enableLed(false);
        colorWest.enableLed(false);
        //flyWheelWest.setDirection(DcMotorSimple.Direction.REVERSE);
        //flyWheelEast.setDirection(DcMotorSimple.Direction.FORWARD);



        //buttonPressWest.setPosition(235 / 255);
    }

//    public void drive()

    public void drive(double angle, double power, double rot) {
        setMotors((float)(power*Math.sin(Math.toRadians(angle))), (float)(power*-Math.cos(Math.toRadians(angle))), (float)rot);
    }
    public double scale(double scaled) {
        double scaler = 0.8;
        return (scaler*Math.pow(scaled, 3) + ( 1 - scaler) * scaled);
    }
    public void setHarvester(double pow) {
        harvester.setPower(pow);
    }

    public void setMotors(float x, float y, float rot) {
        double drive = (double) -y, strafe = (double) x, spin = (double) rot;
        double nePower, nwPower, sePower, swPower;
        nwPower = Range.clip(drive + strafe + spin, -1, 1);
        swPower = Range.clip(drive - strafe + spin, -1, 1);
        nePower = Range.clip(drive - strafe - spin, -1, 1);
        sePower = Range.clip(drive + strafe - spin, -1, 1);
        nwPower = scale(nwPower);
        nePower = -scale(nePower);
        swPower = scale(swPower);
        sePower = -scale(sePower);

        //from here on is just setting motor values
        ne.setPower(nePower);
        se.setPower(sePower);
        sw.setPower(swPower);
        nw.setPower(nwPower);

    }
 /*   public void brake(){
        runtime.reset();
        ne.setPower(Range.clip(-ne.getPower()*100,-1,1));
        se.setPower(Range.clip(-se.getPower()*100,-1,1));
        nw.setPower(Range.clip(-nw.getPower()*100,-1,1));
        sw.setPower(Range.clip(-sw.getPower()*100,-1,1));
        while (runtime.milliseconds() <= 150) {
            if(!linearOpMode.opModeIsActive()) return;

            //Just to add some extra time
        }
        ne.setPower(0);
        se.setPower(0);
        sw.setPower(0);
        nw.setPower(0);
    }*/
    public void stop(){
        ne.setPower(0);
        se.setPower(0);
        sw.setPower(0);
        nw.setPower(0);
    }
   public double startFlyWheel(double speed){
        int CPS;
        CPS = (int) (28*((speed*6)/7))/60;

        /*if (counter.milliseconds() >= 50) {
           flyWheelEast.setPower(-Range.clip((float) Math.pow(10,-10)*Math.pow(((1000*(flyWheelEast.getCurrentPosition()-eastCount)/(counter.milliseconds()))-CPS),3), -1, 1));
           flyWheelWest.setPower(Range.clip((float) Math.pow(10,-10)*Math.pow((((1000*Math.abs(flyWheelWest.getCurrentPosition()-westCount)/(counter.milliseconds()))-CPS)),3), -1, 1));
           counter.reset();
            eastCount = flyWheelEast.getCurrentPosition();
            westCount = flyWheelWest.getCurrentPosition();
        }
        return (2500*((flyWheelWest.getCurrentPosition()-westCount)/(counter.milliseconds())));*/
        flyWheelWest.setMaxSpeed((int) (CPS));
        flyWheelEast.setMaxSpeed((int)(CPS));
        flyWheelWest.setPower(-1);
        flyWheelEast.setPower(1);
        return 1;
    }
    public void stopFlyWheel(){
        startFlyWheel(0);
    }

    public void beaconCheckBlue2(){
        runtime.reset();
        while(runtime.milliseconds() < 2000){
            if(!linearOpMode.opModeIsActive()) return;
            drive(100, .75, 0.15);
        }
        stop();
        /*while(wallSensor.getVoltage() > .55){
            drive(-90, .75, 0);
        }*/
        /*if(colorWest.blue() >=2){
            buttonPressWest.setPosition(235/255);
        }
        else{
            buttonPressWest.setPosition(155/255);
        }*/

    }
    public void lineCheck(){

        while(eLineSensor.getVoltage() < 1.75 || wLineSensor.getVoltage()  < 1.75){
            if(!linearOpMode.opModeIsActive()) return;
            if(eLineSensor.getVoltage() < 1.5){
                ne.setPower(-.1);
                se.setPower(-.1);
            }
            else{
                ne.setPower(.1);
                se.setPower(.1);
            }
            if(wLineSensor.getVoltage() < 1.5){
                nw.setPower(.1);
                sw.setPower(.1);
            }
            else {
                nw.setPower(-.1);
                sw.setPower(-.1);
            }
        }
        stop();


    }
    public double gyroRot(){
        if (gyro.getHeading() == 1 || gyro.getHeading() == 359 || gyro.getHeading() == 0) {
            return 0;
        } else {
            return 1 / (.1 * (gyro.getHeading() - 180));
        }
    }








}

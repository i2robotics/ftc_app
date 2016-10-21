package org.firstinspires.ftc.teamcode.Helpers;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Helpers.HW;
import org.firstinspires.ftc.teamcode.TeleOpTest1;

/**
 * Driving functions for a Mecanum drive train in autonomous.
 */
public class RobotControl {
    private final HardwareMap hardwareMap;
    public DcMotor ne, se, sw, nw, harvester;

    /**
     * Constructor
     *
     * @param hardwareMap from the OpMode. Used to access the hardware.
     */
    public RobotControl(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.ne = this.hardwareMap.dcMotor.get("ne");
        this.se = this.hardwareMap.dcMotor.get("se");
        this.sw = this.hardwareMap.dcMotor.get("sw");
        this.nw = this.hardwareMap.dcMotor.get("nw");
        this.harvester = this.hardwareMap.dcMotor.get("hv");
    }

//    public void drive()

    public void drive(double angle, double power, double rot, HardwareMap hardwareMap) {
        setMotors((float)(power*Math.cos(angle)), (float)(power*Math.sin(angle)), (float)rot, hardwareMap);
    }
    public double scale(double scaled) {
        double scaler = 0.8;
        return (scaler*Math.pow(scaled, 3) + ( 1 - scaler) * scaled);
    }
    public void setHarvester(double pow) {
        harvester.setPower(pow);
    }
    public void setMotors(float x, float y, float rot, HardwareMap hardwareMap) {
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


}

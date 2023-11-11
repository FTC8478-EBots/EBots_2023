package org.firstinspires.ftc.teamcode.drive.ebotsmanip;
import android.os.Handler;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.drive.CenterStageConstants.bluePixelX;
import static org.firstinspires.ftc.teamcode.drive.CenterStageConstants.bluePixelY;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.ARMMOTORHANGINGPOSITION;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.ARMMOTORPREPAREHANGINGPOSITION;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.BASEMOTORHANGINGPOSITION;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.BASEMOTORPREPAREHANGINGPOSITION;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.lowerArmLengthInches;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.lowerArmTicksToDegrees;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.pixelRowsLower;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.pixelRowsUpper;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.upperArmLengthInches;
import static org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants.upperArmTicksToDegrees;

import static java.lang.Math.cos;
import static java.lang.Math.decrementExact;
import static java.lang.Math.sqrt;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants;

import java.lang.Math;
@Config
public class Lift {
    DcMotorEx baseMotor;
    DcMotorEx armMotor;
    int pixelLock = 0;
    Servo wristServo;
    Telemetry telemetry;
    Servo handServo;
    boolean stowed = true;
    boolean wristStability = true;
    boolean manualOverride = false;
    public static double wristFieldCentricAngle = 0;
    double wristFieldCentricAngleFactor = 62.0/90.0;
    public static double wristFactor = 0.9;
    public static double wristZeroPosition = .18; //.75 real, .35 tuning.
    double baseMotorTarget = 0;
    double armMotorTarget = 0;
    final int targetTolerance = 80;
    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        baseMotor = hardwareMap.get(DcMotorEx.class, "baseMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        handServo = hardwareMap.get(Servo.class,"handServo");

       // baseMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        baseMotor.setMotorEnable();
        baseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMotorEnable();

       // baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setTargetPosition(0);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotor.setMotorEnable();
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMotorEnable();
    }
        public void init(LinearOpMode linearOpMode){
        //baseMotor.setVelocity(.2);
            baseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            baseMotor.setPower(-.1);
        armMotor.setPower(-.1);
        baseMotor.setMotorEnable();
        armMotor.setMotorEnable();
        linearOpMode.sleep(4000);
       /* while(baseMotor.getVelocity()>50 || armMotor.getVelocity()>50) {
            if (baseMotor.getVelocity()<50)
                baseMotor.setMotorDisable();
            if (armMotor.getVelocity()<50)
                armMotor.setMotorDisable();
            telemetry.addData("baseMotorVelocity",baseMotor.getVelocity());
            telemetry.addData("armMotorVelocity",armMotor.getVelocity());
            telemetry.update();
            linearOpMode.opModeIsActive();

        }
        */
            baseMotor.setPower(0);
            armMotor.setPower(0);
            linearOpMode.sleep(200);

            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }
    public void setPositionControl() {
        if (manualOverride) {
            manualOverride = false;
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            baseMotor.setTargetPosition(baseMotor.getCurrentPosition());
            baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void setManualControl() {
        if (!manualOverride) {
            manualOverride = true;
            armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            armMotor.setMotorEnable();
            baseMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            baseMotor.setMotorEnable();
        }
    }
     public void setBaseMotorVelocity(double velocity){
        if (stowed) return;
        baseMotor.setPower(-velocity);
     }
    public void setArmMotorVelocity(double velocity){
        if (stowed) return;
        armMotor.setPower(-velocity);
    }
    public double moveLowerArmToAngle(double angle) {
        baseMotor.setTargetPosition((int) (angle * lowerArmTicksToDegrees));
        baseMotor.setPower(.5);
        return baseMotor.getCurrentPosition()/lowerArmTicksToDegrees;
    }
    public void dispense1Pixel() {
        if (pixelLock < 2) {pixelLock += 1;}
        handServo.setPosition(pixelLock/2);
    }
    public void loadPixels() {
        pixelLock = 2;
        handServo.setPosition(1);
    }
    //lock into position 0, dispense adds 1, load puts to 2
    public void lockPixels() {
        pixelLock = 0;
        handServo.setPosition(0.5);
    }
    public double getLowerArmAngleDegrees() {
        return baseMotor.getCurrentPosition()/lowerArmTicksToDegrees;
    }
    public double getLowerArmPosition() {
        return baseMotor.getCurrentPosition();
    }
    public Pose2d getLowerArmXY(){
       return new Pose2d(lowerArmLengthInches * Math.cos(Math.toRadians(getLowerArmAngleDegrees())),
        lowerArmLengthInches * Math.sin(Math.toRadians(getLowerArmAngleDegrees())));
    }
    public double moveUpperArmToAngle(double angle) {
        armMotor.setTargetPosition((int) (angle * upperArmTicksToDegrees));
        armMotor.setPower(.5);
        return armMotor.getCurrentPosition()/upperArmTicksToDegrees;
    }

    public double[] getAngleSettingsDegrees(int row, double horizontaldistancetoboardbottom, double distanceoffground){
        double a1 = horizontaldistancetoboardbottom;
        double a2 = distanceoffground;
        int base = lowerArmLengthInches;
        int arm = upperArmLengthInches;
        int r = row;
        double a = a1;
        double pixelx = (2.25*r+5)/Math.sqrt(3);
        double pixely = (2.25*r+5);
        double k =(-9216*a*a*a + 6912*sqrt(3)*a*a*r + 15360*sqrt(3)*a*a - 31104*a*r*r + 100224*a*r - 1587264*a - 46656*sqrt(3)*r*r*r - 132192*sqrt(3)*r*r + 1119312*sqrt(3)*r + 2628160*sqrt(3));
        double p = (9216*a*a*a - 6912*sqrt(3)*a*a*r - 15360*sqrt(3)*a*a - sqrt(k*k - 4*(9216*a*a - 13824*sqrt(3)*a*r - 30720*sqrt(3)*a + 62208*r*r + 38016*r + 81984)*(2304*a*a*a*a + 15552*a*a*r*r - 50112*a*a*r + 793632*a*a + 104976*r*r*r*r + 128304*r*r*r - 12001284*r*r + 1226628*r + 66706873)) + 31104*a*r*r - 100224*a*r + 1587264*a + 46656*sqrt(3)*r*r*r + 132192*sqrt(3)*r*r - 1119312*sqrt(3)*r - 2628160*sqrt(3))/(2*(9216*a*a - 13824*sqrt(3)*a*r - 30720*sqrt(3)*a + 62208*r*r + 38016*r + 81984));
        double q = (a2+sqrt(144-(p-a1)*(p-a1)));
        double lowerAngle = (Math.asin((q-a2)/12))*180/Math.PI;
        double dbase_arm = base;
        double darm_pixel = arm;
        double dbase_pixel = Math.sqrt((a1-pixelx)*(a1-pixelx)+(a2-pixely)*(a2-pixely));
        double upperAngle = Math.acos((dbase_pixel*dbase_pixel-dbase_arm*dbase_arm-darm_pixel*darm_pixel)/(-2*dbase_arm*dbase_pixel))*180/Math.PI;
        double[] angles = {lowerAngle, upperAngle};
        return (angles);
    }
    /*public void retractMotors() throws InterruptedException {
        setBaseMotorVelocity(-0.1);
        setArmMotorVelocity(-0.1);
        final Handler handler = new Handler();
        handler.postDelayed(new void Runnable() {
            public void run() {
                setBaseMotorVelocity(0);
                setArmMotorVelocity(0);
            }
        }, 5000);
    }*/


    public double getUpperArmAngleDegrees() {
        return armMotor.getCurrentPosition()/upperArmTicksToDegrees;
    }
    public double getUpperArmPosition() {
        return armMotor.getCurrentPosition();
    }
    public Pose2d getUpperArmXY() {
        return new Pose2d(upperArmLengthInches * Math.cos(Math.toRadians(getUpperArmAngleDegrees()))+
        lowerArmLengthInches * Math.cos(Math.toRadians(getLowerArmAngleDegrees())),
        upperArmLengthInches * Math.sin(Math.toRadians(getUpperArmAngleDegrees()))+
                lowerArmLengthInches * Math.sin(Math.toRadians(getLowerArmAngleDegrees())));


    }
    public Pose2d moveArmToPosition(double y, double z) {
        return null;

    }
    public Pose2d getArmPosition() {
        //return lift.getPosition();
        return null;
    }

    public double moveWristToAngle() {
        return 0;
    }

    //public ???? getWristAngle() {}

    //Include position and orientation
    public double movePixelToPositionAngle(Vector2d pixelOrientation) {
        return 0;
    }

    public void setHeight(double i) {
    }

    //public ???? getRelativePixelPosition() {}

    public void prepareToHang() {
        baseMotor.setTargetPosition((int) Math.round(BASEMOTORPREPAREHANGINGPOSITION));
        armMotor.setTargetPosition((int) Math.round(ARMMOTORPREPAREHANGINGPOSITION));
        baseMotor.setPower(.5);
        armMotor.setPower(.5);

    }
    public void hang() {
        baseMotor.setTargetPosition(BASEMOTORHANGINGPOSITION);
        armMotor.setTargetPosition(ARMMOTORHANGINGPOSITION);
        baseMotor.setPower(.6);
        armMotor.setPower(.6);
    }
    public void pixelSetArmPos(int rowfunc) {
        baseMotor.setTargetPosition(pixelRowsLower[rowfunc]);
        armMotor.setTargetPosition(pixelRowsUpper[rowfunc]);
    }
    boolean isAtTarget() {
        return (Math.abs(baseMotor.getCurrentPosition()-baseMotorTarget*lowerArmTicksToDegrees) + Math.abs(armMotor.getCurrentPosition()-armMotorTarget*upperArmTicksToDegrees))<targetTolerance;


    }
    void moveArmToAngle(double baseAngle, double armAngle) {
        if (manualOverride) return;
        baseMotorTarget = baseAngle;
        armMotorTarget = armAngle;
        moveLowerArmToAngle(baseMotorTarget);
        moveUpperArmToAngle(armMotorTarget);
        while(!manualOverride && !isAtTarget()) {

            try {
                telemetry.addLine("Driving To Target");
                Thread.sleep(100);
            } catch (InterruptedException e) {
            }
        }
    }

    public void stowArm() {
        if (stowed) return;
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                moveArmToAngle(-50,50);
                setwristFieldCentricAngle(0);
         //       moveArmToAngle(-50,30);
                moveArmToAngle(-15,10);


                stowed = true;
            }
        };
        new Thread(runnable).start();
    }
    public void pullArm() {
        if (!stowed) return;
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                lockPixels();
                wristEnableStability(true);
                moveArmToAngle(-45,10);
                moveArmToAngle(-50,30);
                setwristFieldCentricAngle(60);
                moveArmToAngle(-50,50);

                stowed = false;
            }
        };
        new Thread(runnable).start();
    }
    public void setwristFieldCentricAngle(double a) {
        wristFieldCentricAngle = a;
    }
    private void setWristAngle(double a) {
        double wristAngle = wristZeroPosition + (a-wristFieldCentricAngle*wristFieldCentricAngleFactor)/180.0;
        wristServo.setPosition(1.0 - (wristAngle));
    }
    public void wristEnableStability(boolean value) {
        wristStability = value;
    }

    public void stabilizeWrist() {
        if (wristStability) {
            setWristAngle((getLowerArmAngleDegrees()+getUpperArmAngleDegrees())*wristFactor);
            //telemetry.addData("Wrist angle setting to", getLowerArmAngleDegrees()+getUpperArmAngleDegrees());
        }
    }
    /*public void prepareColumn (int col) {
        if (!(drive.getPoseEstimate().getX() == bluePixelX & drive.getPoseEstimate().getY() == bluePixelY[column - 1])) {
            Trajectory move = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(bluePixelX, bluePixelY[column - 1], Math.toRadians(0)))
                    .build();
            drive.followTrajectory(move);
        }
    }*/
    }


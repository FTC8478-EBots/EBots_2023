package org.firstinspires.ftc.teamcode.drive.ebotsmanip;
import android.os.Handler;
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
import static java.lang.Math.sqrt;
import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants;

import java.lang.Math;

public class Lift {
    DcMotorEx baseMotor;
    DcMotorEx armMotor;

    Servo wristServo;

    Servo handServo;
    public Lift(DcMotorEx baseMotor, DcMotorEx armMotor,Servo wristServo, Servo handServo) {
        this.baseMotor = baseMotor;
        this.armMotor = armMotor;
        this.wristServo = wristServo;
        this.handServo = handServo;
       // baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setTargetPosition(0);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor.setMotorEnable();
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMotorEnable();
    }
     public void setBaseMotorVelocity(double velocity){
        //baseMotor.setPower(-velocity);
     }
    public double moveLowerArmToAngle() {
        return 0;
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
    public double moveUpperArmToAngle() {
        return baseMotor.getCurrentPosition()/upperArmTicksToDegrees;
    }
    public void setArmMotorVelocity(double velocity){
        //armMotor.setPower(velocity);
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
    public void setArmPos(int rowfunc) {
        baseMotor.setTargetPosition(pixelRowsLower[rowfunc]);
        armMotor.setTargetPosition(pixelRowsUpper[rowfunc]);
    }
    public static void prepareColumn(int col) {

    }
}


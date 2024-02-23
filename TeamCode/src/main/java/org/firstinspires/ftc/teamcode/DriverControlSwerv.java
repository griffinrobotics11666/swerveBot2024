/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

/*
 * Demonstrates an empty iterative OpMode
 */
@TeleOp(name = "Concept: NullOp", group = "Concept")
//@Disabled
@Config
public class DriverControlSwerv extends OpMode {
  int L = 11;
  int W = 11;
  int motorflipperR= -1;
  int motorflipperL= 1;

  static double integralsum = 0;
  public static double kp = 0.01;
  public static double ki = 0;
  public static double kd = 0;
  private double lastError = 0;
  /*public static double x1 = 0;
  public static double x2 = 0;
  public static double y1 = 0;
   */
  ElapsedTime timer = new ElapsedTime();


  DcMotor frontLeftMotor;
  DcMotor backLeftMotor ;
  DcMotor frontRghtMotor;
  DcMotor backRightMotor;

  CRServo frontLeftServo;
  CRServo frontRightServo;
  CRServo backLeftServo;
  CRServo backRightServo;

  AnalogInput backleftangle;
  AnalogInput backrightangle;
  AnalogInput frontleftangle;
  AnalogInput frontrightangle;

  /*
  all measurments are in inches
   */

  private ElapsedTime runtime = new ElapsedTime();




  /**
   * This method will be called once, when the INIT button is pressed.
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
    backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftDrive");
    frontRghtMotor = hardwareMap.get(DcMotor.class, "FrontRightDrive");
    backRightMotor = hardwareMap.get(DcMotor.class, "BackRightDrive");

    frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftTurn");
    frontRightServo = hardwareMap.get(CRServo.class, "frontRightTurn");
    backLeftServo = hardwareMap.get(CRServo.class, "backLeftTurn");
    backRightServo = hardwareMap.get(CRServo.class, "backRightTurn");

    backleftangle = hardwareMap.get(AnalogInput.class, "back_left_angle");
    backrightangle = hardwareMap.get(AnalogInput.class, "back_right_angle");
    frontleftangle = hardwareMap.get(AnalogInput.class, "front_left_angle");
    frontrightangle = hardwareMap.get(AnalogInput.class, "front_right_angle");

  }

  /**
   * This method will be called repeatedly during the period between when
   * the init button is pressed and when the play button is pressed (or the
   * OpMode is stopped).
   */
  @Override
  public void init_loop() {

  }

  /**
   * This method will be called once, when the play button is pressed.
   */
  @Override
  public void start() {
    runtime.reset();


  }

  /**
   * This method will be called repeatedly during the period between when
   * the play button is pressed and when the OpMode is stopped.
   */
  @Override
  public void loop() {

    telemetry.addData("Status", "Run Time: " + runtime.toString());
    double x1 = gamepad1.left_stick_x;
    double x2 = gamepad1.right_stick_x;
    double y1 = -gamepad1.left_stick_y;

    double anglebls = -backleftangle.getVoltage() * 360/ backleftangle.getMaxVoltage() + 138; //wrap angle
    double anglebrs = backrightangle.getVoltage() * 360/ backrightangle.getMaxVoltage() - 44;
    double anglefrs = frontrightangle.getVoltage() * 360/ frontrightangle.getMaxVoltage() - 91;
    double anglefls = -frontleftangle.getVoltage() * 360/ frontleftangle.getMaxVoltage() + 354;

    TelemetryPacket servoCurentPositon = new TelemetryPacket();
    servoCurentPositon.put("angle of back left servo:", anglebls);
    servoCurentPositon.put("angle of front right servo:", anglefrs);
    servoCurentPositon.put("angle of back right servo:", anglebrs);
    servoCurentPositon.put("angle of front left servo:", anglefls);
    FtcDashboard dashboard = FtcDashboard.getInstance();
    dashboard.sendTelemetryPacket(servoCurentPositon);


    double r = Math.sqrt((L * L) + (W * W));


    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    double backRightSpeed = Math.sqrt((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

    double backRightTurnAngle = (Math.atan2(a, d) / Math.PI)*180;
    double backLeftTurnAngle = (Math.atan2(a, c) / Math.PI)*180;
    double frontRightTurnAngle = (Math.atan2(b, d) / Math.PI)*180;
    double frontLeftTurnAngle = (Math.atan2(b, c) / Math.PI)*180;

    Pair<Double, Double> PIDOutputBR = updatePID(anglebrs, backRightTurnAngle);
    Pair<Double, Double> PIDOutputFR = updatePID(anglefrs, frontRightTurnAngle);
    Pair<Double, Double> PIDOutputBL = updatePID(anglebls, backLeftTurnAngle);
    Pair<Double, Double> PIDOutputLF = updatePID(anglefls, frontLeftTurnAngle);

    /*frontLeftMotor.setPower(motorflipperL*frontLeftSpeed);
    backRightMotor.setPower(motorflipperL*backRightSpeed);
    backLeftMotor.setPower(motorflipperR*backLeftSpeed);
   frontRghtMotor.setPower(motorflipperR*frontRightSpeed);*/

    backLeftServo.setPower(PIDOutputBL.first);
    backRightServo.setPower(PIDOutputBR.first);
    frontRightServo.setPower(PIDOutputFR.first);
    frontLeftServo.setPower(PIDOutputLF.first);
    TelemetryPacket wheretobe = new TelemetryPacket();
    wheretobe.put("back left servo want",backLeftTurnAngle);
    wheretobe.put("back right servo want", backRightTurnAngle);
    wheretobe.put("front right servo want",frontRightTurnAngle);
    wheretobe.put("front left angle want",frontLeftTurnAngle);
    wheretobe.put("testLeftstickx", x1);
    wheretobe.put("testRightstick",x2);
    wheretobe.put("testLeftstick", y1);
    wheretobe.put("PIDcontrollerFR:",PIDOutputFR.first);
    wheretobe.put("PIDcontrollerFL:",PIDOutputLF.first);
    wheretobe.put("PIDcontrollerBR:",PIDOutputBR.first);
    wheretobe.put("PIDcontrollerBL:",PIDOutputBL.first);
    wheretobe.put("errorFR:",PIDOutputFR.second);
    wheretobe.put("errorFL:",PIDOutputLF.second);
    wheretobe.put("errorBR:",PIDOutputBR.second);
    wheretobe.put("errorBL:",PIDOutputBL.second);
    dashboard.sendTelemetryPacket(wheretobe);


  }

  public Pair<Double, Double> updatePID(double servoAngle, double targetAngle) {
    servoAngle = angleWrap(servoAngle);
    targetAngle = angleWrap(targetAngle);
    double error = angleWrap(targetAngle - servoAngle);
    error = angleWrap(error);
    integralsum += error * timer.seconds();
    double derivative = (error - lastError) / timer.seconds();
    lastError = error;

    timer.reset();
    return new Pair<>((error * kp)+ (integralsum * ki) + (derivative * kd), error);
  }
    public double angleWrap(double degree) {
      while (degree > 180) {
        degree -= 360;
      }
      while (degree < -180) {
        degree += 360;
      }
      return degree;
  }

  /**
   * This method will be called once, when this OpMode is stopped.
   * <p>
   * Your ability to control hardware from this method will be limited.
   */
  @Override
  public void stop() {

  }
}

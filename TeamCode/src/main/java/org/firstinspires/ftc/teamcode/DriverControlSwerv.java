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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

/*
 * Demonstrates an empty iterative OpMode
 */
@TeleOp(name = "Concept: NullOp", group = "Concept")
//@Disabled
public class DriverControlSwerv extends OpMode {
  int L = 9;
  int W = 9;

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
    DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
    DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftDrive");
    DcMotor frontRghtMotor = hardwareMap.get(DcMotor.class, "FrontRightdrive");
    DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "BackRightDrive");

    Servo frontLeftServo = hardwareMap.get(Servo.class, "frontLeftTurn");
    Servo frontRightServo = hardwareMap.get(Servo.class, "frontRightTurn");
    Servo backLeftServo = hardwareMap.get(Servo.class, "backLeftTurn");
    Servo backRightServo = hardwareMap.get(Servo.class, "backRightTurn");

    double x1 = gamepad1.left_stick_x;
    double x2 = gamepad1.right_stick_x;
    double y1 = gamepad1.left_stick_y;

    double r = Math.sqrt((L * L) + (W * W));
    y1 *= -1;

    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    double backRightSpeed = Math.sqrt((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

    double backRightAngle = Math.atan2(a, d) / Math.PI;
    double backLeftAngle = Math.atan2(a, c) / Math.PI;
    double frontRightAngle = Math.atan2(b, d) / Math.PI;
    double frontLeftAngle = Math.atan2(b, c) / Math.PI;

    frontLeftMotor.setPower(frontLeftSpeed);
    backRightMotor.setPower(backRightSpeed);
    backLeftMotor.setPower(backLeftSpeed);
    frontRghtMotor.setPower(frontRightSpeed);
  }


  /**
   * This method will be called repeatedly during the period between when
   * the play button is pressed and when the OpMode is stopped.
   */
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
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

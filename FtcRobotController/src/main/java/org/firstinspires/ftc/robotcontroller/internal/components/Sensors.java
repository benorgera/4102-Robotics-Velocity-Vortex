package org.firstinspires.ftc.robotcontroller.internal.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcontroller.internal.Utils;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by benorgera on 10/25/16.
 */
public class Sensors {


    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration acceleration;

    public Sensors(BNO055IMU imu) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        this.imu = imu;
        imu.initialize(parameters);
    }
    
    public void startSensors() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    
    private void getCalibrationString() {
        
        
    }

    private void getReadings() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        acceleration = imu.getGravity();
    }


    private String getFruitStatus() {
        return "status: " + imu.getSystemStatus().toShortString() + ", calib: " + imu.getCalibrationStatus().toString();
    }

    private String getData() {
        return "gyro: (" + Utils.formatAngle(angles.angleUnit, angles.firstAngle) + ", " + Utils.formatAngle(angles.angleUnit, angles.secondAngle) + ", " + Utils.formatAngle(angles.angleUnit, angles.thirdAngle) + ")";
    }

//
//    telemetry.addLine()
//            .addData("heading", new Func<String>() {
//        @Override public String value() {
//            return formatAngle(angles.angleUnit, angles.firstAngle);
//        }
//    })
//            .addData("roll", new Func<String>() {
//        @Override public String value() {
//            return formatAngle(angles.angleUnit, angles.secondAngle);
//        }
//    })
//            .addData("pitch", new Func<String>() {
//        @Override public String value() {
//            return formatAngle(angles.angleUnit, angles.thirdAngle);
//        }
//    });
//
//    telemetry.addLine()
//            .addData("grvty", new Func<String>() {
//        @Override public String value() {
//            return acceleration.toString();
//        }
//    })
//            .addData("mag", new Func<String>() {
//        @Override public String value() {
//            return String.format(Locale.getDefault(), "%.3f",
//                    Math.sqrt(acceleration.xAccel*acceleration.xAccel
//                            + acceleration.yAccel*acceleration.yAccel
//                            + acceleration.zAccel*acceleration.zAccel));
//        }
//    });
//
//    priva
//
//
//        // At the beginning of each telemetry update, grab a bunch of data
//        // from the IMU that we will then display in separate lines.
//        telemetry.addAction(new Runnable() { @Override public void run()
//        {
//            // Acquiring the angles is relatively expensive; we don't want
//            // to do that in each of the three items that need that info, as that's
//            // three times the necessary expense.
//            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//            acceleration  = imu.getLinearAcceleration();
//        }
//        });
//
//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override public String value() {
//                        return acceleration.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(acceleration.xAccel*acceleration.xAccel
//                                        + acceleration.yAccel*acceleration.yAccel
//                                        + acceleration.zAccel*acceleration.zAccel));
//                    }
//                });
//    }



}

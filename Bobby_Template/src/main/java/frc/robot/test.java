package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;

public class test {
    private static CANSparkMax [] motors = {
        new CANSparkMax(13, MotorType.kBrushless), // L1, L2, R1, R2
        new CANSparkMax(14, MotorType.kBrushless),
        new CANSparkMax(7, MotorType.kBrushless),
        new CANSparkMax(12, MotorType.kBrushless),
    };

    private static Joystick joystick;

    public static void update() {
        double speed = joystick.getY() / 4;
        double rotation = joystick.getX() / 4;

        if (Math.abs(speed) > 0.1) 
            for (CANSparkMax motor : motors) {
                motor.set(speed);
            }
        else if (Math.abs(rotation) > 0.1) {
            motors[0].set(-rotation);
            motors[1].set(-rotation);
            motors[2].set(rotation);
            motors[3].set(rotation);
        } else {
            for (CANSparkMax motor : motors) {
                motor.set(0);
            }
        }
    }

    public static void init() {
        motors[0].setInverted(true);
        motors[1].setInverted(true);

        joystick = new Joystick(0);
    }
}
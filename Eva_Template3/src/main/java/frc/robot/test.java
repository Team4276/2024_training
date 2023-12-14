package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;

public class test {
    private static CANSparkMax motor;
    private static CANSparkMax motor2;
    private static Joystick joystick; 

    public static void update(){
        double speed = joystick.getY();
        double rotation = joystick.getX();

        if (Math.abs(speed) > 0.1) {
            motor.set(speed / 4);
            motor2.set(speed / 4);

        } else if(Math.abs(rotation) > 0.1) {
            motor.set(rotation / 4);
            motor2.set(rotation / 4);
        }

        else {
            motor.set(0);
            motor2.set(0);
        }
    }

    public static void init(){
        motor = new CANSparkMax(13, MotorType.kBrushless);
        motor2 = new CANSparkMax(14, MotorType.kBrushless);
        joystick = new Joystick(0);
    }
}

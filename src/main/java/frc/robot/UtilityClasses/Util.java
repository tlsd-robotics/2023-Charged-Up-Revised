package frc.robot.UtilityClasses;

public class Util {

    public static double map(double Value, double FromMin, double FromMax, double ToMin, double ToMax) {
        return (((Value - FromMin) / (FromMax - FromMin)) * (ToMax - ToMin)) + ToMin;
    }

    //Returns true if value is between or equal to min and max
    public static boolean inRange(double value, double min, double max) {
        return (value <= max) && (value >= min);
    }
}

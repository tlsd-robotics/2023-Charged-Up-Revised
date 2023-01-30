package frc.robot;

public class Util {

    public static double map(double Value, double FromMin, double FromMax, double ToMin, double ToMax) {
        return (((Value - FromMin) / (FromMax - FromMin)) * (ToMax - ToMin)) + ToMin;
    }

    public static double getHypotenuse(double valX, double valY) {
        return Math.sqrt(Math.pow(valX, 2) + Math.pow(valY, 2));
    }
    
}

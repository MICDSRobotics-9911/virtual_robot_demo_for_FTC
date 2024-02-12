package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class Derivatives {

    public static double getDerivative(double currentValue, double previousValue, double dt) {
        return (currentValue - previousValue) / dt;
    }

    public static double getDerivative(int currentValue, int previousValue, double dt) {
        return getDerivative(currentValue, (double)previousValue, dt);
    }

    public static List<Double> getDerivative(List<Double> currentValues, List<Double> previousValues, double dt) {
        List<Double> derivative = new ArrayList<>();
        for (int i = 0; i < currentValues.size(); i++) {
            derivative.add(
                    getDerivative(currentValues.get(i), previousValues.get(i), dt)
            );
        }
        return derivative;
    }

    public static List<Double> getDerivativeInteger(List<Integer> currentValues, List<Integer> previousValues, double dt) {
        return getDerivative(currentValues.stream().map(Integer::doubleValue).collect(Collectors.toList()),
                previousValues.stream().map(Integer::doubleValue).collect(Collectors.toList()), dt);
    }

}

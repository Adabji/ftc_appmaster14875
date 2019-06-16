package edu.spa.ftclib.internal.controller;

import edu.spa.ftclib.internal.sensor.Sensor;

/**
 * Created by Gabriel on 2018-01-02.
 */

public class FinishableIntegratedController extends IntegratedController {
    FinishingAlgorithm finisher;
    public FinishableIntegratedController(Sensor sensor, ControlAlgorithm algorithm, FinishingAlgorithm finisher) {
        super(sensor, algorithm);
        this.finisher = finisher;
    }

    @Override
    public void setTarget(double target) {
        finisher.setTarget(target); //We need to set the finisher's target before super.setTarget updates the finisher. This mistake took a while to find.
        super.setTarget(target);
    }

    @Override
    public double update() {
        double sensorValue = super.update();
        finisher.input(sensorValue);
        return sensorValue;
    }

    public boolean finished() {
        return finisher.finished();
    }
}

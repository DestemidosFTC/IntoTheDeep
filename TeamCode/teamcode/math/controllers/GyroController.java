package org.firstinspires.ftc.teamcode.math.controllers;

/**
 *  Controlador P dedicado a correção angular
 *  referências: <a href="https://www.ctrlaltftc.com/practical-examples/controlling-heading">Dealing with Angles</a>
 */
public class GyroController {

    private final double kP;

    public GyroController(double kP) {
        this.kP = kP;
    }

    /**
     * Normalizamos o ângulo recebido em um intervalo de -180° a +180°
     * para evitar erros numéricos na conversão e erros de direção em relação ao robô.
     * @param angleInRadians ângulo a ser convertido
     * @return ângulo normalizado em radianos
     */
    public double correctAngle(double angleInRadians) {
        while (angleInRadians > Math.PI) {
            angleInRadians -= 2 * Math.PI;
        }
        while (angleInRadians < -Math.PI) {
            angleInRadians += 2 * Math.PI;
        }

        return angleInRadians;
    }

    /**
     * Calcula o valor necessário para que o ângulo atual do robô
     * alcançe o ângulo desejado
     * @param targetAngle em radianos
     * @param currentAngle em radianos
     * @return valor de correção computado pelo controlador
     */
    public double calculate(double targetAngle, double currentAngle) {

        double error = correctAngle(targetAngle - currentAngle);

        // TODO: inserir o termo F
        return (kP * error); // + (kF * 0);
    }
}

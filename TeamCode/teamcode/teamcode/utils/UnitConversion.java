package org.firstinspires.ftc.teamcode.utils;

/** Classe utilitária para conversão de medidas comuns na FTC **/
public final class UnitConversion {
    private static final double kInchesToMeters = 0.0254;
    private static final double kInchesToCm = kInchesToMeters * 100;
    private static final double TAU = 2 * Math.PI;

    /**
     * Converte de centimetros para polegadas
     *
     * @param centimeters .
     * @return medida convertida em polegadas .
     */
    public static double cmToInches(double centimeters) {
        return centimeters / kInchesToCm;
    }

    /**
     * Converte de polegadas para centimetros
     *
     * @param inches .
     * @return medida convertida em centímetros .
     */
    public static double inchesToCm(double inches) {
        return inches * kInchesToCm;
    }

    /**
     * Converte de metros para polegadas
     *
     * @param meters .
     * @return medida convertida em polegadas .
     */
    public static double metersToInches(double meters) {
        return meters / kInchesToMeters;
    }

    /**
     * Converte de polegadas para metros
     *
     * @param inches .
     * @return medida convertida em metros .
     */
    public static double inchesToMeters(double inches) {
        return inches * kInchesToMeters;
    }

    /**
     * Converte de RPM (rotações por minuto) para Radianos por segundo
     *
     * @param rpm .
     * @return medida equivalente em radianos por segundo .
     */
    public static double rpmToRadiansPerSecond(double rpm) {
        return rpm * TAU / 60.0;
    }

    /**
     * Converte de Radianos por segundo para RPM (rotações por minuto)
     *
     * @param radiansPerSecond .
     * @return medida equivalente em RPM.
     */
    public static double radiansPerSecondToRPM(double radiansPerSecond) {
        return radiansPerSecond * 60.0 / TAU;
    }

    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Converte os ticks medidos por um encoder em metros
     *
     * @param ticks posição atual dada pelo encoder
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @param wheelRadius raio da roda em metros
     * @return medida percorrida pelo motor em metros
     */
    public static double encoderTicksToMeters(double ticks, double ticksPerRevoltuion, double wheelRadius) {
        return ticks * wheelRadius * TAU / ticksPerRevoltuion;
    }

    /**
     * Converte os ticks medidos por um encoder em centímetros
     *
     * @param ticks posição atual dada pelo encoder
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @param wheelRadius raio da roda em centímetros
     * @return medida percorrida pelo motor em centímetros
     */
    public static double encoderTicksToCm(double ticks, double ticksPerRevoltuion, double wheelRadius) {
        return ticks * wheelRadius * TAU / ticksPerRevoltuion;
    }

    /**
     * Converte os ticks medidos por um encoder em polegadas
     *
     * @param ticks posição atual dada pelo encoder
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @param wheelRadius raio da roda em polegadas
     * @return medida percorrida pelo motor em polegadas
     */
    public static double encoderTicksToInches(double ticks, double ticksPerRevoltuion, double wheelRadius) {
        return ticks * wheelRadius * TAU / ticksPerRevoltuion;
    }

    /**
     * Converte os ticks medidos por um encoder em graus
     *
     * @param ticks posição atual dada pelo encoder
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @return ângulo percorrido pelo motor em graus
     */
    public static double encoderTicksToDegrees(double ticks, double ticksPerRevoltuion) {
        return ticks * ticksPerRevoltuion / 360.0;
    }

    /**
     * Converte os ticks medidos por um encoder em radianos
     *
     * @param ticks posição atual dada pelo encoder
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @return ângulo percorrido pelo motor em radianos
     */
    public static double encoderTicksToRadians(double ticks, double ticksPerRevoltuion) {
        return ticks * ticksPerRevoltuion / TAU;
    }

    /**
     * Converte uma medida em metros, para o seu equivalente em ticks de encoder
     *
     * @param meters o valor desejado em radianos
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @return quantidade de ticks equivalente para percorrer a medida desejada
     */
    public static int metersToEncoderTicks(double meters, double ticksPerRevoltuion, double wheelRadius) {
        final double wheelCircumference = TAU * wheelRadius;
        return (int) ((meters / wheelCircumference) * ticksPerRevoltuion);
    }

    /**
     * Converte uma medida em centímetros, para o seu equivalente em ticks de encoder
     *
     * @param cm o valor desejado em radianos
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @return quantidade de ticks equivalente para percorrer a medida desejada
     */
    public static int cmToEncoderTicks(double cm, double ticksPerRevoltuion, double wheelRadius) {
        final double wheelCircumference = TAU * wheelRadius;
        return (int) ((cm / wheelCircumference) * ticksPerRevoltuion);
    }

    /**
     * Converte uma medida em polegdas, para o seu equivalente em ticks de encoder
     *
     * @param inches o valor desejado em radianos
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @return quantidade de ticks equivalente para percorrer a medida desejada
     */
    public static int inchesToEncoderTicks(double inches, double ticksPerRevoltuion, double wheelRadius) {
        final double wheelCircumference = TAU * wheelRadius;
        return (int) ((inches / wheelCircumference) * ticksPerRevoltuion);
    }

    /**
     * Converte uma medida em graus, para o seu equivalente em ticks de encoder
     *
     * @param degrees o valor desejado em radianos
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @return quantidade de ticks equivalente para percorrer a medida desejada
     */
    public static int degreesToEncoderTicks(double degrees, double ticksPerRevoltuion) {
        return (int) (ticksPerRevoltuion / 360.0 * degrees);
    }

    /**
     * Converte uma medida em radianos, para o seu equivalente em ticks de encoder
     *
     * @param radians o valor desejado em radianos
     * @param ticksPerRevoltuion a quantidade de tick que o encoder mede durante uma rotação, varia de motor pra motor
     * @return quantidade de ticks equivalente para percorrer a medida desejada
     */
    public static int radiansToEncoderTicks(double radians, double ticksPerRevoltuion) {
        return (int) (ticksPerRevoltuion / TAU * radians);
    }
}

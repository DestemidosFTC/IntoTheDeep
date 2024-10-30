package org.firstinspires.ftc.teamcode.utils;


/**
 * Classe utilitária que agrupa várias funções matemáticas
 * úteis para interpolação e controle de valores
 */
public final class MathUtils {

    /**
     * realiza uma interpolção linear
     * @param min
     * @param max
     * @param t
     * @return
     */
    public static double Lerp(double min, double max, double t) {
        return (1 - t) * min + t * max;
    }

    /**
     * Interpolação linear invertida, inserimos um valor V e recebemos
     * um valor T proporcional aos limites definidos
     * @param min valor mínimo de T
     * @param max valor máximo de T
     * @param v valor que estamos procurando a proporção
     */
    public static double InverseLerp(double min, double max, double v) {
        return (v - min) / (max - min);
    }

    /**
     * remapeia os valores de acordo com os limites definidos a baixo
     * @param iMin
     * @param iMax
     * @param oMin
     * @param oMax
     * @param value
     * @return
     */
    public static double ReMap(double iMin, double iMax, double oMin, double oMax, double value) {
        double t = InverseLerp(iMin, iMax, value);
        return Lerp(oMin, oMax, t);
    }

    /**
     * uma interpolação bem mais suave e simples
     * baseado em: https://thebookofshaders.com/glossary/?search=smoothstep
     * @param min
     * @param max
     * @param value
     * @return
     */
    public static double SmoothStep(double min, double max, double value) {
        double x = Clamp(InverseLerp(min, max, value), 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    /**
     * limita um valor dentro dos limites estabelecidos
     * @param value
     * @param min
     * @param max
     * @return
     */
    public static double Clamp(double value, double min, double max) {

        if (value > max) return max;

        return Math.max(value, min);

    }
}

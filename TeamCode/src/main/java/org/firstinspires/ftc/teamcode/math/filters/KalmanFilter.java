package org.firstinspires.ftc.teamcode.math.filters;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

/**
 * TODO: transformar esse filtro, na sua versão unidimensional
 * referências: https://www.ctrlaltftc.com/advanced/the-kalman-filter
 */
public class KalmanFilter {
    // kinematics description
    private SimpleMatrix F, Q, H;

    // sytem state estimate
    private SimpleMatrix x, P;

    public void configure(DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H ) {
        this.F = new SimpleMatrix(F);
        this.Q = new SimpleMatrix(Q);
        this.H = new SimpleMatrix(H);
    }

    public void setState( DMatrixRMaj x, DMatrixRMaj P ) {
        this.x = new SimpleMatrix(x);
        this.P = new SimpleMatrix(P);
    }

    public void predict() {
        // x = F x
        x = F.mult(x);

        // P = F P F' + Q
        P = F.mult(P).mult(F.transpose()).plus(Q);
    }

    public void update( DMatrixRMaj _z, DMatrixRMaj _R ) {
        // a fast way to make the matrices usable by SimpleMatrix
        SimpleMatrix z = SimpleMatrix.wrap(_z);
        SimpleMatrix R = SimpleMatrix.wrap(_R);

        // y = z - H x
        SimpleMatrix y = z.minus(H.mult(x));

        // S = H P H' + R
        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);

        // K = PH'S^(-1)
        SimpleMatrix K = P.mult(H.transpose().mult(S.invert()));

        // x = x + Ky
        x = x.plus(K.mult(y));

        // P = (I-kH)P = P - KHP
        P = P.minus(K.mult(H).mult(P));
    }

    public DMatrixRMaj getState() { return x.getMatrix(); }

    public DMatrixRMaj getCovariance() { return P.getMatrix(); }
}

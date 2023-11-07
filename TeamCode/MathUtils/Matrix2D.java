package org.firstinspires.ftc.teamcode.MathUtils;

public class Matrix2D {
    double[][] elements;
    /**
     * [1st axis size, 2nd axis size]
     */
    int[] shape = new int[2];
    /*
     * The number of elements in the matrix.
     */
    int size;

    /**
     * The determinant of the matrix.
     */

    public Matrix2D(double[][] elem) {
        elements = elem;
        shape[0] = elem.length;
        shape[1] = elem[0].length;
        size = shape[0]*shape[1];
    }

    public double get(int index1, int index2) {
        return elements[index1][index2];
    }

    private static Matrix2D vectorizedOp(Matrix2D m1, Matrix2D m2, MatricizedOperation op) {
        assert m1.shape[0] == m2.shape[0];
        assert m1.shape[1] == m2.shape[1];
        double[][] newmat = new double[m1.shape[0]][m1.shape[1]];
        for (int i = 0; i < m1.shape[0]; i++) {
            for (int j = 0; j < m1.shape[1]; j++) {
                newmat[i][j] = op.operation(m1.get(i, j), m2.get(i, j));
            }
        }
        return new Matrix2D(newmat);
    }

    /**
     * Vectorized add. Commutative.
     * @param m1
     * @param m2
     */
    public static Matrix2D add(Matrix2D m1, Matrix2D m2) {
        MatricizedOperation op = (a, b) -> {return a + b;};
        return vectorizedOp(m1, m2, op);
    }

    /*
     * Vectorized multiply. Commutative.
     */
    public static Matrix2D forEachMultiply(Matrix2D m1, Matrix2D m2) {
        MatricizedOperation op = (a, b) -> {return a * b;};
        return vectorizedOp(m1, m2, op);
    }
 
    // /*
    //  * Matrix multiplication (m1 * m2). Non-commutative.
    //  */
    // public static Matrix2D matMultiply(Matrix2D m1, Matrix2D m2) {
    //     assert m1.shape[0] == m2.shape[0];
    //     assert m1.shape[1] == m2.shape[1];
    //     assert m1.shape[0] >= m2.shape[1];
    //     int[][] newmat = new int[m2.shape[0]][m1.shape[1]];

    // }

    public static void main(String[] args) {
    }
}

interface MatricizedOperation {
    double operation(double a, double b);
}
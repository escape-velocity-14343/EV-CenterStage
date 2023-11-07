package org.firstinspires.ftc.teamcode.subsystems.MathUtils;

public class Vector {
    public double[] elements;
    public int size;

    /**
     * Argument (angle) of the vector. Assumes 2D.
     */
    public double arg;

    public Vector(double[] nums) {
        elements = nums;
        size = elements.length;
        arg = Math.atan2(elements[1], elements[0]);
    }

    public Vector(double x, double y) {
        elements = new double[]{x, y};
        size = elements.length;
        arg = Math.atan2(elements[1], elements[0]);
    }

    /**
     * Returns a 2D unit vector with the desired angle.
     * @param arg The angle of the unit vector.
     */
    public static Vector unitVector2D(double arg) {
        return new Vector(new double[]{Math.cos(arg), Math.sin(arg)});
    }

    /**
     * Returns the identity vector of addition for the specified dimension.
     * @return dim=2: [0, 0]
     */
    public static Vector zeroVector(int dim) {
        double[] zeroes = new double[dim];
        for (int i = 0; i < dim; i++) {
            zeroes[i] = 0;
        }
        return new Vector(zeroes);
    }

    public double get(int index) {
        return elements[index];
    }

    public Vector scale(double scalar) {
        double[] newelem = new double[size];
        for (int i = 0; i < size; i++) {
            newelem[i] = elements[i] * scalar;
        }
        return new Vector(newelem);
    }

    public double magnitude() {
        double sum = 0;
        for (double el : elements) {
            sum += Math.pow(el, 2);
        }
        return Math.sqrt(sum);
    }

    public static double dist(Vector v1, Vector v2) {
        assert v1.size == v2.size;
        return Vector.add(v1, v2.scale(-1)).magnitude();
    }

    public static double dot2D(Vector v1, Vector v2) {
        assert v1.size == v2.size && v1.size == 2;
        return v1.magnitude()*v2.magnitude()*Math.cos(Angle.normalize(Math.abs(v1.arg-v2.arg)));
    }

    /**
     * Applies an operation to every corresponding element in two vectors. Does not modify either vector.
     * @param op The operation. Declare with a SAM lambda from VectorizedOperation (double a, double b) -> {function (returns double)}.
     * @return The vector post-operation.
     */
    private static Vector vectorizedOp(Vector v1, Vector v2, VectorizedOperation op) {
        assert v1.size == v2.size;
        double[] newvec = new double[v1.size];
        for (int i = 0; i < v1.size; i++) {
                newvec[i] = op.operation(v1.get(i), v2.get(i));
        }
        return new Vector(newvec);
    }

    /**
     * Vectorized add. Commutative.
     */
    public static Vector add(Vector v1, Vector v2) {
        VectorizedOperation op = (a, b) -> {return a + b;};
        return vectorizedOp(v1, v2, op);
    }

    /**
     * Vectorized multiply. Commutative.
     */
    public static Vector vectorizedMultiply(Vector m1, Vector m2) {
        VectorizedOperation op = (a, b) -> {return a * b;};
        return vectorizedOp(m1, m2, op);
    }

    public static void main(String[] args) {
    }
}

interface VectorizedOperation {
    double operation(double a, double b);
}
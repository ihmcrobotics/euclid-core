package us.ihmc.geometry.matrix;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.exceptions.NotARotationMatrixException;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;

public abstract class Matrix3DFeatures
{
   public static final double EPS_CHECK_IDENTITY = 1.0e-7;
   public static final double EPS_CHECK_ROTATION = 1.0e-7;
   public static final double EPS_CHECK_2D = 1.0e-8;
   public static final double EPS_CHECK_ZERO_ROTATION = 1.0e-10;
   public static final double EPS_CHECK_SKEW = 1.0e-8;

   /**
    * Computes the determinant of the matrix described by the given 9 coefficients.
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @return the determinant of the matrix.
    */
   public static double determinant(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      double det;
      det = m00 * (m11 * m22 - m12 * m21);
      det += m01 * (m12 * m20 - m10 * m22);
      det += m02 * (m10 * m21 - m11 * m20);
      return det;
   }

   /**
    * Asserts that the given dense-matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    * 
    * @param matrix the matrix to verify. Not modified.
    * @throws NotARotationMatrixException if the matrix is not a rotation matrix.
    */
   public static void checkIfRotationMatrix(DenseMatrix64F matrix)
   {
      Matrix3DFeatures.checkMatrixSize(matrix);
      double m00 = matrix.get(0, 0);
      double m01 = matrix.get(0, 1);
      double m02 = matrix.get(0, 2);
      double m10 = matrix.get(1, 0);
      double m11 = matrix.get(1, 1);
      double m12 = matrix.get(1, 2);
      double m20 = matrix.get(2, 0);
      double m21 = matrix.get(2, 1);
      double m22 = matrix.get(2, 2);
      checkIfRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Asserts that the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    * 
    * @param matrixArray the matrix to verify, not null, not modified. The array is expected to be
    *           encoded in a row-major format.
    * @throws NotARotationMatrixException if the matrix is not a rotation matrix.
    */
   public static void checkIfRotationMatrix(double[] matrixArray)
   {
      checkIfRotationMatrix(matrixArray[0], matrixArray[1], matrixArray[2], matrixArray[3], matrixArray[4], matrixArray[5], matrixArray[6], matrixArray[7],
                            matrixArray[8]);
   }

   /**
    * Asserts that the given coefficients describe a rotation matrix.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * The given matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @throws NotARotationMatrixException if the matrix is not a rotation matrix.
    */
   public static void checkIfRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      boolean isRotationMatrix = Matrix3DFeatures.isRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_ROTATION);
      if (!isRotationMatrix)
         throw new NotARotationMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Asserts that the given matrix is a 3-by-3 matrix.
    * 
    * @param matrix the matrix to verify. Not modified.
    * @throws RuntimeException if the matrix is not a 3-by-3 matrix.
    */
   public static void checkMatrixSize(DenseMatrix64F matrix)
   {
      if (matrix.getNumRows() != 3 || matrix.getNumCols() != 3)
         throw new RuntimeException("Unexpected matrix size: " + matrix.getNumRows() + "-by-" + matrix.getNumCols() + ". Must be 3-by-3.");
   }

   /**
    * Tests if the matrix described by the given 9 coefficients is equal to the identity matrix.
    * <p>
    * The assertion is done on a per coefficient basis using {@link EPS_CHECK_IDENTITY} as the
    * tolerance.
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @return {@code true} if the matrix is considered to be equal to the identity matrix,
    *         {@code false} otherwise.
    */
   public static boolean isIdentity(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return isIdentity(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_IDENTITY);
   }

   /**
    * Tests if the matrix described by the given 9 coefficients is equal to the identity matrix.
    * <p>
    * The assertion is done on a per coefficient basis using {@code epsilon} as the tolerance.
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @param epsilon the tolerance as shown above.
    * @return {@code true} if the matrix is considered to be equal to the identity matrix,
    *         {@code false} otherwise.
    */
   public static boolean isIdentity(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22, double epsilon)
   {
      if (Math.abs(m00 - 1.0) > epsilon || Math.abs(m11 - 1.0) > epsilon || Math.abs(m22 - 1.0) > epsilon)
         return false;
      if (Math.abs(m01) > epsilon || Math.abs(m02) > epsilon || Math.abs(m12) > epsilon)
         return false;
      if (Math.abs(m10) > epsilon || Math.abs(m20) > epsilon || Math.abs(m21) > epsilon)
         return false;
      return true;
   }

   /**
    * Tests if the matrix described by the 9 coefficients represents a negligible rotation.
    * <p>
    * This matrix represents a 'zero' rotation if:
    * <ul>
    * <li>its trace is equal to {@code 3} +/- {@link EPS_CHECK_ZERO_ROTATION},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/-
    * {@link EPS_CHECK_ZERO_ROTATION}.
    * </ul>
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @return {@code true} if this matrix represents a 'zero' rotation, {@code false} otherwise.
    */
   public static boolean isZeroRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return isZeroRotation(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_ZERO_ROTATION);
   }

   /**
    * Tests if the matrix described by the 9 coefficients represents a negligible rotation.
    * <p>
    * This matrix represents a 'zero' rotation if:
    * <ul>
    * <li>its trace is equal to {@code 3} +/- {@code epsilon},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/-
    * {@code epsilon}.
    * </ul>
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @param epsilon the tolerance to use.
    * @return {@code true} if this matrix represents a 'zero' rotation, {@code false} otherwise.
    */
   public static boolean isZeroRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
                                        double epsilon)
   {
      return Math.abs(m00 + m11 + m22 - 3) < epsilon && Math.abs(m01 + m10) < epsilon && Math.abs(m02 + m20) < epsilon && Math.abs(m12 + m21) < epsilon;
   }

   /**
    * Tests if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    * 
    * @param matrix the matrix to verify, not null, not modified.
    * @return {@code true} if the given matrix is a rotation matrix, {@code false} otherwise.
    */
   public static boolean isRotationMatrix(DenseMatrix64F matrix)
   {
      return isRotationMatrix(matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 2), matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 2), matrix.get(2, 0),
                              matrix.get(2, 1), matrix.get(2, 2));
   }

   /**
    * Tests if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    * 
    * @param matrixArray the matrix to verify, not null, not modified. The array is expected to be
    *           encoded in a row-major format.
    * @return {@code true} if the given matrix is a rotation matrix, {@code false} otherwise.
    */
   public static boolean isRotationMatrix(double[] doubleArray)
   {
      return isRotationMatrix(doubleArray[0], doubleArray[1], doubleArray[2], doubleArray[3], doubleArray[4], doubleArray[5], doubleArray[6], doubleArray[7],
                              doubleArray[8]);
   }

   /**
    * Verify if the given coefficients describe a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @return {@code true} if the given matrix is a rotation matrix, {@code false} otherwise.
    */
   public static boolean isRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return isRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_ROTATION);
   }

   /**
    * Verify if the given coefficients describe a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @param epsilon the tolerance as shown above.
    * @return {@code true} if the given matrix is a rotation matrix, {@code false} otherwise.
    */
   public static boolean isRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
                                          double epsilon)
   {
      double xyDot = m00 * m10 + m01 * m11 + m02 * m12;
      if (Math.abs(xyDot) > epsilon)
         return false;

      double xzDot = m00 * m20 + m01 * m21 + m02 * m22;
      if (Math.abs(xzDot) > epsilon)
         return false;

      double yzDot = m10 * m20 + m11 * m21 + m12 * m22;
      if (Math.abs(yzDot) > epsilon)
         return false;

      double xNormSquared = m00 * m00 + m01 * m01 + m02 * m02;
      if (Math.abs(xNormSquared - 1.0) > epsilon)
         return false;

      double yNormSquared = m10 * m10 + m11 * m11 + m12 * m12;
      if (Math.abs(yNormSquared - 1.0) > epsilon)
         return false;

      double zNormSquared = m20 * m20 + m21 * m21 + m22 * m22;
      if (Math.abs(zNormSquared - 1.0) > epsilon)
         return false;

      double determinant = Matrix3DFeatures.determinant(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      if (Math.abs(determinant - 1.0) > epsilon)
         return false;
      return true;
   }

   /**
    * Verify the matrix described by the 9 given coefficients is a transformation in the XY plane.
    * <p>
    * The matrix is considered to be a 2D transformation in the XY plane if:
    * <ul>
    * <li>the last diagonal coefficient m22 is equal to 1.0 +/- {@code epsilon},
    * <li>the coefficients {@code m20}, {@code m02}, {@code m21}, and {@code m12} are equal to 0.0
    * +/- {@code epsilon}.
    * </ul>
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @param epsilon the tolerance used as shown above.
    * @return {@code true} if the given matrix describes a 2D transformation in the XY plane,
    *         {@code false} otherwise.
    */
   public static boolean isMatrix2D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22, double epsilon)
   {
      if (Math.abs(m20) > epsilon)
         return false;
      if (Math.abs(m02) > epsilon)
         return false;
      if (Math.abs(m21) > epsilon)
         return false;
      if (Math.abs(m12) > epsilon)
         return false;
      if (Math.abs(m22 - 1.0) > epsilon)
         return false;
      return true;
   }

   /**
    * Verify if the matrix described by the 9 given coefficients is skew symmetric:
    * 
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * The given matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@link EPS_CHECK_SKEW},
    * <li>the sum of each pair of cross diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/-
    * {@link EPS_CHECK_SKEW}.
    * </ul>
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @return {@code true} if the matrix is skew symmetric, {@code false} otherwise.
    */
   public static boolean isMatrixSkewSymmetric(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return isMatrixSkewSymmetric(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_SKEW);
   }

   /**
    * Verify if the matrix described by the 9 given coefficients is skew symmetric:
    * 
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * The given matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@link EPS_CHECK_SKEW},
    * <li>the sum of each pair of cross diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/-
    * {@link EPS_CHECK_SKEW}.
    * </ul>
    * </p>
    * 
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    * @param epsilon the tolerance used as shown above.
    * @return {@code true} if the matrix is skew symmetric, {@code false} otherwise.
    */
   public static boolean isMatrixSkewSymmetric(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
                                               double epsilon)
   {
      if (Math.abs(m00) > epsilon || Math.abs(m11) > epsilon || Math.abs(m22) > epsilon)
         return false;
      if (Math.abs(m01 + m10) > epsilon || Math.abs(m02 + m20) > epsilon || Math.abs(m12 + m21) > epsilon)
         return false;
      return true;
   }

   /**
    * Tests on a per component basis if the two given matrices are equal to an {@code epsilon}.
    * 
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal, {@code false} otherwise.
    */
   public static boolean epsilonEquals(Matrix3DReadOnly<?> m1, Matrix3DReadOnly<?> m2, double epsilon)
   {
      double diff;

      diff = m1.getM00() - m2.getM00();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m1.getM01() - m2.getM01();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m1.getM02() - m2.getM02();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m1.getM10() - m2.getM10();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m1.getM11() - m2.getM11();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m1.getM12() - m2.getM12();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m1.getM20() - m2.getM20();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m1.getM21() - m2.getM21();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m1.getM22() - m2.getM22();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      return true;
   }

   /**
    * Tests on a per coefficient basis if the two matrices {@code m1} and {@code m2} are
    * <b>exactly</b> equal.
    * <p>
    * If any of the two matrices is {@code null}, this methods returns {@code false}.
    * </p>
    * 
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @return {@code true} if the two matrices are <b>exactly</b> equal, {@code false} otherwise or
    *         if at least one of the two matrices is {@code null}.
    */
   public static boolean equals(Matrix3DReadOnly<?> m1, Matrix3DReadOnly<?> m2)
   {
      try
      {
         if (!m1.getClass().equals(m2.getClass()))
            return false;
         if (m1.getM00() != m2.getM00() || m1.getM01() != m2.getM01() || m1.getM02() != m2.getM02())
            return false;
         if (m1.getM10() != m2.getM10() || m1.getM11() != m2.getM11() || m1.getM12() != m2.getM12())
            return false;
         if (m1.getM20() != m2.getM20() || m1.getM21() != m2.getM21() || m1.getM22() != m2.getM22())
            return false;
         return true;
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }
}

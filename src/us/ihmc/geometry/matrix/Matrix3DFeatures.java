package us.ihmc.geometry.matrix;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.exceptions.NotAMatrix2DException;
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
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
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
    * Computes the determinant of the given matrix.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * 
    * @param matrix the matrix to compute the determinant of, not null, not modified.
    * @return the determinant of the matrix.
    */
   public static double determinant(Matrix3DReadOnly matrix)
   {
      return determinant(matrix.getM00(), matrix.getM01(), matrix.getM02(), matrix.getM10(), matrix.getM11(), matrix.getM12(), matrix.getM20(), matrix.getM21(),
            matrix.getM22());
   }

   /**
    * Verify if at least on the element of the given matrix is equal to {@linkplain Double#NaN}.
    * 
    * @param matrix the matrix to verify, not null, not modified.
    * @return true if at least one element of the matrix is equal to {@linkplain Double#NaN}, false otherwise.
    */
   public static boolean containsNaN(Matrix3DReadOnly matrix)
   {
      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();
      return containsNaN(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * <b> This method is meant for internal use only.</b>
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
    * @return true if at least one element of the matrix is equal to {@linkplain Double#NaN}, false otherwise.
    */
   public static boolean containsNaN(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      if (Double.isNaN(m00) || Double.isNaN(m01) || Double.isNaN(m02))
         return true;
      if (Double.isNaN(m10) || Double.isNaN(m11) || Double.isNaN(m12))
         return true;
      if (Double.isNaN(m20) || Double.isNaN(m21) || Double.isNaN(m22))
         return true;
      return false;
   }

   /**
    * Verify if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * <p>
    * @param matrix the matrix to verify, not null, not modified.
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
    * Verify if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * <p>
    * @param matrix the matrix to verify, not null, not modified.
    * @throws NotARotationMatrixException if the matrix is not a rotation matrix.
    */
   public static void checkIfRotationMatrix(Matrix3DReadOnly matrix)
   {
      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();
      checkIfRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Verify if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * <p>
    * @param matrixArray the matrix to verify, not null, not modified. The array is expected to be encoded in a row-major format.
    * @throws NotARotationMatrixException if the matrix is not a rotation matrix.
    */
   public static void checkIfRotationMatrix(double[] matrixArray)
   {
      checkIfRotationMatrix(matrixArray[0], matrixArray[1], matrixArray[2], matrixArray[3], matrixArray[4], matrixArray[5], matrixArray[6], matrixArray[7],
            matrixArray[8]);
   }

   /**
    * Verify if the given coefficients describe a rotation matrix.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * <p>
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
    * Verify the given matrix describes transformation in the XY plane.
    * 
    * <p>
    * The matrix is considered to be a 2D transformation in the XY plane if:
    * <li> the last diagonal coefficient m22 is equal to 1.0 +/- {@link EPS_CHECK_2D},
    * <li> the sum of the two pairs of coefficients (m20, m02) and (m21, m12) are equal to 0.0 +/- {@link EPS_CHECK_2D}.
    * @param matrix the matrix to verify, not null, not modified.
    * @throws NotAMatrix2DException if the matrix represents a 3D transformation.
    */
   public static void checkIfMatrix2D(Matrix3DReadOnly matrix)
   {
      boolean isMatrix2d = isMatrix2D(matrix);
      if (!isMatrix2d)
         throw new NotAMatrix2DException(matrix);
   }

   /**
    * Verify that the given matrix is a 3-by-3 matrix.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * @param matrix the matrix to verify, not null, not modified.
    * @throws RuntimeException if the matrix is not a 3-by-3 matrix.
    */
   public static void checkMatrixSize(DenseMatrix64F matrix)
   {
      if (matrix.getNumRows() != 3 || matrix.getNumCols() != 3)
         throw new RuntimeException("Unexpected matrix size: " + matrix.getNumRows() + "-by-" + matrix.getNumCols() + ". Must be 3-by-3.");
   }

   /**
    * Verify if the given matrix is equal to the identity matrix.
    * <p>
    * The assertion is done on a per coefficient basis using <code>epsilon</code> as the tolerance.
    * 
    * @param matrix the matrix to verify, not null, not modified.
    * @param epsilon the tolerance as shown above. 
    * @return true if the given matrix is considered to be equal to the identity matrix.
    */
   public static boolean isIdentity(Matrix3DReadOnly matrix, double epsilon)
   {
      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();
      return isIdentity(m00, m01, m02, m10, m11, m12, m20, m21, m22, epsilon);

   }

   /**
    * Verify if the matrix described by the given 9 coefficients is equal to the identity matrix.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * The assertion is done on a per coefficient basis using {@link EPS_CHECK_IDENTITY} as the tolerance.
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
    * @return true if the matrix is considered to be equal to the identity matrix.
    */
   public static boolean isIdentity(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return isIdentity(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_IDENTITY);
   }

   /**
    * Verify if the matrix described by the given 9 coefficients is equal to the identity matrix.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * The assertion is done on a per coefficient basis using <code>epsilon</code> as the tolerance.
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
    * @return true if the matrix is considered to be equal to the identity matrix.
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

   public static boolean isZeroRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return isZeroRotation(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_ZERO_ROTATION);
   }

   public static boolean isZeroRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
         double epsilon)
   {
      return Math.abs(m00 + m11 + m22 - 3) < epsilon && Math.abs(m01 + m10) < epsilon && Math.abs(m02 + m20) < epsilon && Math.abs(m12 + m21) < epsilon;
   }

   /**
    * Verify if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * <p>
    * @param matrix the matrix to verify, not null, not modified.
    * @return true if the given matrix is a rotation matrix, false otherwise.
    */
   public static boolean isRotationMatrix(DenseMatrix64F matrix)
   {
      return isRotationMatrix(matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 2), matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 2), matrix.get(2, 0),
            matrix.get(2, 1), matrix.get(2, 2));
   }

   /**
    * Verify if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * <p>
    * @param matrixArray the matrix to verify, not null, not modified. The array is expected to be encoded in a row-major format.
    * @return true if the given matrix is a rotation matrix, false otherwise.
    */
   public static boolean isRotationMatrix(double[] doubleArray)
   {
      return isRotationMatrix(doubleArray[0], doubleArray[1], doubleArray[2], doubleArray[3], doubleArray[4], doubleArray[5], doubleArray[6], doubleArray[7],
            doubleArray[8]);
   }

   /**
    * Verify if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * <p>
    * @param matrix the matrix to verify, not null, not modified.
    * @return true if the given matrix is a rotation matrix, false otherwise.
    */
   public static boolean isRotationMatrix(Matrix3DReadOnly matrix)
   {
      return isRotationMatrix(matrix, EPS_CHECK_ROTATION);
   }

   /**
    * Verify if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- <code>epsilon</code>,
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- <code>epsilon</code>,
    * <li> the determinant of the matrix is equal to 1.0 +/- <code>epsilon</code>.
    * <p>
    * @param matrix the matrix to verify, not null, not modified.
    * @param epsilon the tolerance as shown above. 
    * @return true if the given matrix is a rotation matrix, false otherwise.
    */
   public static boolean isRotationMatrix(Matrix3DReadOnly matrix, double epsilon)
   {
      return isRotationMatrix(matrix.getM00(), matrix.getM01(), matrix.getM02(), matrix.getM10(), matrix.getM11(), matrix.getM12(), matrix.getM20(),
            matrix.getM21(), matrix.getM22(), epsilon);
   }

   /**
    * Verify if the given coefficients describe a rotation matrix.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link EPS_CHECK_ROTATION}.
    * <p>
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
    * @return true if the given matrix is a rotation matrix, false otherwise.
    */
   public static boolean isRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return isRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_ROTATION);
   }

   /**
    * Verify if the given coefficients describe a rotation matrix.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- <code>epsilon</code>,
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- <code>epsilon</code>,
    * <li> the determinant of the matrix is equal to 1.0 +/- <code>epsilon</code>.
    * <p>
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
    * @return true if the given matrix is a rotation matrix, false otherwise.
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
    * Verify the given matrix describes transformation in the XY plane.
    * 
    * <p>
    * The matrix is considered to be a 2D transformation in the XY plane if:
    * <li> the last diagonal coefficient m22 is equal to 1.0 +/- {@link EPS_CHECK_2D},
    * <li> the coefficients <code>m20</code>, <code>m02</code>, <code>m21</code>, and <code>m12</code> are equal to 0.0 +/- <code>epsilon</code>.
    * 
    * @param matrix the matrix to verify, not null, not modified.
    * @return true if the given matrix describes a 2D transformation in the XY plane, false otherwise.
    */
   public static boolean isMatrix2D(Matrix3DReadOnly matrix)
   {
      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();
      return isMatrix2D(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_2D);
   }

   /**
    * Verify the matrix described by the 9 given coefficients is a transformation in the XY plane.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * 
    * The matrix is considered to be a 2D transformation in the XY plane if:
    * <li> the last diagonal coefficient m22 is equal to 1.0 +/- <code>epsilon</code>,
    * <li> the coefficients <code>m20</code>, <code>m02</code>, <code>m21</code>, and <code>m12</code> are equal to 0.0 +/- <code>epsilon</code>.
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
    * @return true if the given matrix describes a 2D transformation in the XY plane, false otherwise.
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
    * Verify if the given matrix is skew symmetric:
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * 
    * The given matrix is considered to be skew symmetric if:
    * <li> each diagonal coefficient is equal to 0.0 +/- <code>epsilon</code>,
    * <li> the sum of each pair of cross diagonal coefficients (m10, m01), (m12, m21), and (m20, m02) are equal to 0.0 +/- <code>epsilon</code>.
    * 
    * @param matrix to verify, not null, not modified.
    * @param epsilon the tolerance used as shown above.
    * @return true if the matrix is skew symmetric, false otherwise.
    */
   public static boolean isMatrixSkewSymmetric(Matrix3DReadOnly matrix, double epsilon)
   {
      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();
      return isMatrixSkewSymmetric(m00, m01, m02, m10, m11, m12, m20, m21, m22, epsilon);
   }

   /**
    * Verify if the matrix described by the 9 given coefficients is skew symmetric:
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * 
    * The given matrix is considered to be skew symmetric if:
    * <li> each diagonal coefficient is equal to 0.0 +/- {@link EPS_CHECK_SKEW},
    * <li> the sum of each pair of cross diagonal coefficients (m10, m01), (m12, m21), and (m20, m02) are equal to 0.0 +/- {@link EPS_CHECK_SKEW}.
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
    * @return true if the matrix is skew symmetric, false otherwise.
    */
   public static boolean isMatrixSkewSymmetric(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return isMatrixSkewSymmetric(m00, m01, m02, m10, m11, m12, m20, m21, m22, EPS_CHECK_SKEW);
   }

   /**
    * Verify if the matrix described by the 9 given coefficients is skew symmetric:
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * 
    * The given matrix is considered to be skew symmetric if:
    * <li> each diagonal coefficient is equal to 0.0 +/- <code>epsilon</code>,
    * <li> the sum of each pair of cross diagonal coefficients (m10, m01), (m12, m21), and (m20, m02) are equal to 0.0 +/- <code>epsilon</code>.
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
    * @return true if the matrix is skew symmetric, false otherwise.
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
    * Verify on a per coefficient basis if the two matrices <code>m1</code> and <code>m2</code> are equal.
    * @param m1 the first matrix, not null, not modified.
    * @param m2 the second matrix, not null, not modified.
    * @param epsilon the maximum difference between each coefficient of <code>m1</code> and <code>m2</code> for which both numbers are still
     * considered equal.
    * @return true if the two matrices are considered equal, false otherwise.
    */
   public static boolean epsilonEquals(Matrix3DReadOnly m1, Matrix3DReadOnly m2, double epsilon)
   {
      double diff;

      diff = m2.getM00() - m1.getM00();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m2.getM01() - m1.getM01();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m2.getM02() - m1.getM02();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m2.getM10() - m1.getM10();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m2.getM11() - m1.getM11();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m2.getM12() - m1.getM12();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m2.getM20() - m1.getM20();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m2.getM21() - m1.getM21();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = m2.getM22() - m1.getM22();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      return true;
   }

   /**
    * Verify on a per coefficient basis if the two matrices <code>m1</code> and <code>m2</code> are <b>exactly</b> equal.
    * @param m1 the first matrix, null returns false, not modified.
    * @param m2 the second matrix, null returns false, not modified.
    * @return true if the two matrices are <b>exactly</b> equal, false otherwise or if at least one of the two matrices is null.
    */
   public static boolean equals(Matrix3DReadOnly m1, Matrix3DReadOnly m2)
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

package us.ihmc.geometry.matrix.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.exceptions.NotAMatrix2DException;
import us.ihmc.geometry.exceptions.NotARotationMatrixException;
import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.matrix.Matrix3DFeatures;
import us.ihmc.geometry.matrix.Matrix3DReadOnlyTools;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;

/**
 * Read-only interface for any type of 3D matrices.
 * 
 * @author Sylvain
 *
 */
public interface Matrix3DReadOnly<T extends Matrix3DReadOnly<T>> extends EpsilonComparable<T>
{
   /**
    * Gets the 1st row 1st column coefficient of this matrix.
    * 
    * @return the 1st row 1st column coefficient.
    */
   double getM00();

   /**
    * Gets the 1st row 2nd column coefficient of this matrix.
    * 
    * @return the 1st row 2nd column coefficient.
    */
   double getM01();

   /**
    * Gets the 1st row 3rd column coefficient of this matrix.
    * 
    * @return the 1st row 3rd column coefficient.
    */
   double getM02();

   /**
    * Gets the 2nd row 1st column coefficient of this matrix.
    * 
    * @return the 2nd row 1st column coefficient.
    */
   double getM10();

   /**
    * Gets the 2nd row 2nd column coefficient of this matrix.
    * 
    * @return the 2nd row 2nd column coefficient.
    */
   double getM11();

   /**
    * Gets the 2nd row 3rd column coefficient of this matrix.
    * 
    * @return the 2nd row 3rd column coefficient.
    */
   double getM12();

   /**
    * Gets the 3rd row 1st column coefficient of this matrix.
    * 
    * @return the 3rd row 1st column coefficient.
    */
   double getM20();

   /**
    * Gets the 3rd row 2nd column coefficient of this matrix.
    * 
    * @return the 3rd row 2nd column coefficient.
    */
   double getM21();

   /**
    * Gets the 3rd row 3rd column coefficient of this matrix.
    * 
    * @return the 3rd row 3rd column coefficient.
    */
   double getM22();

   /**
    * Retrieves and returns a coefficient of this matrix given its
    * row and column indices.
    * 
    * @param row the row of the coefficient to return.
    * @param column the column of the coefficient to return.
    * @return the coefficient's value.
    * @throws ArrayIndexOutOfBoundsException if either {@code row} &notin; [0, 2] or {@code column} &notin; [0, 2].
    */
   default double getElement(int row, int column)
   {
      switch (row)
      {
      case 0:
         switch (column)
         {
         case 0:
            return getM00();
         case 1:
            return getM01();
         case 2:
            return getM02();
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }
      case 1:
         switch (column)
         {
         case 0:
            return getM10();
         case 1:
            return getM11();
         case 2:
            return getM12();
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }

      case 2:
         switch (column)
         {
         case 0:
            return getM20();
         case 1:
            return getM21();
         case 2:
            return getM22();
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }

      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Packs the coefficients of this matrix into a row-major 1D array.
    * 
    * @param matrixArrayToPack the array in which the coefficients of this matrix are stored. Modified.
    */
   default void get(double[] matrixArrayToPack)
   {
      matrixArrayToPack[0] = getM00();
      matrixArrayToPack[1] = getM01();
      matrixArrayToPack[2] = getM02();
      matrixArrayToPack[3] = getM10();
      matrixArrayToPack[4] = getM11();
      matrixArrayToPack[5] = getM12();
      matrixArrayToPack[6] = getM20();
      matrixArrayToPack[7] = getM21();
      matrixArrayToPack[8] = getM22();
   }

   /**
    * Packs the coefficients of this matrix into a row-major 1D array starting at the given index {@code startIndex}.
    * 
    * @param matrixArrayToPack the array in which the coefficients of this matrix are stored. Modified.
    * @param startIndex index in the array to store the first coefficient of this matrix.
    */
   default void get(double[] matrixArrayToPack, int startIndex)
   {
      matrixArrayToPack[startIndex++] = getM00();
      matrixArrayToPack[startIndex++] = getM01();
      matrixArrayToPack[startIndex++] = getM02();
      matrixArrayToPack[startIndex++] = getM10();
      matrixArrayToPack[startIndex++] = getM11();
      matrixArrayToPack[startIndex++] = getM12();
      matrixArrayToPack[startIndex++] = getM20();
      matrixArrayToPack[startIndex++] = getM21();
      matrixArrayToPack[startIndex] = getM22();
   }

   /**
    * Packs the coefficients of this matrix into 2D matrix.
    * <p>
    * Note: the given matrix has to be large enough to store this matrix.
    * </p>
    * 
    * @param matrixToPack the 2D matrix in which the coefficients of this are stored. Modified.
    */
   default void get(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(0, 0, getM00());
      matrixToPack.set(0, 1, getM01());
      matrixToPack.set(0, 2, getM02());
      matrixToPack.set(1, 0, getM10());
      matrixToPack.set(1, 1, getM11());
      matrixToPack.set(1, 2, getM12());
      matrixToPack.set(2, 0, getM20());
      matrixToPack.set(2, 1, getM21());
      matrixToPack.set(2, 2, getM22());
   }

   default void get(DenseMatrix64F matrixToPack, int startRow, int startColumn)
   {
      int row = startRow;
      int column = startColumn;
      matrixToPack.set(row, column++, getM00());
      matrixToPack.set(row, column++, getM01());
      matrixToPack.set(row++, column, getM02());
      column = startColumn;
      matrixToPack.set(row, column++, getM10());
      matrixToPack.set(row, column++, getM11());
      matrixToPack.set(row++, column, getM12());
      column = startColumn;
      matrixToPack.set(row, column++, getM20());
      matrixToPack.set(row, column++, getM21());
      matrixToPack.set(row, column, getM22());
   }

   default void getColumn(int column, double columnArrayToPack[])
   {
      switch (column)
      {
      case 0:
         columnArrayToPack[0] = getM00();
         columnArrayToPack[1] = getM10();
         columnArrayToPack[2] = getM20();
         return;
      case 1:
         columnArrayToPack[0] = getM01();
         columnArrayToPack[1] = getM11();
         columnArrayToPack[2] = getM21();
         return;
      case 2:
         columnArrayToPack[0] = getM02();
         columnArrayToPack[1] = getM12();
         columnArrayToPack[2] = getM22();
         return;
      default:
         throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
      }
   }

   default void getColumn(int column, TupleBasics columnToPack)
   {
      switch (column)
      {
      case 0:
         columnToPack.setX(getM00());
         columnToPack.setY(getM10());
         columnToPack.setZ(getM20());
         return;
      case 1:
         columnToPack.setX(getM01());
         columnToPack.setY(getM11());
         columnToPack.setZ(getM21());
         return;
      case 2:
         columnToPack.setX(getM02());
         columnToPack.setY(getM12());
         columnToPack.setZ(getM22());
         return;
      default:
         throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
      }
   }

   default void getRow(int row, double rowArrayToPack[])
   {
      switch (row)
      {
      case 0:
         rowArrayToPack[0] = getM00();
         rowArrayToPack[1] = getM01();
         rowArrayToPack[2] = getM02();
         return;
      case 1:
         rowArrayToPack[0] = getM10();
         rowArrayToPack[1] = getM11();
         rowArrayToPack[2] = getM12();
         return;
      case 2:
         rowArrayToPack[0] = getM20();
         rowArrayToPack[1] = getM21();
         rowArrayToPack[2] = getM22();
         return;
      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
   }

   default void getRow(int row, TupleBasics rowVectorToPack)
   {
      switch (row)
      {
      case 0:
         rowVectorToPack.setX(getM00());
         rowVectorToPack.setY(getM01());
         rowVectorToPack.setZ(getM02());
         return;
      case 1:
         rowVectorToPack.setX(getM10());
         rowVectorToPack.setY(getM11());
         rowVectorToPack.setZ(getM12());
         return;
      case 2:
         rowVectorToPack.setX(getM20());
         rowVectorToPack.setY(getM21());
         rowVectorToPack.setZ(getM22());
         return;
      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Verify if at least on the element of this matrix is equal to {@linkplain Double#NaN}.
    * 
    * @return true if at least one element of the matrix is equal to {@linkplain Double#NaN}, false otherwise.
    */
   default boolean containsNaN()
   {
      return Matrix3DFeatures.containsNaN(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   /**
    * Computes the determinant of this matrix.
    * 
    * @return the determinant of this matrix.
    */
   default double determinant()
   {
      return Matrix3DFeatures.determinant(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   /**
    * Verify if this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link Matrix3DFeatures#EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link Matrix3DFeatures#EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link Matrix3DFeatures#EPS_CHECK_ROTATION}.
    * <p>
    * @throws NotARotationMatrixException if the matrix is not a rotation matrix.
    */
   default void checkIfRotationMatrix()
   {
      Matrix3DFeatures.checkIfRotationMatrix(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   /**
    * Verify if this matrix describes transformation in the XY plane.
    * 
    * <p>
    * This matrix is considered to be a 2D transformation in the XY plane if:
    * <li> the last diagonal coefficient m22 is equal to 1.0 +/- {@link Matrix3DFeatures#EPS_CHECK_2D},
    * <li> the sum of the two pairs of coefficients (m20, m02) and (m21, m12) are equal to 0.0 +/- {@link Matrix3DFeatures#EPS_CHECK_2D}.
    * @param matrix the matrix to verify, not null, not modified.
    * @throws NotAMatrix2DException if the matrix represents a 3D transformation.
    */
   default void checkIfMatrix2D()
   {
      if (!isMatrix2D())
         throw new NotAMatrix2DException(this);
   }

   /**
    * Verify if the given matrix is equal to the identity matrix.
    * <p>
    * The assertion is done on a per coefficient basis using <code>epsilon</code> as the tolerance.
    * 
    * @param matrix the matrix to verify, not null, not modified.
    * @param epsilon the tolerance as shown above. 
    * @return {@code true} if the given matrix is considered to be equal to the identity matrix.
    */
   default boolean isIdentity(double epsilon)
   {
      return Matrix3DFeatures.isIdentity(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   default boolean isZeroRotation()
   {
      return Matrix3DFeatures.isZeroRotation(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   default boolean isZeroRotation(double epsilon)
   {
      return Matrix3DFeatures.isZeroRotation(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   /**
    * Verify if the given matrix is a rotation matrix.
    * <p>
    * The given matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- {@link Matrix3DFeatures#EPS_CHECK_ROTATION},
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@link Matrix3DFeatures#EPS_CHECK_ROTATION},
    * <li> the determinant of the matrix is equal to 1.0 +/- {@link Matrix3DFeatures#EPS_CHECK_ROTATION}.
    * <p>
    * @param matrix the matrix to verify, not null, not modified.
    * @return {@code true} if the given matrix is a rotation matrix, {@code false} otherwise.
    */
   default boolean isRotationMatrix()
   {
      return isRotationMatrix(Matrix3DFeatures.EPS_CHECK_ROTATION);
   }

   /**
    * Verify if this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <li> the length of each row vector is equal to 1.0 +/- <code>epsilon</code>,
    * <li> the dot product of each pair of row vectors is equal to 0.0 +/- <code>epsilon</code>,
    * <li> the determinant of the matrix is equal to 1.0 +/- <code>epsilon</code>.
    * <p>
    * @param matrix the matrix to verify, not null, not modified.
    * @param epsilon the tolerance as shown above. 
    * @return {@code true} if the given matrix is a rotation matrix, {@code false} otherwise.
    */
   default boolean isRotationMatrix(double epsilon)
   {
      return Matrix3DFeatures.isRotationMatrix(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   /**
    * Verify this matrix describes transformation in the XY plane.
    * 
    * <p>
    * A matrix is considered to be a 2D transformation in the XY plane if:
    * <li> the last diagonal coefficient m22 is equal to 1.0 +/- {@value Matrix3DFeatures#EPS_CHECK_2D},
    * <li> the coefficients <code>m20</code>, <code>m02</code>, <code>m21</code>, and <code>m12</code> are equal to 0.0 +/- {@value Matrix3DFeatures#EPS_CHECK_2D}.
    * 
    * @param matrix the matrix to verify, not null, not modified.
    * @return {@code true} if the given matrix describes a 2D transformation in the XY plane, {@code false} otherwise.
    */
   default boolean isMatrix2D()
   {
      return Matrix3DFeatures.isMatrix2D(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(),
            Matrix3DFeatures.EPS_CHECK_2D);
   }

   /**
    * Verify if this matrix is skew symmetric:
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * 
    * This matrix is considered to be skew symmetric if:
    * <li> each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    * <li> the sum of each pair of cross diagonal coefficients (m10, m01), (m12, m21), and (m20, m02) are equal to 0.0 +/- {@code epsilon}.
    * 
    * @param matrix to verify, not null, not modified.
    * @param epsilon the tolerance used as shown above.
    * @return true if the matrix is skew symmetric, false otherwise.
    */
   default boolean isMatrixSkewSymmetric(double epsilon)
   {
      return Matrix3DFeatures.isMatrixSkewSymmetric(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   default boolean epsilonEquals(Matrix3DReadOnly<?> other, double epsilon)
   {
      double diff;

      diff = other.getM00() - getM00();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = other.getM01() - getM01();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = other.getM02() - getM02();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = other.getM10() - getM10();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = other.getM11() - getM11();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = other.getM12() - getM12();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = other.getM20() - getM20();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = other.getM21() - getM21();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = other.getM22() - getM22();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      return true;
   }
}
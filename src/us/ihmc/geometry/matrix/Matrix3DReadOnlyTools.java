package us.ihmc.geometry.matrix;

import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;

public abstract class Matrix3DReadOnlyTools
{
   /**
    * Provides a {@code String} representation of the given matrix as follows: <br>
    * m00, m01, m02 <br>
    * m10, m11, m12 <br>
    * m20, m21, m22
    * 
    * @param matrix the matrix to get a {@code String} representation of. Not modified.
    * @return the {@code String} representing the given matrix.
    */
   public static String toString(Matrix3DReadOnly<?> matrix)
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
      return Matrix3DReadOnlyTools.toString(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Provides a {@code String} representation of the given matrix as follows: <br>
    * m00, m01, m02 <br>
    * m10, m11, m12 <br>
    * m20, m21, m22
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
    * @return the {@code String} representing the given matrix.
    */
   public static String toString(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return m00 + ", " + m01 + ", " + m02 + "\n" + m10 + ", " + m11 + ", " + m12 + "\n" + m20 + ", " + m21 + ", " + m22 + "\n";
   }

   /**
    * Create an {@linkplain ArrayIndexOutOfBoundsException} for a bad column index.
    * 
    * @param column the bad column index.
    * @return the exception
    */
   public static ArrayIndexOutOfBoundsException columnOutOfBoundsException(int maxColumnIndex, int column)
   {
      return new ArrayIndexOutOfBoundsException("column should be in [0, " + maxColumnIndex + "], but is: " + column);
   }

   /**
    * Create an {@linkplain ArrayIndexOutOfBoundsException} for a bad row index.
    * 
    * @param row the bad row index.
    * @return the exception
    */
   public static ArrayIndexOutOfBoundsException rowOutOfBoundsException(int maxRowIndex, int row)
   {
      return new ArrayIndexOutOfBoundsException("row should be in [0, " + maxRowIndex + "], but is: " + row);
   }
}

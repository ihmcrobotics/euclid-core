package us.ihmc.geometry.matrix;

import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;

public abstract class Matrix3DReadOnlyTools
{
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

   public static String toString(double[] matrixArray)
   {
      return Matrix3DReadOnlyTools.toString(matrixArray[0], matrixArray[1], matrixArray[2], matrixArray[3], matrixArray[4], matrixArray[5], matrixArray[6], matrixArray[7], matrixArray[8]);
   }

   public static String toString(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return m00 + ", " + m01 + ", " + m02 + "\n" + m10 + ", " + m11 + ", " + m12 + "\n" + m20 + ", " + m21 + ", " + m22 + "\n";
   }

   /**
    * Crate an {@linkplain ArrayIndexOutOfBoundsException} for a bad column index.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * 
    * @param column the bad column index.
    * @return the exception
    */
   public static ArrayIndexOutOfBoundsException columnOutOfBoundsException(int maxColumnIndex, int column)
   {
      return new ArrayIndexOutOfBoundsException("column should be in [0, " + maxColumnIndex + "], but is: " + column);
   }

   /**
    * Crate an {@linkplain ArrayIndexOutOfBoundsException} for a bad row index.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * 
    * @param row the bad row index.
    * @return the exception
    */
   public static ArrayIndexOutOfBoundsException rowOutOfBoundsException(int maxRowIndex, int row)
   {
      return new ArrayIndexOutOfBoundsException("row should be in [0, " + maxRowIndex + "], but is: " + row);
   }
}

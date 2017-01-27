package us.ihmc.geometry.matrix.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.interfaces.Settable;

public interface Matrix3DBasics<T extends Matrix3DBasics<T>> extends Matrix3DReadOnly<T>, Settable<T>
{
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);

   /**
    * Sets this matrix to contain only {@linkplain Double#NaN}:
    * <pre>
    *     | NaN  NaN  NaN |
    * m = | NaN  NaN  NaN |
    *     | NaN  NaN  NaN |
    */
   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   default boolean containsNaN()
   {
      return Matrix3DReadOnly.super.containsNaN();
   }

   /**
    * Sets this matrix to identity:
    * <pre>
    *     | 1  0  0 |
    * m = | 0  1  0 |
    *     | 0  0  1 |
    */
   default void setIdentity()
   {
      set(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
   }

   default void transpose()
   {
      set(getM00(), getM10(), getM20(), getM01(), getM11(), getM21(), getM02(), getM12(), getM22());
   }

   default void set(Matrix3DReadOnly<?> other)
   {
      set(other.getM00(), other.getM01(), other.getM02(), other.getM10(), other.getM11(), other.getM12(), other.getM20(), other.getM21(), other.getM22());
   }

   default void set(double[] matrixArray)
   {
      double m00 = matrixArray[0];
      double m01 = matrixArray[1];
      double m02 = matrixArray[2];
      double m10 = matrixArray[3];
      double m11 = matrixArray[4];
      double m12 = matrixArray[5];
      double m20 = matrixArray[6];
      double m21 = matrixArray[7];
      double m22 = matrixArray[8];
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   default void set(DenseMatrix64F matrix)
   {
      double m00 = matrix.get(0, 0);
      double m01 = matrix.get(0, 1);
      double m02 = matrix.get(0, 2);
      double m10 = matrix.get(1, 0);
      double m11 = matrix.get(1, 1);
      double m12 = matrix.get(1, 2);
      double m20 = matrix.get(2, 0);
      double m21 = matrix.get(2, 1);
      double m22 = matrix.get(2, 2);
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   default void set(DenseMatrix64F matrix, int startRow, int startColumn)
   {
      int row = startRow;
      int column = startColumn;
   
      double m00 = matrix.get(row, column++);
      double m01 = matrix.get(row, column++);
      double m02 = matrix.get(row, column);
      row++;
      column = startColumn;
      double m10 = matrix.get(row, column++);
      double m11 = matrix.get(row, column++);
      double m12 = matrix.get(row, column);
      row++;
      column = startColumn;
      double m20 = matrix.get(row, column++);
      double m21 = matrix.get(row, column++);
      double m22 = matrix.get(row, column);
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }
}
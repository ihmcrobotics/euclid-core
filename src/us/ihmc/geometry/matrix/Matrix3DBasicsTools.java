package us.ihmc.geometry.matrix;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;

public abstract class Matrix3DBasicsTools
{
   public static void setMatrixFromArray(double[] matrixArray, Matrix3DBasics matrixToPack)
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
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void setMatrixFromOther(Matrix3DReadOnly other, Matrix3DBasics matrixToPack)
   {
      double m00 = other.getM00();
      double m01 = other.getM01();
      double m02 = other.getM02();
      double m10 = other.getM10();
      double m11 = other.getM11();
      double m12 = other.getM12();
      double m20 = other.getM20();
      double m21 = other.getM21();
      double m22 = other.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void setMatrixFromDenseMatrix(DenseMatrix64F matrix, Matrix3DBasics matrixToPack)
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
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void setMatrixFromDenseMatrix(DenseMatrix64F matrix, int startRow, int startColumn, Matrix3DBasics matrixToPack)
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
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }
}

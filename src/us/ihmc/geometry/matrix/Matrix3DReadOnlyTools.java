package us.ihmc.geometry.matrix;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;

public abstract class Matrix3DReadOnlyTools
{
   public static void getMatrixAsArray(Matrix3DReadOnly matrixToReadFrom, double[] matrixArrayToPack)
   {
      matrixArrayToPack[0] = matrixToReadFrom.getM00();
      matrixArrayToPack[1] = matrixToReadFrom.getM01();
      matrixArrayToPack[2] = matrixToReadFrom.getM02();
      matrixArrayToPack[3] = matrixToReadFrom.getM10();
      matrixArrayToPack[4] = matrixToReadFrom.getM11();
      matrixArrayToPack[5] = matrixToReadFrom.getM12();
      matrixArrayToPack[6] = matrixToReadFrom.getM20();
      matrixArrayToPack[7] = matrixToReadFrom.getM21();
      matrixArrayToPack[8] = matrixToReadFrom.getM22();
   }

   public static void getMatrixAsArray(Matrix3DReadOnly matrixToReadFrom, double[] matrixArrayToPack, int startIndex)
   {
      matrixArrayToPack[startIndex++] = matrixToReadFrom.getM00();
      matrixArrayToPack[startIndex++] = matrixToReadFrom.getM01();
      matrixArrayToPack[startIndex++] = matrixToReadFrom.getM02();
      matrixArrayToPack[startIndex++] = matrixToReadFrom.getM10();
      matrixArrayToPack[startIndex++] = matrixToReadFrom.getM11();
      matrixArrayToPack[startIndex++] = matrixToReadFrom.getM12();
      matrixArrayToPack[startIndex++] = matrixToReadFrom.getM20();
      matrixArrayToPack[startIndex++] = matrixToReadFrom.getM21();
      matrixArrayToPack[startIndex]   = matrixToReadFrom.getM22();
   }

   public static void getMatrixAsDenseMatrix(Matrix3DReadOnly matrixToReadFrom, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(0, 0, matrixToReadFrom.getM00());
      matrixToPack.set(0, 1, matrixToReadFrom.getM01());
      matrixToPack.set(0, 2, matrixToReadFrom.getM02());
      matrixToPack.set(1, 0, matrixToReadFrom.getM10());
      matrixToPack.set(1, 1, matrixToReadFrom.getM11());
      matrixToPack.set(1, 2, matrixToReadFrom.getM12());
      matrixToPack.set(2, 0, matrixToReadFrom.getM20());
      matrixToPack.set(2, 1, matrixToReadFrom.getM21());
      matrixToPack.set(2, 2, matrixToReadFrom.getM22());
   }

   public static void getMatrixAsDenseMatrix(Matrix3DReadOnly matrixToReadFrom, DenseMatrix64F matrixToPack, int startRow, int startColumn)
   {
      int row = startRow;
      int column = startColumn;
      matrixToPack.set(row, column++, matrixToReadFrom.getM00());
      matrixToPack.set(row, column++, matrixToReadFrom.getM01());
      matrixToPack.set(row++, column, matrixToReadFrom.getM02());
      column = startColumn;
      matrixToPack.set(row, column++, matrixToReadFrom.getM10());
      matrixToPack.set(row, column++, matrixToReadFrom.getM11());
      matrixToPack.set(row++, column, matrixToReadFrom.getM12());
      column = startColumn;
      matrixToPack.set(row, column++, matrixToReadFrom.getM20());
      matrixToPack.set(row, column++, matrixToReadFrom.getM21());
      matrixToPack.set(row, column  , matrixToReadFrom.getM22());
   }

   public static double getMatrixElement(Matrix3DReadOnly matrixToReadFrom, int row, int column)
   {
      switch (row)
      {
      case 0:
         switch (column)
         {
         case 0:
            return matrixToReadFrom.getM00();
         case 1:
            return matrixToReadFrom.getM01();
         case 2:
            return matrixToReadFrom.getM02();
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }
      case 1:
         switch (column)
         {
         case 0:
            return matrixToReadFrom.getM10();
         case 1:
            return matrixToReadFrom.getM11();
         case 2:
            return matrixToReadFrom.getM12();
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }
   
      case 2:
         switch (column)
         {
         case 0:
            return matrixToReadFrom.getM20();
         case 1:
            return matrixToReadFrom.getM21();
         case 2:
            return matrixToReadFrom.getM22();
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }
   
      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
   }

   public static void getMatrixColumn(Matrix3DReadOnly matrixToReadFrom, int column, double columnArrayToPack[])
   {
      switch (column)
      {
      case 0:
         columnArrayToPack[0] = matrixToReadFrom.getM00();
         columnArrayToPack[1] = matrixToReadFrom.getM10();
         columnArrayToPack[2] = matrixToReadFrom.getM20();
         return;
      case 1:
         columnArrayToPack[0] = matrixToReadFrom.getM01();
         columnArrayToPack[1] = matrixToReadFrom.getM11();
         columnArrayToPack[2] = matrixToReadFrom.getM21();
         return;
      case 2:
         columnArrayToPack[0] = matrixToReadFrom.getM02();
         columnArrayToPack[1] = matrixToReadFrom.getM12();
         columnArrayToPack[2] = matrixToReadFrom.getM22();
         return;
      default:
         throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
      }
   }

   public static void getMatrixColumn(Matrix3DReadOnly matrixToReadFrom, int column, TupleBasics columnToPack)
   {
      switch (column)
      {
      case 0:
         columnToPack.setX(matrixToReadFrom.getM00());
         columnToPack.setY(matrixToReadFrom.getM10());
         columnToPack.setZ(matrixToReadFrom.getM20());
         return;
      case 1:
         columnToPack.setX(matrixToReadFrom.getM01());
         columnToPack.setY(matrixToReadFrom.getM11());
         columnToPack.setZ(matrixToReadFrom.getM21());
         return;
      case 2:
         columnToPack.setX(matrixToReadFrom.getM02());
         columnToPack.setY(matrixToReadFrom.getM12());
         columnToPack.setZ(matrixToReadFrom.getM22());
         return;
      default:
         throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
      }
   }

   public static void getMatrixRow(Matrix3DReadOnly matrixToReadFrom, int row, double rowArrayToPack[])
   {
      switch (row)
      {
      case 0:
         rowArrayToPack[0] = matrixToReadFrom.getM00();
         rowArrayToPack[1] = matrixToReadFrom.getM01();
         rowArrayToPack[2] = matrixToReadFrom.getM02();
         return;
      case 1:
         rowArrayToPack[0] = matrixToReadFrom.getM10();
         rowArrayToPack[1] = matrixToReadFrom.getM11();
         rowArrayToPack[2] = matrixToReadFrom.getM12();
         return;
      case 2:
         rowArrayToPack[0] = matrixToReadFrom.getM20();
         rowArrayToPack[1] = matrixToReadFrom.getM21();
         rowArrayToPack[2] = matrixToReadFrom.getM22();
         return;
      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
   }

   public static void getMatrixRow(Matrix3DReadOnly matrixToReadFrom, int row, TupleBasics rowVectorToPack)
   {
      switch (row)
      {
      case 0:
         rowVectorToPack.setX(matrixToReadFrom.getM00());
         rowVectorToPack.setY(matrixToReadFrom.getM01());
         rowVectorToPack.setZ(matrixToReadFrom.getM02());
         return;
      case 1:
         rowVectorToPack.setX(matrixToReadFrom.getM10());
         rowVectorToPack.setY(matrixToReadFrom.getM11());
         rowVectorToPack.setZ(matrixToReadFrom.getM12());
         return;
      case 2:
         rowVectorToPack.setX(matrixToReadFrom.getM20());
         rowVectorToPack.setY(matrixToReadFrom.getM21());
         rowVectorToPack.setZ(matrixToReadFrom.getM22());
         return;
      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
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

   public static String toString(Matrix3DReadOnly matrix)
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
}

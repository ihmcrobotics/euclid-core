package us.ihmc.geometry.matrix;

import java.io.Serializable;

import us.ihmc.geometry.exceptions.SingularMatrixException;
import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public class Matrix3D implements Serializable, Matrix3DBasics<Matrix3D>, GeometryObject<Matrix3D>
{
   private static final long serialVersionUID = -1016899240187632674L;
   /** The 9 coefficients of this matrix. */
   private double m00, m01, m02, m10, m11, m12, m20, m21, m22;

   public Matrix3D()
   {
   }

   public Matrix3D(double[] matrixArray)
   {
      set(matrixArray);
   }

   public Matrix3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public Matrix3D(Matrix3DReadOnly<?> other)
   {
      set(other);
   }

   @Override
   public void setToZero()
   {
      set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      this.m00 = m00;
      this.m01 = m01;
      this.m02 = m02;
   
      this.m10 = m10;
      this.m11 = m11;
      this.m12 = m12;
   
      this.m20 = m20;
      this.m21 = m21;
      this.m22 = m22;
   }

   @Override
   public void set(Matrix3D other)
   {
      set((Matrix3DReadOnly<?>) other); 
   }

   /**
    * Converts a vector to tilde form (matrix implementation of cross product):
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    */
   public void setToTildeForm(TupleReadOnly tuple)
   {
      m00 = 0.0;
      m01 = -tuple.getZ();
      m02 = tuple.getY();
          
      m10 = tuple.getZ();
      m11 = 0.0;
      m12 = -tuple.getX();
          
      m20 = -tuple.getY();
      m21 = tuple.getX();
      m22 = 0.0;
   }

   public void add(Matrix3DReadOnly<?> other)
   {
      m00 += other.getM00();
      m01 += other.getM01();
      m02 += other.getM02();

      m10 += other.getM10();
      m11 += other.getM11();
      m12 += other.getM12();

      m20 += other.getM20();
      m21 += other.getM21();
      m22 += other.getM22();
   }

   public void add(Matrix3DReadOnly<?> matrix1, Matrix3DReadOnly<?> matrix2)
   {
      m00 = matrix1.getM00() + matrix2.getM00();
      m01 = matrix1.getM01() + matrix2.getM01();
      m02 = matrix1.getM02() + matrix2.getM02();

      m10 = matrix1.getM10() + matrix2.getM10();
      m11 = matrix1.getM11() + matrix2.getM11();
      m12 = matrix1.getM12() + matrix2.getM12();

      m20 = matrix1.getM20() + matrix2.getM20();
      m21 = matrix1.getM21() + matrix2.getM21();
      m22 = matrix1.getM22() + matrix2.getM22();
   }

   public void sub(Matrix3DReadOnly<?> other)
   {
      m00 -= other.getM00();
      m01 -= other.getM01();
      m02 -= other.getM02();

      m10 -= other.getM10();
      m11 -= other.getM11();
      m12 -= other.getM12();

      m20 -= other.getM20();
      m21 -= other.getM21();
      m22 -= other.getM22();
   }

   public void sub(Matrix3DReadOnly<?> matrix1, Matrix3DReadOnly<?> matrix2)
   {
      m00 = matrix1.getM00() - matrix2.getM00();
      m01 = matrix1.getM01() - matrix2.getM01();
      m02 = matrix1.getM02() - matrix2.getM02();

      m10 = matrix1.getM10() - matrix2.getM10();
      m11 = matrix1.getM11() - matrix2.getM11();
      m12 = matrix1.getM12() - matrix2.getM12();

      m20 = matrix1.getM20() - matrix2.getM20();
      m21 = matrix1.getM21() - matrix2.getM21();
      m22 = matrix1.getM22() - matrix2.getM22();
   }

   public void fill(double scalar)
   {
      m00 = scalar;
      m01 = scalar;
      m02 = scalar;

      m10 = scalar;
      m11 = scalar;
      m12 = scalar;
      
      m20 = scalar;
      m21 = scalar;
      m22 = scalar;
   }

   public void scale(double scalar)
   {
      m00 *= scalar;
      m01 *= scalar;
      m02 *= scalar;

      m10 *= scalar;
      m11 *= scalar;
      m12 *= scalar;
      
      m20 *= scalar;
      m21 *= scalar;
      m22 *= scalar;
   }

   public void scaleRows(double scalarRow0, double scalarRow1, double scalarRow2)
   {
      m00 *= scalarRow0;
      m01 *= scalarRow0;
      m02 *= scalarRow0;

      m10 *= scalarRow1;
      m11 *= scalarRow1;
      m12 *= scalarRow1;
      
      m20 *= scalarRow2;
      m21 *= scalarRow2;
      m22 *= scalarRow2;
   }

   public void scaleColumns(double scalarColumn0, double scalarColumn1, double scalarColumn2)
   {
      m00 *= scalarColumn0;
      m01 *= scalarColumn1;
      m02 *= scalarColumn2;

      m10 *= scalarColumn0;
      m11 *= scalarColumn1;
      m12 *= scalarColumn2;
      
      m20 *= scalarColumn0;
      m21 *= scalarColumn1;
      m22 *= scalarColumn2;
   }

   public void multiplyOuter()
   {
      Matrix3DTools.multiplyTransposeRight(this, this, this);
   }

   public void multiplyOuter(Matrix3DReadOnly<?> other)
   {
      set(other);
      multiplyOuter();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * Invert this matrix such that: m = m<sup>-1</sup>
    * @throws SingularMatrixException when the matrix is not invertible.
    */
   public void invert()
   {
      boolean success = Matrix3DTools.invert(this);
      if (!success)
         throw new SingularMatrixException(this);
   }

   /**
    * Set this matrix to the inverse of the other matrix.
    * 
    * @param matrix the other matrix. Not modified.
    * @throws SingularMatrixException when the matrix is not invertible.
    */
   public void setAndInvert(Matrix3DReadOnly<?> matrix)
   {
      set(matrix);
      invert();
   }

   public void normalize()
   {
      Matrix3DTools.normalize(this);
   }

   public void setAndNormalize(Matrix3DReadOnly<?> matrix)
   {
      set(matrix);
      normalize();
   }

   public void setAndTranspose(Matrix3DReadOnly<?> matrix)
   {
      set(matrix);
      transpose();
   }

   public void multiply(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiply(this, other, this);
   }

   public void multiplyTransposeThis(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyTransposeLeft(this, other, this);
   }

   public void multiplyTransposeOther(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyTransposeRight(this, other, this);
   }

   public void multiplyTransposeBoth(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyTransposeBoth(this, other, this);
   }

   public void multiplyInvertThis(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyInvertLeft(this, other, this);
   }

   public void multiplyInvertOther(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   public void multiplyInvertOther(RotationMatrixReadOnly<?> other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   public void multiplyInvertOther(RotationScaleMatrixReadOnly<?> other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   public void preMultiply(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiply(other, this, this);
   }

   public void preMultiplyTransposeThis(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyTransposeRight(other, this, this);
   }

   public void preMultiplyTransposeOther(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyTransposeLeft(other, this, this);
   }

   public void preMultiplyTransposeBoth(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyTransposeBoth(other, this, this);
   }

   public void preMultiplyInvertThis(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyInvertRight(other, this, this);
   }

   public void preMultiplyInvertOther(Matrix3DReadOnly<?> other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }
   
   public void preMultiplyInvertOther(RotationMatrixReadOnly<?> other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }
   
   public void preMultiplyInvertOther(RotationScaleMatrixReadOnly<?> other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }

   public void transform(TupleBasics tupleToTransform)
   {
      Matrix3DTools.transform(this, tupleToTransform, tupleToTransform);
   }

   public void transform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      Matrix3DTools.transform(this, tupleOriginal, tupleTransformed);
   }

   public void transform(Tuple2DBasics tupleToTransform)
   {
      Matrix3DTools.transform(this, tupleToTransform, tupleToTransform, true);
   }

   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      Matrix3DTools.transform(this, tupleOriginal, tupleTransformed, true);
   }

   public void transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      Matrix3DTools.transform(this, tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      Matrix3DTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   public void transform(Matrix3D matrixToTransform)
   {
      Matrix3DTools.transform(this, matrixToTransform, matrixToTransform);
   }

   public void transform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      Matrix3DTools.transform(this, matrixOriginal, matrixTransformed);
   }

   public void inverseTransform(TupleBasics tupleToTransform)
   {
      Matrix3DTools.inverseTransform(this, tupleToTransform, tupleToTransform);
   }

   public void inverseTransform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   public void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      Matrix3DTools.inverseTransform(this, tupleToTransform, tupleToTransform, true);
   }

   public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed, true);
   }

   public void setRow(int row, double rowArray[])
   {
      setRow(row, rowArray[0], rowArray[1], rowArray[2]);
   }

   public void setRow(int row, double x, double y, double z)
   {
      switch (row)
      {
      case 0:
         m00 = x;
         m01 = y;
         m02 = z;
         return;
   
      case 1:
         m10 = x;
         m11 = y;
         m12 = z;
         return;
   
      case 2:
         m20 = x;
         m21 = y;
         m22 = z;
         return;
   
      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
   }

   public void setRow(int row, TupleReadOnly rowValues)
   {
      setRow(row, rowValues.getX(), rowValues.getY(), rowValues.getZ());
   }

   public void setColumn(int column, double columnArray[])
   {
      setColumn(column, columnArray[0], columnArray[1], columnArray[2]);
   }

   public void setColumn(int column, double x, double y, double z)
   {
      switch (column)
      {
      case 0:
         m00 = x;
         m10 = y;
         m20 = z;
         break;
   
      case 1:
         m01 = x;
         m11 = y;
         m21 = z;
         break;
   
      case 2:
         m02 = x;
         m12 = y;
         m22 = z;
         break;
   
      default:
         throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
      }
   }

   public void setColumn(int column, TupleReadOnly columnValues)
   {
      setColumn(column, columnValues.getX(), columnValues.getY(), columnValues.getZ());
   }

   public void scaleRow(int row, double scalar)
   {
      switch (row)
      {
      case 0:
         m00 *= scalar;
         m01 *= scalar;
         m02 *= scalar;
         return;
   
      case 1:
         m10 *= scalar;
         m11 *= scalar;
         m12 *= scalar;
         return;
   
      case 2:
         m20 *= scalar;
         m21 *= scalar;
         m22 *= scalar;
         return;
   
      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
   }

   public void scaleColumn(int column, double scalar)
   {
      switch (column)
      {
      case 0:
         m00 *= scalar;
         m10 *= scalar;
         m20 *= scalar;
         break;
   
      case 1:
         m01 *= scalar;
         m11 *= scalar;
         m21 *= scalar;
         break;
   
      case 2:
         m02 *= scalar;
         m12 *= scalar;
         m22 *= scalar;
         break;
   
      default:
         throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
      }
   }

   public void setElement(int row, int column, double value)
   {
      switch (row)
      {
      case 0:
         switch (column)
         {
         case 0:
            setM00(value);
            return;
         case 1:
            setM01(value);
            return;
         case 2:
            setM02(value);
            return;
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }
   
      case 1:
         switch (column)
         {
         case 0:
            setM10(value);
            return;
         case 1:
            setM11(value);
            return;
         case 2:
            setM12(value);
            return;
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }
   
      case 2:
         switch (column)
         {
         case 0:
            setM20(value);
            return;
         case 1:
            setM21(value);
            return;
         case 2:
            setM22(value);
            return;
         default:
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(2, column);
         }
   
      default:
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(2, row);
      }
   }

   public void setM00(double m00)
   {
      this.m00 = m00;
   }

   public void setM01(double m01)
   {
      this.m01 = m01;
   }

   public void setM02(double m02)
   {
      this.m02 = m02;
   }

   public void setM10(double m10)
   {
      this.m10 = m10;
   }

   public void setM11(double m11)
   {
      this.m11 = m11;
   }

   public void setM12(double m12)
   {
      this.m12 = m12;
   }

   public void setM20(double m20)
   {
      this.m20 = m20;
   }

   public void setM21(double m21)
   {
      this.m21 = m21;
   }

   public void setM22(double m22)
   {
      this.m22 = m22;
   }

   public void scaleM00(double scalar)
   {
      this.m00 *= scalar;
   }

   public void scaleM01(double scalar)
   {
      this.m01 *= scalar;
   }

   public void scaleM02(double scalar)
   {
      this.m02 *= scalar;
   }

   public void scaleM10(double scalar)
   {
      this.m10 *= scalar;
   }

   public void scaleM11(double scalar)
   {
      this.m11 *= scalar;
   }

   public void scaleM12(double scalar)
   {
      this.m12 *= scalar;
   }

   public void scaleM20(double scalar)
   {
      this.m20 *= scalar;
   }

   public void scaleM21(double scalar)
   {
      this.m21 *= scalar;
   }

   public void scaleM22(double scalar)
   {
      this.m22 *= scalar;
   }

   @Override
   public double getM00()
   {
      return m00;
   }

   @Override
   public double getM01()
   {
      return m01;
   }

   @Override
   public double getM02()
   {
      return m02;
   }

   @Override
   public double getM10()
   {
      return m10;
   }

   @Override
   public double getM11()
   {
      return m11;
   }

   @Override
   public double getM12()
   {
      return m12;
   }

   @Override
   public double getM20()
   {
      return m20;
   }

   @Override
   public double getM21()
   {
      return m21;
   }

   @Override
   public double getM22()
   {
      return m22;
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Matrix3D) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Matrix3D other)
   {
      return Matrix3DFeatures.equals(this, other);
   }

   @Override
   public String toString()
   {
      return Matrix3DReadOnlyTools.toString(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(m00);
      bits = 31L * bits + Double.doubleToLongBits(m01);
      bits = 31L * bits + Double.doubleToLongBits(m02);
      bits = 31L * bits + Double.doubleToLongBits(m10);
      bits = 31L * bits + Double.doubleToLongBits(m11);
      bits = 31L * bits + Double.doubleToLongBits(m12);
      bits = 31L * bits + Double.doubleToLongBits(m20);
      bits = 31L * bits + Double.doubleToLongBits(m21);
      bits = 31L * bits + Double.doubleToLongBits(m22);
      return (int) (bits ^ bits >> 32);
   }

   /**
    * Verify on a per coefficient basis if this matrix is equal to {@code other}.
    * @param other the second matrix. Not modified.
    * @param epsilon tolerance to use when comparing each coefficient.
    * @return {@code true} if the two matrices are considered equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Matrix3D other, double epsilon)
   {
      return epsilonEquals(other, epsilon);
   }
}

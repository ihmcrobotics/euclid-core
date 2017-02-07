package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public abstract class Tuple2D<T extends Tuple2D<T>> implements Serializable, Tuple2DBasics<T>
{
   private final static long serialVersionUID = -1825643561716102256L;

   private double x, y;

   public Tuple2D()
   {
      setToZero();
   }

   public Tuple2D(double x, double y)
   {
      set(x, y);
   }

   public Tuple2D(double[] tupleArray)
   {
      set(tupleArray);
   }

   public Tuple2D(Tuple2DReadOnly<?> other)
   {
      set(other);
   }

   @Override
   public void setToNaN()
   {
      x = Double.NaN;
      y = Double.NaN;
   }

   @Override
   public void setToZero()
   {
      x = 0.0;
      y = 0.0;
   }

   public void absolute()
   {
      x = Math.abs(x);
      y = Math.abs(y);
   }

   public void negate()
   {
      x = -x;
      y = -y;
   }

   public void clipToMax(double max)
   {
      x = Math.min(max, x);
      y = Math.min(max, y);
   }

   public void clipToMin(double min)
   {
      x = Math.max(min, x);
      y = Math.max(min, y);
   }

   public void clipToMinMax(double min, double max)
   {
      clipToMax(max);
      clipToMin(min);
   }

   public boolean containsNaN()
   {
      return Double.isNaN(x) || Double.isNaN(y);
   }

   public void setAndAbsolute(Tuple2DReadOnly<?> other)
   {
      set(other);
      absolute();
   }

   public void setAndNegate(Tuple2DReadOnly<?> other)
   {
      set(other);
      negate();
   }

   public void setAndClipToMax(double max, Tuple2DReadOnly<?> other)
   {
      set(other);
      clipToMax(max);
   }

   public void setAndClipToMin(double min, Tuple2DReadOnly<?> other)
   {
      set(other);
      clipToMin(min);
   }

   public void setAndClipToMinMax(double min, double max, Tuple2DReadOnly<?> other)
   {
      set(other);
      clipToMinMax(min, max);
   }

   @Override
   public void set(double x, double y)
   {
      this.x = x;
      this.y = y;
   }

   public void set(double[] tupleArray)
   {
      set(tupleArray, 0);
   }

   public void set(double[] tupleArray, int startIndex)
   {
      x = tupleArray[startIndex++];
      y = tupleArray[startIndex];
   }

   public void set(DenseMatrix64F matrix)
   {
      set(matrix, 0);
   }

   public void set(DenseMatrix64F matrix, int startRow)
   {
      set(matrix, startRow, 0);
   }

   public void set(DenseMatrix64F matrix, int startRow, int column)
   {
      x = matrix.get(startRow++, column);
      y = matrix.get(startRow, column);
   }

   @Override
   public void set(int index, double value)
   {
      switch (index)
      {
      case 0:
         x = value;
         break;
      case 1:
         y = value;
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   @Override
   public void set(Tuple2DReadOnly<?> other)
   {
      x = other.getX();
      y = other.getY();
   }

   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   @Override
   public void add(double x, double y)
   {
      this.x += x;
      this.y += y;
   }

   @Override
   public void add(Tuple2DReadOnly<?> other)
   {
      x += other.getX();
      y += other.getY();
   }

   public void add(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      set(tuple1);
      add(tuple2);
   }

   @Override
   public void sub(double x, double y)
   {
      this.x -= x;
      this.y -= y;
   }

   @Override
   public void sub(Tuple2DReadOnly<?> other)
   {
      x -= other.getX();
      y -= other.getY();
   }

   public void sub(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      set(tuple1);
      sub(tuple2);
   }

   public void scale(double scalar)
   {
      scale(scalar, scalar);
   }

   public void scale(double scalarX, double scalarY)
   {
      x *= scalarX;
      y *= scalarY;
   }

   public void scale(double scalar, Tuple2DReadOnly<?> other)
   {
      set(other);
      scale(scalar);
   }

   public void scaleAdd(double scalar, Tuple2DReadOnly<?> other)
   {
      scale(scalar);
      add(other);
   }

   public void scaleAdd(double scalar, Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      scale(scalar, tuple1);
      add(tuple2);
   }

   public void interpolate(Tuple2DReadOnly<?> other, double alpha)
   {
      x = Tuple3DTools.interpolate(x, other.getX(), alpha);
      y = Tuple3DTools.interpolate(y, other.getY(), alpha);
   }

   public void interpolate(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2, double alpha)
   {
      set(tuple1);
      interpolate(tuple2, alpha);
   }

   public void get(double[] tupleArrayToPack)
   {
      get(tupleArrayToPack, 0);
   }

   public void get(double[] tupleArrayToPack, int startIndex)
   {
      tupleArrayToPack[startIndex++] = x;
      tupleArrayToPack[startIndex] = y;
   }

   public void get(DenseMatrix64F tupleMatrixToPack)
   {
      get(tupleMatrixToPack, 0, 0);
   }

   public void get(DenseMatrix64F tupleMatrixToPack, int startRow)
   {
      get(tupleMatrixToPack, startRow, 0);
   }

   public void get(DenseMatrix64F tupleMatrixToPack, int startRow, int column)
   {
      tupleMatrixToPack.set(startRow++, column, x);
      tupleMatrixToPack.set(startRow, column, y);
   }

   @Override
   public double get(int index)
   {
      switch (index)
      {
      case 0:
         return x;
      case 1:
         return y;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   public void get(Tuple2DBasics<?> other)
   {
      other.setX(x);
      other.setY(y);
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   public boolean epsilonEquals(Tuple2DReadOnly<?> other, double epsilon)
   {
      return Tuple3DTools.epsilonEquals(this, other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Tuple2DReadOnly<?>) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Tuple2DReadOnly<?> other)
   {
      try
      {
         return x == other.getX() && y == other.getY();
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "(" + x + ", " + y + ")";
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(x);
      bits = 31L * bits + Double.doubleToLongBits(y);
      return (int) (bits ^ bits >> 32);
   }
}

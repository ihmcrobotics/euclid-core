package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public abstract class Tuple2D32<T extends Tuple2D32<T>> implements Serializable, Tuple2DBasics<T>
{
   private static final long serialVersionUID = 8630707913840584158L;

   private float x, y;

   public Tuple2D32()
   {
      setToZero();
   }

   public Tuple2D32(float x, float y)
   {
      set(x, y);
   }

   public Tuple2D32(float[] tupleArray)
   {
      set(tupleArray);
   }

   public Tuple2D32(Tuple2DReadOnly<?> other)
   {
      set(other);
   }

   @Override
   public void setToNaN()
   {
      x = Float.NaN;
      y = Float.NaN;
   }

   @Override
   public void setToZero()
   {
      x = 0.0f;
      y = 0.0f;
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

   public void clipToMax(float max)
   {
      x = Math.min(max, x);
      y = Math.min(max, y);
   }

   public void clipToMin(float min)
   {
      x = Math.max(min, x);
      y = Math.max(min, y);
   }

   public void clipToMinMax(float min, float max)
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

   public void setAndClipToMax(float max, Tuple2DReadOnly<?> other)
   {
      set(other);
      clipToMax(max);
   }

   public void setAndClipToMin(float min, Tuple2DReadOnly<?> other)
   {
      set(other);
      clipToMin(min);
   }

   public void setAndClipToMinMax(float min, float max, Tuple2DReadOnly<?> other)
   {
      set(other);
      clipToMinMax(min, max);
   }

   @Override
   public void set(double x, double y)
   {
      this.x = (float) x;
      this.y = (float) y;
   }

   public void set(float[] tupleArray)
   {
      set(tupleArray, 0);
   }

   public void set(float[] tupleArray, int startIndex)
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
      x = (float) matrix.get(startRow++, column);
      y = (float) matrix.get(startRow, column);
   }

   @Override
   public void set(int index, double value)
   {
      switch (index)
      {
      case 0:
         x = (float) value;
         break;
      case 1:
         y = (float) value;
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   @Override
   public void set(Tuple2DReadOnly<?> other)
   {
      x = (float) other.getX();
      y = (float) other.getY();
   }

   @Override
   public void setX(double x)
   {
      this.x = (float) x;
   }

   @Override
   public void setY(double y)
   {
      this.y = (float) y;
   }

   public void setX(float x)
   {
      this.x = x;
   }

   public void setY(float y)
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

   public void scale(float scalar)
   {
      scale(scalar, scalar);
   }

   public void scale(float scalarX, float scalarY)
   {
      x *= scalarX;
      y *= scalarY;
   }

   public void scale(float scalar, Tuple2DReadOnly<?> other)
   {
      set(other);
      scale(scalar);
   }

   public void scaleAdd(float scalar, Tuple2DReadOnly<?> other)
   {
      scale(scalar);
      add(other);
   }

   public void scaleAdd(float scalar, Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      scale(scalar, tuple1);
      add(tuple2);
   }

   public void interpolate(Tuple2DReadOnly<?> other, float alpha)
   {
      x = (float) Tuple3DTools.interpolate(x, other.getX(), alpha);
      y = (float) Tuple3DTools.interpolate(y, other.getY(), alpha);
   }

   public void interpolate(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2, float alpha)
   {
      set(tuple1);
      interpolate(tuple2, alpha);
   }

   public void get(float[] tupleArrayToPack)
   {
      get(tupleArrayToPack, 0);
   }

   public void get(float[] tupleArrayToPack, int startIndex)
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

   public float getX32()
   {
      return x;
   }

   public float getY32()
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
      bits = 31L * bits + Float.floatToIntBits(x);
      bits = 31L * bits + Float.floatToIntBits(y);
      return (int) (bits ^ bits >> 32);
   }
}

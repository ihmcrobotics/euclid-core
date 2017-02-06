package us.ihmc.geometry.tuple;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.tuple.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple.interfaces.Tuple3DReadOnly;

public abstract class Tuple32 implements Serializable, Tuple3DBasics
{
   private static final long serialVersionUID = 7705737376600145100L;

   private float x, y, z;

   public Tuple32()
   {
      setToZero();
   }

   public Tuple32(float x, float y, float z)
   {
      set(x, y, z);
   }

   public Tuple32(float[] tupleArray)
   {
      set(tupleArray);
   }

   public Tuple32(Tuple3DReadOnly other)
   {
      set(other);
   }

   @Override
   public void setToNaN()
   {
      x = Float.NaN;
      y = Float.NaN;
      z = Float.NaN;
   }

   @Override
   public void setToZero()
   {
      x = 0.0f;
      y = 0.0f;
      z = 0.0f;
   }

   public void absolute()
   {
      x = Math.abs(x);
      y = Math.abs(y);
      z = Math.abs(z);
   }

   public void negate()
   {
      x = -x;
      y = -y;
      z = -z;
   }

   public void clipToMax(float max)
   {
      x = Math.min(max, x);
      y = Math.min(max, y);
      z = Math.min(max, z);
   }

   public void clipToMin(float min)
   {
      x = Math.max(min, x);
      y = Math.max(min, y);
      z = Math.max(min, z);
   }

   public void clipToMinMax(float min, float max)
   {
      clipToMax(max);
      clipToMin(min);
   }

   public boolean containsNaN()
   {
      return TupleTools.containsNaN(this);
   }

   public void setAndAbsolute(Tuple3DReadOnly other)
   {
      set(other);
      absolute();
   }

   public void setAndNegate(Tuple3DReadOnly other)
   {
      set(other);
      negate();
   }

   public void setAndClipToMax(float max, Tuple3DReadOnly other)
   {
      set(other);
      clipToMax(max);
   }

   public void setAndClipToMin(float min, Tuple3DReadOnly other)
   {
      set(other);
      clipToMin(min);
   }

   public void setAndClipToMinMax(float min, float max, Tuple3DReadOnly other)
   {
      set(other);
      clipToMinMax(min, max);
   }

   @Override
   public void set(double x, double y, double z)
   {
      this.x = (float) x;
      this.y = (float) y;
      this.z = (float) z;
   }

   public void set(float x, float y, float z)
   {
      this.x = x;
      this.y = y;
      this.z = z;
   }

   public void set(float[] tupleArray)
   {
      set(tupleArray, 0);
   }

   public void set(float[] tupleArray, int startIndex)
   {
      x = tupleArray[startIndex++];
      y = tupleArray[startIndex++];
      z = tupleArray[startIndex];
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
      y = (float) matrix.get(startRow++, column);
      z = (float) matrix.get(startRow, column);
   }

   @Override
   public void set(int index, double value)
   {
      set(index, (float) value);
   }

   public void set(int index, float value)
   {
      switch (index)
      {
      case 0:
         x = value;
         break;
      case 1:
         y = value;
         break;
      case 2:
         z = value;
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   @Override
   public void set(Tuple3DReadOnly other)
   {
      x = (float) other.getX();
      y = (float) other.getY();
      z = (float) other.getZ();
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

   @Override
   public void setZ(double z)
   {
      this.z = (float) z;
   }

   public void setX(float x)
   {
      this.x = x;
   }

   public void setY(float y)
   {
      this.y = y;
   }

   public void setZ(float z)
   {
      this.z = z;
   }

   @Override
   public void add(double x, double y, double z)
   {
      add((float) x, (float) y, (float) z);
   }

   public void add(float x, float y, float z)
   {
      this.x += x;
      this.y += y;
      this.z += z;
   }

   @Override
   public void add(Tuple3DReadOnly other)
   {
      x += other.getX();
      y += other.getY();
      z += other.getZ();
   }

   public void add(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      set(tuple1);
      add(tuple2);
   }

   @Override
   public void sub(double x, double y, double z)
   {
      sub((float) x, (float) y, (float) z);
   }

   public void sub(float x, float y, float z)
   {
      this.x -= x;
      this.y -= y;
      this.z -= z;
   }

   @Override
   public void sub(Tuple3DReadOnly other)
   {
      x -= other.getX();
      y -= other.getY();
      z -= other.getZ();
   }

   public void sub(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      set(tuple1);
      sub(tuple2);
   }

   public void scale(float scalar)
   {
      scale(scalar, scalar, scalar);
   }

   public void scale(float scalarX, float scalarY, float scalarZ)
   {
      x *= scalarX;
      y *= scalarY;
      z *= scalarZ;
   }

   public void scale(float scalar, Tuple3DReadOnly other)
   {
      set(other);
      scale(scalar);
   }

   public void scaleAdd(float scalar, Tuple3DReadOnly other)
   {
      scale(scalar);
      add(other);
   }

   public void scaleAdd(float scalar, Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      scale(scalar, tuple1);
      add(tuple2);
   }

   public void interpolate(Tuple3DReadOnly other, float alpha)
   {
      x = (float) TupleTools.interpolate(x, other.getX(), alpha);
      y = (float) TupleTools.interpolate(y, other.getY(), alpha);
      z = (float) TupleTools.interpolate(z, other.getZ(), alpha);
   }

   public void interpolate(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2, float alpha)
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
      tupleArrayToPack[startIndex++] = y;
      tupleArrayToPack[startIndex] = z;
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
      tupleMatrixToPack.set(startRow++, column, y);
      tupleMatrixToPack.set(startRow, column, z);
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
      case 2:
         return z;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   public void get(Tuple3DBasics other)
   {
      other.setX(x);
      other.setY(y);
      other.setZ(z);
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

   @Override
   public double getZ()
   {
      return z;
   }

   public float getX32()
   {
      return x;
   }

   public float getY32()
   {
      return y;
   }

   public float getZ32()
   {
      return z;
   }

   public boolean epsilonEquals(Tuple32 other, double epsilon)
   {
      return TupleTools.epsilonEquals(this, other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Tuple32) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Tuple32 other)
   {
      try
      {
         return x == other.x && y == other.y && z == other.z;
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "(" + x + ", " + y + ", " + z + ")";
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Float.floatToIntBits(x);
      bits = 31L * bits + Float.floatToIntBits(y);
      bits = 31L * bits + Float.floatToIntBits(z);
      return (int) (bits ^ bits >> 32);
   }
}

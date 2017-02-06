package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class Tuple3D implements Serializable, Tuple3DBasics
{
   private final static long serialVersionUID = -2901667320185555345L;

   private double x, y, z;

   public Tuple3D()
   {
      setToZero();
   }

   public Tuple3D(double x, double y, double z)
   {
      set(x, y, z);
   }

   public Tuple3D(double[] tupleArray)
   {
      set(tupleArray);
   }

   public Tuple3D(Tuple3DReadOnly other)
   {
      set(other);
   }

   @Override
   public void setToNaN()
   {
      x = Double.NaN;
      y = Double.NaN;
      z = Double.NaN;
   }

   @Override
   public void setToZero()
   {
      x = 0.0;
      y = 0.0;
      z = 0.0;
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

   public void clipToMax(double max)
   {
      x = Math.min(max, x);
      y = Math.min(max, y);
      z = Math.min(max, z);
   }

   public void clipToMin(double min)
   {
      x = Math.max(min, x);
      y = Math.max(min, y);
      z = Math.max(min, z);
   }

   public void clipToMinMax(double min, double max)
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

   public void setAndClipToMax(double max, Tuple3DReadOnly other)
   {
      set(other);
      clipToMax(max);
   }

   public void setAndClipToMin(double min, Tuple3DReadOnly other)
   {
      set(other);
      clipToMin(min);
   }

   public void setAndClipToMinMax(double min, double max, Tuple3DReadOnly other)
   {
      set(other);
      clipToMinMax(min, max);
   }

   @Override
   public void set(double x, double y, double z)
   {
      this.x = x;
      this.y = y;
      this.z = z;
   }

   public void set(double[] tupleArray)
   {
      set(tupleArray, 0);
   }

   public void set(double[] tupleArray, int startIndex)
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
      x = matrix.get(startRow++, column);
      y = matrix.get(startRow++, column);
      z = matrix.get(startRow, column);
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
      x = other.getX();
      y = other.getY();
      z = other.getZ();
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
   public void setZ(double z)
   {
      this.z = z;
   }

   @Override
   public void add(double x, double y, double z)
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

   public void scale(double scalar)
   {
      scale(scalar, scalar, scalar);
   }

   public void scale(double scalarX, double scalarY, double scalarZ)
   {
      x *= scalarX;
      y *= scalarY;
      z *= scalarZ;
   }

   public void scale(double scalar, Tuple3DReadOnly other)
   {
      set(other);
      scale(scalar);
   }

   public void scaleAdd(double scalar, Tuple3DReadOnly other)
   {
      scale(scalar);
      add(other);
   }

   public void scaleAdd(double scalar, Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      scale(scalar, tuple1);
      add(tuple2);
   }

   public void interpolate(Tuple3DReadOnly other, double alpha)
   {
      x = TupleTools.interpolate(x, other.getX(), alpha);
      y = TupleTools.interpolate(y, other.getY(), alpha);
      z = TupleTools.interpolate(z, other.getZ(), alpha);
   }

   public void interpolate(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2, double alpha)
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

   public boolean epsilonEquals(Tuple3D other, double epsilon)
   {
      return TupleTools.epsilonEquals(this, other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Tuple3D) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Tuple3D other)
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
      bits = 31L * bits + Double.doubleToLongBits(x);
      bits = 31L * bits + Double.doubleToLongBits(y);
      bits = 31L * bits + Double.doubleToLongBits(z);
      return (int) (bits ^ bits >> 32);
   }
}

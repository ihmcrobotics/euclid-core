package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class Tuple3D<T extends Tuple3D<T>> implements Serializable, Tuple3DBasics<T>
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

   public Tuple3D(Tuple3DReadOnly<?> other)
   {
      set(other);
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

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Tuple3D<?>) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Tuple3D<?> other)
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

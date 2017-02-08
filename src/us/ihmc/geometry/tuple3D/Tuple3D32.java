package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class Tuple3D32<T extends Tuple3D32<T>> implements Serializable, Tuple3DBasics<T>
{
   private static final long serialVersionUID = 7705737376600145100L;

   private float x, y, z;

   public Tuple3D32()
   {
      setToZero();
   }

   public Tuple3D32(float x, float y, float z)
   {
      set(x, y, z);
   }

   public Tuple3D32(float[] tupleArray)
   {
      set(tupleArray);
   }

   public Tuple3D32(Tuple3DReadOnly<?> other)
   {
      set(other);
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

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Tuple3D32<?>) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Tuple3D32<?> other)
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

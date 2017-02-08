package us.ihmc.geometry.tuple4D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;

public class Vector4D32 implements Serializable, Vector4DBasics<Vector4D>
{
   private static final long serialVersionUID = 3048835798807478377L;

   private float x, y, z, s;

   public Vector4D32()
   {
      setToZero();
   }

   public Vector4D32(double x, double y, double z, double s)
   {
      set(x, y, z, s);
   }

   public Vector4D32(double[] vectorArray)
   {
      set(vectorArray);
   }

   public Vector4D32(Tuple4DReadOnly<?> other)
   {
      set(other);
   }

   public void set(Tuple3DReadOnly<?> tuple)
   {
      x = (float) tuple.getX();
      y = (float) tuple.getY();
      z = (float) tuple.getZ();
      s = (float) 0.0;
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

   @Override
   public void setS(double s)
   {
      this.s = (float) s;
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
   public double getS()
   {
      return s;
   }

   @Override
   public float getX32()
   {
      return x;
   }

   @Override
   public float getY32()
   {
      return y;
   }

   @Override
   public float getZ32()
   {
      return z;
   }

   @Override
   public float getS32()
   {
      return s;
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Vector4D) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return Tuple4DTools.toString(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(x);
      bits = 31L * bits + Double.doubleToLongBits(y);
      bits = 31L * bits + Double.doubleToLongBits(z);
      bits = 31L * bits + Double.doubleToLongBits(s);
      return (int) (bits ^ bits >> 32);
   }
}

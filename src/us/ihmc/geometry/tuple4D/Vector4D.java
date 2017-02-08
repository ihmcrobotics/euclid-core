package us.ihmc.geometry.tuple4D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;

public class Vector4D implements Serializable, Vector4DBasics<Vector4D>
{
   private static final long serialVersionUID = 3048835798807478377L;

   private double x, y, z, s;

   public Vector4D()
   {
      setToZero();
   }

   public Vector4D(double x, double y, double z, double s)
   {
      set(x, y, z, s);
   }

   public Vector4D(double[] vectorArray)
   {
      set(vectorArray);
   }

   public Vector4D(Tuple4DReadOnly<?> other)
   {
      set(other);
   }

   public void set(Tuple3DReadOnly<?> tuple)
   {
      x = tuple.getX();
      y = tuple.getY();
      z = tuple.getZ();
      s = 0.0;
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
   public void setS(double s)
   {
      this.s = s;
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

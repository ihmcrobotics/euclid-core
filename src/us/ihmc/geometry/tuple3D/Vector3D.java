package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;

public class Vector3D implements Serializable, Vector3DBasics<Vector3D>
{
   private static final long serialVersionUID = -8494204802892437237L;

   private double x;
   private double y;
   private double z;

   public Vector3D()
   {
      setToZero();
   }

   public Vector3D(double x, double y, double z)
   {
      set(x, y, z);
   }

   public Vector3D(double[] vectorArray)
   {
      set(vectorArray);
   }

   public Vector3D(Tuple3DReadOnly<?> other)
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
         return equals((Vector3D) object);
      }
      catch (ClassCastException e)
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

package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;

public class Vector3D32 implements Serializable, Vector3DBasics<Vector3D32>
{
   private static final long serialVersionUID = 1186918378545386628L;

   private float x;
   private float y;
   private float z;

   public Vector3D32()
   {
      setToZero();
   }

   public Vector3D32(float x, float y, float z)
   {
      set(x, y, z);
   }

   public Vector3D32(float[] vectorArray)
   {
      set(vectorArray);
   }

   public Vector3D32(Tuple3DReadOnly<?> tuple)
   {
      set(tuple);
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
         return equals((Vector3D32) object);
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
      bits = 31L * bits + Float.floatToIntBits(x);
      bits = 31L * bits + Float.floatToIntBits(y);
      bits = 31L * bits + Float.floatToIntBits(z);
      return (int) (bits ^ bits >> 32);
   }
}

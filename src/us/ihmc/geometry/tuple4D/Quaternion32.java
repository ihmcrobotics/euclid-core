package us.ihmc.geometry.tuple4D;

import java.io.Serializable;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public class Quaternion32 implements Serializable, QuaternionBasics<Quaternion32>
{
   private static final long serialVersionUID = -3523313039213464150L;

   private float x, y, z, s;

   public Quaternion32()
   {
      setToZero();
   }

   public Quaternion32(float x, float y, float z, float s)
   {
      set(x, y, z, s);
   }

   public Quaternion32(float[] quaternionArray)
   {
      set(quaternionArray);
   }

   public Quaternion32(QuaternionReadOnly<?> other)
   {
      set(other);
   }

   public Quaternion32(RotationMatrixReadOnly<?> rotationMatrix)
   {
      set(rotationMatrix);
   }

   public Quaternion32(AxisAngleReadOnly<?> axisAngle)
   {
      set(axisAngle);
   }

   public Quaternion32(Vector3DReadOnly<?> rotationVector)
   {
      set(rotationVector);
   }

   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      x = (float) qx;
      y = (float) qy;
      z = (float) qz;
      s = (float) qs;
   }

   @Override
   public double getS()
   {
      return s;
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
   public float getS32()
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
   public boolean equals(Object object)
   {
      try
      {
         return equals((Quaternion32) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public String toStringAsYawPitchRoll()
   {
      return "yaw-pitch-roll: (" + getYaw() + ", " + getPitch() + ", " + getRoll() + ")";
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
      bits = 31L * bits + Float.floatToIntBits(x);
      bits = 31L * bits + Float.floatToIntBits(y);
      bits = 31L * bits + Float.floatToIntBits(z);
      bits = 31L * bits + Float.floatToIntBits(s);
      return (int) (bits ^ bits >> 32);
   }
}

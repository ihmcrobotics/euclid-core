package us.ihmc.geometry.tuple4D;

import java.io.Serializable;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public class Quaternion implements Serializable, QuaternionBasics<Quaternion>
{
   private static final long serialVersionUID = -3523313039213464150L;

   private double x, y, z, s;

   public Quaternion()
   {
      setToZero();
   }

   public Quaternion(double x, double y, double z, double s)
   {
      set(x, y, z, s);
   }

   public Quaternion(double[] quaternionArray)
   {
      set(quaternionArray);
   }

   public Quaternion(QuaternionReadOnly<?> other)
   {
      set(other);
   }

   public Quaternion(RotationMatrixReadOnly<?> rotationMatrix)
   {
      set(rotationMatrix);
   }

   public Quaternion(AxisAngleReadOnly<?> axisAngle)
   {
      set(axisAngle);
   }

   public Quaternion(Vector3DReadOnly<?> rotationVector)
   {
      set(rotationVector);
   }

   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      x = qx;
      y = qy;
      z = qz;
      s = qs;
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
   public boolean equals(Object object)
   {
      try
      {
         return equals((Quaternion) object);
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
      return "(" + x + ", " + y + ", " + z + ", " + s + ")";
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

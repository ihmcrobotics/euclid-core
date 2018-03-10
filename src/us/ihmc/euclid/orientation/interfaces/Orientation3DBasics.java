package us.ihmc.euclid.orientation.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Orientation3DBasics extends Orientation3DReadOnly
{
   void normalize();

   void invert();

   void setFromRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);

   void setFromAxisAngle(double x, double y, double z, double angle);

   void setFromQuaternion(double x, double y, double z, double s);

   void setFromRotationVector(double x, double y, double z);

   void setFromYawPitchRoll(double yaw, double pitch, double roll);

   default void setYawPitchRoll(double[] yawPitchRoll)
   {
      setFromYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   /**
    * Sets the components of this axis-angle such that it represents the same orientation as the
    * given {@code rotationVector}. See
    * {@link AxisAngleConversion#convertRotationVectorToAxisAngle(Vector3DReadOnly, AxisAngleBasics)}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector to convert. Not modified.
    */
   default void set(Vector3DReadOnly rotationVector)
   {
      setFromRotationVector(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ());
   }

   default void setEuler(Vector3DReadOnly eulerAngles)
   {
      setFromYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX());
   }

   default void setEuler(double rotX, double rotY, double rotZ)
   {
      setFromYawPitchRoll(rotZ, rotY, rotX);
   }

   void set(Orientation3DReadOnly orientation3DReadOnly);

   default void setAndNormalize(Orientation3DReadOnly orientation3DReadOnly)
   {
      set(orientation3DReadOnly);
      normalize();
   }

   default void setAndInvert(Orientation3DReadOnly orientation3DReadOnly)
   {
      set(orientation3DReadOnly);
      invert();
   }

   void multiply(Orientation3DReadOnly orientation);

   void multiplyInvertOther(Orientation3DReadOnly orientation);

   void multiplyInvertThis(Orientation3DReadOnly orientation);

   void multiplyInvertBoth(Orientation3DReadOnly orientation);
   
   void preMultiply(Orientation3DReadOnly orientation);
   
   void preMultiplyInvertOther(Orientation3DReadOnly orientation);
   
   void preMultiplyInvertThis(Orientation3DReadOnly orientation);

   void preMultiplyInvertBoth(Orientation3DReadOnly orientation);
}

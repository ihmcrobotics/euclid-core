package us.ihmc.euclid.orientation.interfaces;

public interface Orientation3DBasics extends Orientation3DReadOnly
{
   void set(Orientation3DReadOnly orientation3DReadOnly);

   void setFromRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);

   void setFromAxisAngle(double x, double y, double z, double angle);

   void setFromQuaternion(double x, double y, double z, double s);

   void setFromRotationVector(double x, double y, double z);

   void setFromYawPitchRoll(double yaw, double pitch, double roll);
}

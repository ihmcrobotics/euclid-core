package us.ihmc.euclid.orientation.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;

public interface Orientation3DReadOnly
{
   void get(RotationMatrix rotationMatrixToPack);

   void get(AxisAngleBasics axisAngleToPack);

   void get(QuaternionBasics quaternionToPack);

   void getRotationVector(Vector3DBasics rotationVectorToPack);

   void getYawPitchRoll(double[] yawPitchRollToPack);

   double getYaw();

   double getPitch();

   double getRoll();
}

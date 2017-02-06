package us.ihmc.geometry.tuple;

import us.ihmc.geometry.axisAngle.AxisAngleTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.Matrix3DFeatures;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.tuple.interfaces.Vector3DBasics;
import us.ihmc.geometry.tuple4D.Tuple4DTools;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public abstract class RotationVectorConversion
{
   private static final double EPS = 1.0e-12;

   public static void convertAxisAngleToRotationVector(AxisAngleReadOnly<?> axisAngle, Vector3DBasics rotationVectorToPack)
   {
      convertAxisAngleToRotationVectorImpl(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle(), rotationVectorToPack);
   }

   public static void convertAxisAngleToRotationVectorImpl(double ux, double uy, double uz, double angle, Vector3DBasics rotationVectorToPack)
   {
      if (AxisAngleTools.containsNaN(ux, uy, uz, angle))
      {
         rotationVectorToPack.setToNaN();
         return;
      }

      double uNorm = Math.sqrt(ux * ux + uy * uy + uz * uz);

      if (uNorm > EPS)
      {
         uNorm = 1.0 / uNorm;
         rotationVectorToPack.setX(ux * uNorm * angle);
         rotationVectorToPack.setY(uy * uNorm * angle);
         rotationVectorToPack.setZ(uz * uNorm * angle);
      }
      else
      {
         rotationVectorToPack.setToZero();
      }
   }

   public static void convertQuaternionToRotationVector(QuaternionReadOnly quaternion, Vector3DBasics rotationVectorToPack)
   {
      convertQuaternionToRotationVectorImpl(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), rotationVectorToPack);
   }

   public static void convertQuaternionToRotationVectorImpl(double qx, double qy, double qz, double qs, Vector3DBasics rotationVectorToPack)
   {
      if (Tuple4DTools.containsNaN(qx, qy, qz, qs))
      {
         rotationVectorToPack.setToNaN();
         return;
      }

      double uNorm = qx * qx + qy * qy + qz * qz;

      if (uNorm > EPS)
      {
         uNorm = Math.sqrt(uNorm);
         double angle = 2.0 * Math.atan2(uNorm, qs) / uNorm;
         rotationVectorToPack.setX(qx * angle);
         rotationVectorToPack.setY(qy * angle);
         rotationVectorToPack.setZ(qz * angle);
      }
      else
      {
         rotationVectorToPack.setToZero();
      }
   }

   public static void convertMatrixToRotationVector(RotationScaleMatrixReadOnly<?> rotationScaleMatrix, Vector3DBasics rotationVectorToPack)
   {
      convertMatrixToRotationVector(rotationScaleMatrix.getRotationMatrix(), rotationVectorToPack);
   }

   public static void convertMatrixToRotationVector(RotationMatrixReadOnly<?> rotationMatrix, Vector3DBasics rotationVectorToPack)
   {
      double m00 = rotationMatrix.getM00();
      double m01 = rotationMatrix.getM01();
      double m02 = rotationMatrix.getM02();
      double m10 = rotationMatrix.getM10();
      double m11 = rotationMatrix.getM11();
      double m12 = rotationMatrix.getM12();
      double m20 = rotationMatrix.getM20();
      double m21 = rotationMatrix.getM21();
      double m22 = rotationMatrix.getM22();

      convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, rotationVectorToPack);
   }

   public static void convertMatrixToRotationVectorImpl(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
         Vector3DBasics rotationVectorToPack)
   {
      if (Matrix3DFeatures.containsNaN(m00, m01, m02, m10, m11, m12, m20, m21, m22))
      {
         rotationVectorToPack.setToNaN();
         return;
      }

      double angle, x, y, z; // variables for result

      x = m21 - m12;
      y = m02 - m20;
      z = m10 - m01;

      double s = Math.sqrt(x * x + y * y + z * z);

      if (s > EPS)
      {
         double sin = 0.5 * s;
         double cos = 0.5 * (m00 + m11 + m22 - 1.0);
         angle = Math.atan2(sin, cos);
         x /= s;
         y /= s;
         z /= s;
      }
      else if (Matrix3DFeatures.isZeroRotation(m00, m01, m02, m10, m11, m12, m20, m21, m22))
      {
         rotationVectorToPack.setToZero();
         return;
      }
      else
      {
         // otherwise this singularity is angle = 180
         angle = Math.PI;
         double xx = 0.50 * (m00 + 1.0);
         double yy = 0.50 * (m11 + 1.0);
         double zz = 0.50 * (m22 + 1.0);
         double xy = 0.25 * (m01 + m10);
         double xz = 0.25 * (m02 + m20);
         double yz = 0.25 * (m12 + m21);

         if (xx > yy && xx > zz)
         { // m00 is the largest diagonal term
            x = Math.sqrt(xx);
            y = xy / x;
            z = xz / x;
         }
         else if (yy > zz)
         { // m11 is the largest diagonal term
            y = Math.sqrt(yy);
            x = xy / y;
            z = yz / y;
         }
         else
         { // m22 is the largest diagonal term so base result on this
            z = Math.sqrt(zz);
            x = xz / z;
            y = yz / z;
         }
      }

      rotationVectorToPack.setX(x * angle);
      rotationVectorToPack.setY(y * angle);
      rotationVectorToPack.setZ(z * angle);
   }

   public static void convertYawPitchRollToRotationVector(double[] yawPitchRoll, Vector3DBasics rotationVectorToPack)
   {
      convertYawPitchRollToRotationVector(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], rotationVectorToPack);
   }

   public static void convertYawPitchRollToRotationVector(double yaw, double pitch, double roll, Vector3DBasics rotationVectorToPack)
   {
      double halfYaw = yaw / 2.0;
      double cYaw = Math.cos(halfYaw);
      double sYaw = Math.sin(halfYaw);

      double halfPitch = pitch / 2.0;
      double cPitch = Math.cos(halfPitch);
      double sPitch = Math.sin(halfPitch);

      double halfRoll = roll / 2.0;
      double cRoll = Math.cos(halfRoll);
      double sRoll = Math.sin(halfRoll);

      double qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
      double qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
      double qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
      double qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;

      convertQuaternionToRotationVectorImpl(qx, qy, qz, qs, rotationVectorToPack);
   }
}

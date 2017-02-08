package us.ihmc.geometry.tuple4D;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.geometry.axisAngle.AxisAngleTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.Matrix3DFeatures;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;

public abstract class QuaternionConversion
{
   private static final double EPS = 1.0e-7;

   public static final void computeYawQuaternion(double yaw, QuaternionBasics quaternionToPack)
   {
      double halfYaw = 0.5 * yaw;
      quaternionToPack.setUnsafe(0.0, 0.0, Math.sin(halfYaw), Math.cos(halfYaw));
   }

   public static final void computePitchQuaternion(double pitch, QuaternionBasics quaternionToPack)
   {
      double halfPitch = 0.5 * pitch;
      quaternionToPack.setUnsafe(0.0, Math.sin(halfPitch), 0.0, Math.cos(halfPitch));
   }

   public static final void computeRollQuaternion(double roll, QuaternionBasics quaternionToPack)
   {
      double halfRoll = 0.5 * roll;
      quaternionToPack.setUnsafe(Math.sin(halfRoll), 0.0, 0.0, Math.cos(halfRoll));
   }

   public static final void convertAxisAngleToQuaternion(AxisAngleReadOnly<?> axisAngle, QuaternionBasics quaternionToPack)
   {
      convertAxisAngleToQuaternionImpl(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle(), quaternionToPack);
   }

   public static final void convertAxisAngleToQuaternionImpl(double ux, double uy, double uz, double angle, QuaternionBasics quaternionToPack)
   {
      if (AxisAngleTools.containsNaN(ux, uy, uz, angle))
      {
         quaternionToPack.setToNaN();
         return;
      }

      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);
      if (uNorm < EPS)
      {
         quaternionToPack.setToZero();
      }
      else
      {
         double halfTheta = 0.5 * angle;
         double cosHalfTheta = Math.cos(halfTheta);
         double sinHalfTheta = Math.sin(halfTheta) / uNorm;
         quaternionToPack.setUnsafe(ux * sinHalfTheta, uy * sinHalfTheta, uz * sinHalfTheta, cosHalfTheta);
      }
   }

   public static void convertMatrixToQuaternion(RotationScaleMatrixReadOnly<?> rotationScaleMatrix, QuaternionBasics quaternionToPack)
   {
      convertMatrixToQuaternion(rotationScaleMatrix.getRotationMatrix(), quaternionToPack);
   }

   public static void convertMatrixToQuaternion(RotationMatrixReadOnly<?> rotationMatrix, QuaternionBasics quaternionToPack)
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
      convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, quaternionToPack);
   }

   static void convertMatrixToQuaternionImpl(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
                                             QuaternionBasics quaternionToPack)
   {
      if (Matrix3DFeatures.containsNaN(m00, m01, m02, m10, m11, m12, m20, m21, m22))
      {
         quaternionToPack.setToNaN();
         return;
      }

      // There are different ways to compute the quaternions elements from the matrix. They all involve computing one element from
      // the diagonal of the matrix, and computing the three other ones using a formula involving a division by the first element,
      // which unfortunately can be zero. Since the norm of the quaternion is 1, we know at least one element has an absolute
      // value greater or equal to 0.5, so it is always possible to select the right formula and avoid division by zero and even
      // numerical inaccuracy. Checking the elements in turn and using the first one greater than 0.45 is safe (this leads to a simple
      // test since qi = 0.45 implies 4 qi^2 - 1 = -0.19)
      double s = m00 + m11 + m22;

      double qx, qy, qz, qs;

      if (s > -0.19)
      {
         // compute q0 and deduce q1, q2 and q3
         qs = 0.5 * FastMath.sqrt(s + 1.0);
         double inv = 0.25 / qs;
         qx = inv * (m21 - m12);
         qy = inv * (m02 - m20);
         qz = inv * (m10 - m01);
      }
      else
      {
         s = m00 - m11 - m22;

         if (s > -0.19)
         {
            // compute q1 and deduce q0, q2 and q3
            qx = 0.5 * FastMath.sqrt(s + 1.0);
            double inv = 0.25 / qx;
            qs = inv * (m21 - m12);
            qy = inv * (m10 + m01);
            qz = inv * (m20 + m02);
         }
         else
         {
            s = m11 - m00 - m22;

            if (s > -0.19)
            {
               // compute q2 and deduce q0, q1 and q3
               qy = 0.5 * FastMath.sqrt(s + 1.0);
               double inv = 0.25 / qy;
               qs = inv * (m02 - m20);
               qx = inv * (m10 + m01);
               qz = inv * (m12 + m21);
            }
            else
            {
               // compute q3 and deduce q0, q1 and q2
               s = m22 - m00 - m11;
               qz = 0.5 * FastMath.sqrt(s + 1.0);
               double inv = 0.25 / qz;
               qs = inv * (m10 - m01);
               qx = inv * (m20 + m02);
               qy = inv * (m12 + m21);
            }
         }
      }
      quaternionToPack.setUnsafe(qx, qy, qz, qs);
   }

   public static void convertRotationVectorToQuaternion(Vector3DReadOnly<?> rotationVector, QuaternionBasics quaternionToPack)
   {
      convertRotationVectorToQuaternionImpl(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ(), quaternionToPack);
   }

   public static void convertRotationVectorToQuaternionImpl(double rx, double ry, double rz, QuaternionBasics quaternionToPack)
   {
      if (Double.isNaN(rx) || Double.isNaN(ry) || Double.isNaN(rz))
      {
         quaternionToPack.setToNaN();
         return;
      }

      double norm = Math.sqrt(rx * rx + ry * ry + rz * rz);

      if (norm < EPS)
      {
         quaternionToPack.setToZero();
      }
      else
      {
         double halfTheta = 0.5 * norm;
         double cosHalfTheta = Math.cos(halfTheta);
         double sinHalfTheta = Math.sin(halfTheta) / norm;
         quaternionToPack.setUnsafe(rx * sinHalfTheta, ry * sinHalfTheta, rz * sinHalfTheta, cosHalfTheta);
      }
   }

   public static void convertYawPitchRollToQuaternion(double[] yawPitchRoll, QuaternionBasics quaternionToPack)
   {
      convertYawPitchRollToQuaternion(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], quaternionToPack);
   }

   public static void convertYawPitchRollToQuaternion(double yaw, double pitch, double roll, QuaternionBasics quaternionToPack)
   {
      double halfYaw = 0.5 * yaw;
      double cYaw = Math.cos(halfYaw);
      double sYaw = Math.sin(halfYaw);

      double halfPitch = 0.5 * pitch;
      double cPitch = Math.cos(halfPitch);
      double sPitch = Math.sin(halfPitch);

      double halfRoll = 0.5 * roll;
      double cRoll = Math.cos(halfRoll);
      double sRoll = Math.sin(halfRoll);

      double qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
      double qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
      double qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
      double qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
      quaternionToPack.setUnsafe(qx, qy, qz, qs);
   }
}

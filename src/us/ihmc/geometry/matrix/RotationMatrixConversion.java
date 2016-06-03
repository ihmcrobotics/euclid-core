package us.ihmc.geometry.matrix;

import us.ihmc.geometry.axisAngle.AxisAngleTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple4D.QuaternionTools;
import us.ihmc.geometry.tuple4D.Tuple4DTools;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public abstract class RotationMatrixConversion
{
   private static final double EPS = 1.0e-12;

   public static void computePitchMatrix(double pitch, RotationMatrix matrixToPack)
   {
      double sinPitch = Math.sin(pitch);
      double cosPitch = Math.cos(pitch);
      matrixToPack.setUnsafe(cosPitch, 0.0, sinPitch, 0.0, 1.0, 0.0, -sinPitch, 0.0, cosPitch);
   }

   public static void computeRollMatrix(double roll, RotationMatrix matrixToPack)
   {
      double sinRoll = Math.sin(roll);
      double cosRoll = Math.cos(roll);
      matrixToPack.setUnsafe(1.0, 0.0, 0.0, 0.0, cosRoll, -sinRoll, 0.0, sinRoll, cosRoll);
   }

   public static void computeYawMatrix(double yaw, RotationMatrix matrixToPack)
   {
      double sinYaw = Math.sin(yaw);
      double cosYaw = Math.cos(yaw);
      matrixToPack.setUnsafe(cosYaw, -sinYaw, 0.0, sinYaw, cosYaw, 0.0, 0.0, 0.0, 1.0);
   }

   public static void convertAxisAngleToMatrix(AxisAngleReadOnly axisAngle, RotationMatrix matrixToPack)
   {
      convertAxisAngleToMatrixImpl(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle(), matrixToPack);
   }

   public static void convertAxisAngleToMatrixImpl(double ux, double uy, double uz, double angle, RotationMatrix matrixToPack)
   {
      if (AxisAngleTools.containsNaN(ux, uy, uz, angle))
      {
         matrixToPack.setToNaN();
         return;
      }

      double uNorm = Math.sqrt(ux * ux + uy * uy + uz * uz);

      if (uNorm < EPS)
      {
         matrixToPack.setIdentity();
      }
      else
      {
         uNorm = 1.0 / uNorm;
         double ax = ux * uNorm;
         double ay = uy * uNorm;
         double az = uz * uNorm;

         double sinTheta = Math.sin(angle);
         double cosTheta = Math.cos(angle);
         double t = 1.0 - cosTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         double m00 = t * ax * ax + cosTheta;
         double m01 = t * xy - sinTheta * az;
         double m02 = t * xz + sinTheta * ay;
         double m10 = t * xy + sinTheta * az;
         double m11 = t * ay * ay + cosTheta;
         double m12 = t * yz - sinTheta * ax;
         double m20 = t * xz - sinTheta * ay;
         double m21 = t * yz + sinTheta * ax;
         double m22 = t * az * az + cosTheta;
         matrixToPack.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      }
   }

   public static void convertQuaternionToMatrix(QuaternionReadOnly quaternion, RotationMatrix matrixToPack)
   {
      convertQuaternionToMatrixImpl(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), matrixToPack);
   }
   
   public static void convertQuaternionToMatrixImpl(double qx, double qy, double qz, double qs, RotationMatrix matrixToPack)
   {
      if (Tuple4DTools.containsNaN(qx, qy, qz, qs))
      {
         matrixToPack.setToNaN();
         return;
      }

      double norm = QuaternionTools.norm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         matrixToPack.setIdentity();
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double yy2 = 2.0 * qy * qy;
      double zz2 = 2.0 * qz * qz;
      double xx2 = 2.0 * qx * qx;
      double xy2 = 2.0 * qx * qy;
      double sz2 = 2.0 * qs * qz;
      double xz2 = 2.0 * qx * qz;
      double sy2 = 2.0 * qs * qy;
      double yz2 = 2.0 * qy * qz;
      double sx2 = 2.0 * qs * qx;

      double m00 = 1.0 - yy2 - zz2;
      double m01 = xy2 - sz2;
      double m02 = xz2 + sy2;
      double m10 = xy2 + sz2;
      double m11 = 1.0 - xx2 - zz2;
      double m12 = yz2 - sx2;
      double m20 = xz2 - sy2;
      double m21 = yz2 + sx2;
      double m22 = 1.0 - xx2 - yy2;
      matrixToPack.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void convertYawPitchRollToMatrix(double[] yawPitchRoll, RotationMatrix matrixToPack)
   {
      convertYawPitchRollToMatrix(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], matrixToPack);
   }

   /**
    * Sets the rotation matrix, based on the yaw, pitch and roll values.
    * @param yaw yaw rotation (about a fixed z-axis)
    * @param pitch pitch rotation (about a fixed y-axis)
    * @param roll roll rotation (about a fixed x-axis)
    * @param matrixToPack the rotation matrix to set, based on the yaw, pitch and roll values
    */
   public static void convertYawPitchRollToMatrix(double yaw, double pitch, double roll, RotationMatrix matrixToPack)
   {
      double cosc = Math.cos(yaw);
      double sinc = Math.sin(yaw);

      double cosb = Math.cos(pitch);
      double sinb = Math.sin(pitch);

      double cosa = Math.cos(roll);
      double sina = Math.sin(roll);

      // Introduction to Robotics, 2.64
      double m00 = cosc * cosb;
      double m01 = cosc * sinb * sina - sinc * cosa;
      double m02 = cosc * sinb * cosa + sinc * sina;
      double m10 = sinc * cosb;
      double m11 = sinc * sinb * sina + cosc * cosa;
      double m12 = sinc * sinb * cosa - cosc * sina;
      double m20 = -sinb;
      double m21 = cosb * sina;
      double m22 = cosb * cosa;
      matrixToPack.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void convertRotationVectorToMatrix(VectorReadOnly rotationVector, RotationMatrix matrixToPack)
   {
      convertRotationVectorToMatrixImpl(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ(), matrixToPack);
   }

   public static void convertRotationVectorToMatrixImpl(double rx, double ry, double rz, RotationMatrix matrixToPack)
   {
      if (Double.isNaN(rx) || Double.isNaN(ry) || Double.isNaN(rz))
      {
         matrixToPack.setToNaN();
         return;
      }

      double norm = Math.sqrt(rx * rx + ry * ry + rz * rz);

      if (norm < EPS)
      {
         matrixToPack.setIdentity();
      }
      else
      {
         double sinTheta = Math.sin(norm);
         double cosTheta = Math.cos(norm);
         double t = 1.0 - cosTheta;

         norm = 1.0 / norm;
         double ax = rx * norm;
         double ay = ry * norm;
         double az = rz * norm;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         double m00 = t * ax * ax + cosTheta;
         double m01 = t * xy - sinTheta * az;
         double m02 = t * xz + sinTheta * ay;
         double m10 = t * xy + sinTheta * az;
         double m11 = t * ay * ay + cosTheta;
         double m12 = t * yz - sinTheta * ax;
         double m20 = t * xz - sinTheta * ay;
         double m21 = t * yz + sinTheta * ax;
         double m22 = t * az * az + cosTheta;
         matrixToPack.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      }
   }
}

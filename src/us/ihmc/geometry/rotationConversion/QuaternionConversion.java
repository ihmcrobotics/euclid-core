package us.ihmc.geometry.rotationConversion;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.tools.EuclidCoreTools;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;

/**
 * This class gathers all the methods necessary to converts any type of rotation into a quaternion.
 * <p>
 * To convert an orientation into other data structure types see:
 * <ul>
 * <li>for axis-angle: {@link AxisAngleConversion},
 * <li>for rotation matrix: {@link RotationMatrixConversion},
 * <li>for rotation vector: {@link RotationVectorConversion},
 * <li>for yaw-pitch-roll: {@link YawPitchRollConversion}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public abstract class QuaternionConversion
{
   private static final double EPS = 1.0e-7;

   /**
    * Sets the given quaternion to represent a counter clockwise rotation around the z-axis of an
    * angle {@code yaw}.
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static final void computeYawQuaternion(double yaw, QuaternionBasics quaternionToPack)
   {
      double halfYaw = 0.5 * yaw;
      quaternionToPack.setUnsafe(0.0, 0.0, Math.sin(halfYaw), Math.cos(halfYaw));
   }

   /**
    * Sets the given quaternion to represent a counter clockwise rotation around the y-axis of an
    * angle {@code pitch}.
    *
    * @param pitch the angle to rotate about the y-axis.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static final void computePitchQuaternion(double pitch, QuaternionBasics quaternionToPack)
   {
      double halfPitch = 0.5 * pitch;
      quaternionToPack.setUnsafe(0.0, Math.sin(halfPitch), 0.0, Math.cos(halfPitch));
   }

   /**
    * Sets the given quaternion to represent a counter clockwise rotation around the x-axis of an
    * angle {@code roll}.
    *
    * @param roll the angle to rotate about the x-axis.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static final void computeRollQuaternion(double roll, QuaternionBasics quaternionToPack)
   {
      double halfRoll = 0.5 * roll;
      quaternionToPack.setUnsafe(Math.sin(halfRoll), 0.0, 0.0, Math.cos(halfRoll));
   }

   /**
    * Converts the given axis-angle into a quaternion.
    * <p>
    * After calling this method, the axis-angle and the quaternion represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the axis-angle is {@link Double#NaN}, the quaternion is set to
    * {@link Double#NaN}.
    * <li>if the length of the axis is below {@link #EPS}, the rotation matrix is set to the neutral
    * quaternion.
    * </ul>
    * </p>
    *
    * @param axisAngle the axis-angle to use for the conversion. Not modified.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static final void convertAxisAngleToQuaternion(AxisAngleReadOnly axisAngle, QuaternionBasics quaternionToPack)
   {
      convertAxisAngleToQuaternion(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle(), quaternionToPack);
   }

   /**
    * Converts the given axis-angle into a quaternion.
    * <p>
    * After calling this method, the axis-angle and the quaternion represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the axis-angle is {@link Double#NaN}, the quaternion is set to
    * {@link Double#NaN}.
    * <li>if the length of the axis is below {@link #EPS}, the rotation matrix is set to the neutral
    * quaternion.
    * </ul>
    * </p>
    *
    * @param ux the axis x-component of the axis-angle to use for the conversion.
    * @param uy the axis y-component of the axis-angle to use for the conversion.
    * @param uz the axis z-component of the axis-angle to use for the conversion.
    * @param angle the angle of the axis-angle to use for the conversion.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static final void convertAxisAngleToQuaternion(double ux, double uy, double uz, double angle, QuaternionBasics quaternionToPack)
   {
      if (EuclidCoreTools.containsNaN(ux, uy, uz, angle))
      {
         quaternionToPack.setToNaN();
         return;
      }

      double uNorm = EuclidCoreTools.norm(ux, uy, uz);
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

   /**
    * Converts the given rotation part of the given rotation-scale matrix into a quaternion.
    * <p>
    * After calling this method, the rotation part of the rotation-scale matrix and the quaternion
    * represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the quaternion is set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationScaleMatrix a 3-by-3 matrix representing an orientation and a scale. Only the
    *           orientation part is used during the conversion. Not modified.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static void convertMatrixToQuaternion(RotationScaleMatrixReadOnly rotationScaleMatrix, QuaternionBasics quaternionToPack)
   {
      convertMatrixToQuaternion(rotationScaleMatrix.getRotationMatrix(), quaternionToPack);
   }

   /**
    * Converts the given rotation matrix into a quaternion.
    * <p>
    * After calling this method, the rotation matrix and the quaternion represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the quaternion is set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationMatrix a 3-by-3 matrix representing an orientation. Not modified.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static void convertMatrixToQuaternion(RotationMatrixReadOnly rotationMatrix, QuaternionBasics quaternionToPack)
   {
      if (rotationMatrix.containsNaN())
      {
         quaternionToPack.setToNaN();
         return;
      }

      double m00 = rotationMatrix.getM00();
      double m01 = rotationMatrix.getM01();
      double m02 = rotationMatrix.getM02();
      double m10 = rotationMatrix.getM10();
      double m11 = rotationMatrix.getM11();
      double m12 = rotationMatrix.getM12();
      double m20 = rotationMatrix.getM20();
      double m21 = rotationMatrix.getM21();
      double m22 = rotationMatrix.getM22();

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
         qs = 0.5 * Math.sqrt(s + 1.0);
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
            qx = 0.5 * Math.sqrt(s + 1.0);
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
               qy = 0.5 * Math.sqrt(s + 1.0);
               double inv = 0.25 / qy;
               qs = inv * (m02 - m20);
               qx = inv * (m10 + m01);
               qz = inv * (m12 + m21);
            }
            else
            {
               // compute q3 and deduce q0, q1 and q2
               s = m22 - m00 - m11;
               qz = 0.5 * Math.sqrt(s + 1.0);
               double inv = 0.25 / qz;
               qs = inv * (m10 - m01);
               qx = inv * (m20 + m02);
               qy = inv * (m12 + m21);
            }
         }
      }
      quaternionToPack.setUnsafe(qx, qy, qz, qs);
   }

   /**
    * Converts the rotation vector into a quaternion.
    * <p>
    * After calling this method, the rotation vector and the quaternion represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least a {@link Double#NaN}, the quaternion is set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector to use in the conversion. Not modified.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static void convertRotationVectorToQuaternion(Vector3DReadOnly rotationVector, QuaternionBasics quaternionToPack)
   {
      convertRotationVectorToQuaternion(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ(), quaternionToPack);
   }

   /**
    * Converts the rotation vector into a quaternion.
    * <p>
    * After calling this method, the rotation vector and the quaternion represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least a {@link Double#NaN}, the quaternion is set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rx the x-component of the rotation vector to use in the conversion.
    * @param ry the y-component of the rotation vector to use in the conversion.
    * @param rz the z-component of the rotation vector to use in the conversion.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static void convertRotationVectorToQuaternion(double rx, double ry, double rz, QuaternionBasics quaternionToPack)
   {
      if (EuclidCoreTools.containsNaN(rx, ry, rz))
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

   /**
    * Converts the given yaw-pitch-roll angles into a quaternion.
    * <p>
    * After calling this method, the yaw-pitch-roll and the quaternion represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either of the yaw, pitch, or roll angle is {@link Double#NaN}, the quaternion is set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
    * representation of an orientation by decomposing it by three successive rotations around the
    * three axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such
    * representation is: <br>
    * R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll) </br>
    * </p>
    *
    * @param yawPitchRoll the yaw-pitch-roll angles to use in the conversion. Not modified.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
   public static void convertYawPitchRollToQuaternion(double[] yawPitchRoll, QuaternionBasics quaternionToPack)
   {
      convertYawPitchRollToQuaternion(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], quaternionToPack);
   }

   /**
    * Converts the given yaw-pitch-roll angles into a quaternion.
    * <p>
    * After calling this method, the yaw-pitch-roll and the quaternion represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either of the yaw, pitch, or roll angle is {@link Double#NaN}, the quaternion is set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
    * representation of an orientation by decomposing it by three successive rotations around the
    * three axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such
    * representation is: <br>
    * R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll) </br>
    * </p>
    *
    * @param yaw the yaw angle to use in the conversion.
    * @param pitch the pitch angle to use in the conversion.
    * @param roll the roll angle to use in the conversion.
    * @param quaternionToPack the quaternion in which the result is stored.
    */
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

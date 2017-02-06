package us.ihmc.geometry.yawPitchRoll;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.geometry.axisAngle.AxisAngleTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.tuple.TupleTools;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple4D.QuaternionTools;
import us.ihmc.geometry.tuple4D.Tuple4DTools;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public abstract class YawPitchRollConversion
{
   public static final double SAFE_THRESHOLD_PITCH = Math.toRadians(1.82);
   public static final double MAX_PITCH_ANGLE = Math.PI / 2.0 - SAFE_THRESHOLD_PITCH;
   public static final double MIN_PITCH_ANGLE = -MAX_PITCH_ANGLE;

   private static final double EPS = 1.0e-12;

   static double computeYawImpl(double m00, double m10)
   {
      if (Double.isNaN(m00) || Double.isNaN(m10))
         return Double.NaN;

      return Math.atan2(m10, m00);
   }

   static double computePitchImpl(double m20)
   {
      if (Double.isNaN(m20))
         return Double.NaN;
      double pitch = Math.asin(-m20);

      if (Math.abs(pitch) > MAX_PITCH_ANGLE)
         return Double.NaN;

      return pitch;
   }

   static double computeRollImpl(double m21, double m22)
   {
      if (Double.isNaN(m21) || Double.isNaN(m22))
         return Double.NaN;

      return Math.atan2(m21, m22);
   }

   public static double computeYaw(RotationMatrixReadOnly<?> rotationMatrix)
   {
      rotationMatrix.normalize();

      if (Double.isNaN(computePitchImpl(rotationMatrix.getM20())))
         return Double.NaN;
      else
         return computeYawImpl(rotationMatrix.getM00(), rotationMatrix.getM10());
   }

   public static double computePitch(RotationMatrixReadOnly<?> rotationMatrix)
   {
      rotationMatrix.normalize();
      return computePitchImpl(rotationMatrix.getM20());
   }

   public static double computeRoll(RotationMatrixReadOnly<?> rotationMatrix)
   {
      rotationMatrix.normalize();

      if (Double.isNaN(computePitchImpl(rotationMatrix.getM20())))
         return Double.NaN;
      else
         return computeRollImpl(rotationMatrix.getM21(), rotationMatrix.getM22());
   }

   public static double computeYaw(RotationScaleMatrixReadOnly<?> rotationScaleMatrix)
   {
      return computeYaw(rotationScaleMatrix.getRotationMatrix());
   }

   public static double computePitch(RotationScaleMatrixReadOnly<?> rotationScaleMatrix)
   {
      return computePitch(rotationScaleMatrix.getRotationMatrix());
   }

   public static double computeRoll(RotationScaleMatrixReadOnly<?> rotationScaleMatrix)
   {
      return computeRoll(rotationScaleMatrix.getRotationMatrix());
   }

   public static void convertMatrixToYawPitchRoll(RotationMatrixReadOnly<?> rotationMatrix, double[] yawPitchRollToPack)
   {
      rotationMatrix.normalize();
      double pitch = computePitchImpl(rotationMatrix.getM20());
      yawPitchRollToPack[1] = pitch;
      if (Double.isNaN(pitch))
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
      }
      else
      {
         yawPitchRollToPack[0] = computeYawImpl(rotationMatrix.getM00(), rotationMatrix.getM10());
         yawPitchRollToPack[2] = computeRollImpl(rotationMatrix.getM21(), rotationMatrix.getM22());
      }
   }

   public static void convertMatrixToYawPitchRoll(RotationMatrixReadOnly<?> rotationMatrix, Tuple3DBasics eulerAnglesToPack)
   {
      rotationMatrix.normalize();
      double pitch = computePitchImpl(rotationMatrix.getM20());
      eulerAnglesToPack.setY(pitch);

      if (Double.isNaN(pitch))
      {
         eulerAnglesToPack.setX(Double.NaN);
         eulerAnglesToPack.setZ(Double.NaN);
      }
      else
      {
         eulerAnglesToPack.setX(computeRollImpl(rotationMatrix.getM21(), rotationMatrix.getM22()));
         eulerAnglesToPack.setZ(computeYawImpl(rotationMatrix.getM00(), rotationMatrix.getM10()));
      }
   }

   public static void convertMatrixToYawPitchRoll(RotationScaleMatrixReadOnly<?> rotationScaleMatrix, double[] yawPitchRollToPack)
   {
      convertMatrixToYawPitchRoll(rotationScaleMatrix.getRotationMatrix(), yawPitchRollToPack);
   }

   public static void convertMatrixToYawPitchRoll(RotationScaleMatrixReadOnly<?> rotationScaleMatrix, Tuple3DBasics eulerAnglesToPack)
   {
      convertMatrixToYawPitchRoll(rotationScaleMatrix.getRotationMatrix(), eulerAnglesToPack);
   }
   
   static double computeYawFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      return FastMath.atan2(2.0 * (qx * qy + qz * qs), 1.0 - 2.0 * (qy * qy + qz * qz));
   }
   
   static double computePitchFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      double pitchArgument = 2.0 * (qs * qy - qx * qz);

      double pitch = FastMath.asin(pitchArgument);
      if (Math.abs(pitch) > MAX_PITCH_ANGLE)
         return Double.NaN;
      return pitch;
   }

   static double computeRollFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      return FastMath.atan2(2.0 * (qy * qz + qx * qs), 1.0 - 2.0 * (qx * qx + qy * qy));
   }

   public static double computeYaw(QuaternionReadOnly quaternion)
   {
      if (Tuple4DTools.containsNaN(quaternion))
         return Double.NaN;

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = QuaternionTools.norm(quaternion);
      if (norm < EPS)
         return 0.0;

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      if (Double.isNaN(computePitchFromQuaternionImpl(qx, qy, qz, qs)))
         return Double.NaN;
      else
         return computeYawFromQuaternionImpl(qx, qy, qz, qs);
   }

   public static double computePitch(QuaternionReadOnly quaternion)
   {
      if (Tuple4DTools.containsNaN(quaternion))
         return Double.NaN;

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = QuaternionTools.norm(quaternion);
      if (norm < EPS)
         return 0.0;

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      return computePitchFromQuaternionImpl(qx, qy, qz, qs);
   }

   public static double computeRoll(QuaternionReadOnly quaternion)
   {
      if (Tuple4DTools.containsNaN(quaternion))
         return Double.NaN;

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = QuaternionTools.norm(quaternion);
      if (norm < EPS)
         return 0.0;

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      if (Double.isNaN(computePitchFromQuaternionImpl(qx, qy, qz, qs)))
         return Double.NaN;
      else
         return computeRollFromQuaternionImpl(qx, qy, qz, qs);
   }

   public static void convertQuaternionToYawPitchRoll(QuaternionReadOnly quaternion, double[] yawPitchRollToPack)
   {
      if (Tuple4DTools.containsNaN(quaternion))
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[1] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
         return;
      }

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = QuaternionTools.norm(quaternion);
      if (norm < EPS)
      {
         yawPitchRollToPack[0] = 0.0;
         yawPitchRollToPack[1] = 0.0;
         yawPitchRollToPack[2] = 0.0;
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double pitch = computePitchFromQuaternionImpl(qx, qy, qz, qs);
      yawPitchRollToPack[1] = pitch;
      if (Double.isNaN(pitch))
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
      }
      else
      {
         yawPitchRollToPack[0] = computeYawFromQuaternionImpl(qx, qy, qz, qs);
         yawPitchRollToPack[2] = computeRollFromQuaternionImpl(qx, qy, qz, qs);
      }
   }

   public static void convertQuaternionToYawPitchRoll(QuaternionReadOnly quaternion, Vector eulerAnglesToPack)
   {
      if (Tuple4DTools.containsNaN(quaternion))
      {
         eulerAnglesToPack.setToNaN();
         return;
      }

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = QuaternionTools.norm(quaternion);
      if (norm < EPS)
      {
         eulerAnglesToPack.setToZero();
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double pitch = computePitchFromQuaternionImpl(qx, qy, qz, qs);
      eulerAnglesToPack.setY(pitch);
      if (Double.isNaN(pitch))
      {
         eulerAnglesToPack.setToNaN();
      }
      else
      {
         eulerAnglesToPack.setZ(computeYawFromQuaternionImpl(qx, qy, qz, qs));
         eulerAnglesToPack.setX(computeRollFromQuaternionImpl(qx, qy, qz, qs));
      }
   }

   public static double computeYawFromAxisAngleImpl(double ux, double uy, double uz, double angle)
   {
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m10 = t * ux * uy + sinTheta * uz;
      double m00 = t * ux * ux + cosTheta;
      return computeYawImpl(m00, m10);
   }
   
   public static double computePitchFromAxisAngleImpl(double ux, double uy, double uz, double angle)
   {
      double m20 = (1.0 - Math.cos(angle)) * ux * uz - Math.sin(angle) * uy;
      return computePitchImpl(m20);
   }

   public static double computeRollFromAxisAngleImpl(double ux, double uy, double uz, double angle)
   {
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m21 = t * uy * uz + sinTheta * ux;
      double m22 = t * uz * uz + cosTheta;
      return computeRollImpl(m21, m22);
   }

   public static double computeYaw(AxisAngleReadOnly<?> axisAngle)
   {
      if (AxisAngleTools.containsNaN(axisAngle))
         return Double.NaN;

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);

      if (uNorm < EPS)
         return 0.0;

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      if (Double.isNaN(computePitchFromAxisAngleImpl(ux, uy, uz, angle)))
         return Double.NaN;
      else
         return computeYawFromAxisAngleImpl(ux, uy, uz, angle);
   }

   public static double computePitch(AxisAngleReadOnly<?> axisAngle)
   {
      if (AxisAngleTools.containsNaN(axisAngle))
         return Double.NaN;

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);

      if (uNorm < EPS)
         return 0.0;

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      return computePitchFromAxisAngleImpl(ux, uy, uz, angle);
   }

   public static double computeRoll(AxisAngleReadOnly<?> axisAngle)
   {
      if (AxisAngleTools.containsNaN(axisAngle))
         return Double.NaN;

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);

      if (uNorm < EPS)
         return 0.0;

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      if (Double.isNaN(computePitchFromAxisAngleImpl(ux, uy, uz, angle)))
         return Double.NaN;
      else
         return computeRollFromAxisAngleImpl(ux, uy, uz, angle);
   }

   public static void convertAxisAngleToYawPitchRoll(AxisAngleReadOnly<?> axisAngle, double[] yawPitchRollToPack)
   {
      if (AxisAngleTools.containsNaN(axisAngle))
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[1] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
         return;
      }

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();
      double uNorm = Math.sqrt(ux * ux + uy * uy + uz * uz);
      if (uNorm < EPS)
      {
         yawPitchRollToPack[0] = 0.0;
         yawPitchRollToPack[1] = 0.0;
         yawPitchRollToPack[2] = 0.0;
         return;
      }

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;
      convertAxisAngleToYawPitchRollImpl(ux, uy, uz, angle, yawPitchRollToPack);
   }

   public static void convertAxisAngleToYawPitchRollImpl(double ux, double uy, double uz, double angle, double[] yawPitchRollToPack)
   {
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m20 = t * ux * uz - sinTheta * uy;
      double pitch = computePitchImpl(m20);
      yawPitchRollToPack[1] = pitch;

      if (Double.isNaN(pitch))
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
      }
      else
      {
         double m10 = t * ux * uy + sinTheta * uz;
         double m00 = t * ux * ux + cosTheta;
         double m21 = t * uy * uz + sinTheta * ux;
         double m22 = t * uz * uz + cosTheta;
         
         yawPitchRollToPack[0] = computeYawImpl(m00, m10);
         yawPitchRollToPack[2] = computeRollImpl(m21, m22);
      }
   }

   public static void convertAxisAngleToYawPitchRoll(AxisAngleReadOnly<?> axisAngle, Tuple3DBasics eulerAnglesToPack)
   {
      if (AxisAngleTools.containsNaN(axisAngle))
      {
         eulerAnglesToPack.setToNaN();
         return;
      }

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();
      double uNorm = Math.sqrt(ux * ux + uy * uy + uz * uz);
      if (uNorm < EPS)
      {
         eulerAnglesToPack.setToZero();
         return;
      }

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;
      convertAxisAngleToYawPitchRollImpl(ux, uy, uz, angle, eulerAnglesToPack);
   }

   public static void convertAxisAngleToYawPitchRollImpl(double ux, double uy, double uz, double angle, Tuple3DBasics eulerAnglesToPack)
   {
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m20 = t * ux * uz - sinTheta * uy;
      double pitch = computePitchImpl(m20);
      eulerAnglesToPack.setY(pitch);

      if (Double.isNaN(pitch))
      {
         eulerAnglesToPack.setToNaN();
      }
      else
      {
         double m10 = t * ux * uy + sinTheta * uz;
         double m00 = t * ux * ux + cosTheta;
         double m21 = t * uy * uz + sinTheta * ux;
         double m22 = t * uz * uz + cosTheta;
         
         eulerAnglesToPack.setZ(computeYawImpl(m00, m10));
         eulerAnglesToPack.setX(computeRollImpl(m21, m22));
      }
   }

   public static double computeYaw(VectorReadOnly rotationVector)
   {
      if (TupleTools.containsNaN(rotationVector))
         return Double.NaN;

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);

      if (uNorm < EPS)
         return 0.0;

      angle = uNorm;
      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      if (Double.isNaN(computePitchFromAxisAngleImpl(ux, uy, uz, angle)))
         return Double.NaN;
      else
         return computeYawFromAxisAngleImpl(ux, uy, uz, angle);
   }

   public static double computePitch(VectorReadOnly rotationVector)
   {
      if (TupleTools.containsNaN(rotationVector))
         return Double.NaN;

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);
      angle = uNorm;

      if (uNorm < EPS)
         return 0.0;

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;
      return computePitchFromAxisAngleImpl(ux, uy, uz, angle);
   }

   public static double computeRoll(VectorReadOnly rotationVector)
   {
      if (TupleTools.containsNaN(rotationVector))
         return Double.NaN;

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);

      if (uNorm < EPS)
         return 0.0;

      angle = uNorm;
      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      if (Double.isNaN(computePitchFromAxisAngleImpl(ux, uy, uz, angle)))
         return Double.NaN;
      else
         return computeRollFromAxisAngleImpl(ux, uy, uz, angle);
   }

   public static void convertRotationVectorToYawPitchRoll(VectorReadOnly rotationVector, double[] yawPitchRollToPack)
   {
      if (TupleTools.containsNaN(rotationVector))
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[1] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
         return;
      }

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);

      if (uNorm < EPS)
      {
         yawPitchRollToPack[0] = 0.0;
         yawPitchRollToPack[1] = 0.0;
         yawPitchRollToPack[2] = 0.0;
         return;
      }

      angle = uNorm;
      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      convertAxisAngleToYawPitchRollImpl(ux, uy, uz, angle, yawPitchRollToPack);
   }

   public static void convertRotationVectorToYawPitchRoll(VectorReadOnly rotationVector, VectorBasics eulerAnglesToPack)
   {
      if (TupleTools.containsNaN(rotationVector))
      {
         eulerAnglesToPack.setToNaN();
         return;
      }

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = AxisAngleTools.axisNorm(ux, uy, uz, angle);

      if (uNorm < EPS)
      {
         eulerAnglesToPack.setToZero();
         return;
      }

      angle = uNorm;
      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      convertAxisAngleToYawPitchRollImpl(ux, uy, uz, angle, eulerAnglesToPack);
   }
}

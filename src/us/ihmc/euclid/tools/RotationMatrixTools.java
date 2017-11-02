package us.ihmc.euclid.tools;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public abstract class RotationMatrixTools
{
   /**
    * Performs the multiplication: {@code m1} * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiply(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, RotationMatrix matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM10() + m1.getM02() * m2.getM20();
      double m01 = m1.getM00() * m2.getM01() + m1.getM01() * m2.getM11() + m1.getM02() * m2.getM21();
      double m02 = m1.getM00() * m2.getM02() + m1.getM01() * m2.getM12() + m1.getM02() * m2.getM22();
      double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM10() + m1.getM12() * m2.getM20();
      double m11 = m1.getM10() * m2.getM01() + m1.getM11() * m2.getM11() + m1.getM12() * m2.getM21();
      double m12 = m1.getM10() * m2.getM02() + m1.getM11() * m2.getM12() + m1.getM12() * m2.getM22();
      double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM10() + m1.getM22() * m2.getM20();
      double m21 = m1.getM20() * m2.getM01() + m1.getM21() * m2.getM11() + m1.getM22() * m2.getM21();
      double m22 = m1.getM20() * m2.getM02() + m1.getM21() * m2.getM12() + m1.getM22() * m2.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>T</sup> * {@code m2}<sup>T</sup> and stores the
    * result in {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeBoth(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, RotationMatrix matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM10() * m2.getM01() + m1.getM20() * m2.getM02();
      double m01 = m1.getM00() * m2.getM10() + m1.getM10() * m2.getM11() + m1.getM20() * m2.getM12();
      double m02 = m1.getM00() * m2.getM20() + m1.getM10() * m2.getM21() + m1.getM20() * m2.getM22();
      double m10 = m1.getM01() * m2.getM00() + m1.getM11() * m2.getM01() + m1.getM21() * m2.getM02();
      double m11 = m1.getM01() * m2.getM10() + m1.getM11() * m2.getM11() + m1.getM21() * m2.getM12();
      double m12 = m1.getM01() * m2.getM20() + m1.getM11() * m2.getM21() + m1.getM21() * m2.getM22();
      double m20 = m1.getM02() * m2.getM00() + m1.getM12() * m2.getM01() + m1.getM22() * m2.getM02();
      double m21 = m1.getM02() * m2.getM10() + m1.getM12() * m2.getM11() + m1.getM22() * m2.getM12();
      double m22 = m1.getM02() * m2.getM20() + m1.getM12() * m2.getM21() + m1.getM22() * m2.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>T</sup> * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeLeft(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, RotationMatrix matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM10() * m2.getM10() + m1.getM20() * m2.getM20();
      double m01 = m1.getM00() * m2.getM01() + m1.getM10() * m2.getM11() + m1.getM20() * m2.getM21();
      double m02 = m1.getM00() * m2.getM02() + m1.getM10() * m2.getM12() + m1.getM20() * m2.getM22();
      double m10 = m1.getM01() * m2.getM00() + m1.getM11() * m2.getM10() + m1.getM21() * m2.getM20();
      double m11 = m1.getM01() * m2.getM01() + m1.getM11() * m2.getM11() + m1.getM21() * m2.getM21();
      double m12 = m1.getM01() * m2.getM02() + m1.getM11() * m2.getM12() + m1.getM21() * m2.getM22();
      double m20 = m1.getM02() * m2.getM00() + m1.getM12() * m2.getM10() + m1.getM22() * m2.getM20();
      double m21 = m1.getM02() * m2.getM01() + m1.getM12() * m2.getM11() + m1.getM22() * m2.getM21();
      double m22 = m1.getM02() * m2.getM02() + m1.getM12() * m2.getM12() + m1.getM22() * m2.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1} * {@code m2}<sup>T</sup> and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeRight(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, RotationMatrix matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM01() + m1.getM02() * m2.getM02();
      double m01 = m1.getM00() * m2.getM10() + m1.getM01() * m2.getM11() + m1.getM02() * m2.getM12();
      double m02 = m1.getM00() * m2.getM20() + m1.getM01() * m2.getM21() + m1.getM02() * m2.getM22();
      double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM01() + m1.getM12() * m2.getM02();
      double m11 = m1.getM10() * m2.getM10() + m1.getM11() * m2.getM11() + m1.getM12() * m2.getM12();
      double m12 = m1.getM10() * m2.getM20() + m1.getM11() * m2.getM21() + m1.getM12() * m2.getM22();
      double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM01() + m1.getM22() * m2.getM02();
      double m21 = m1.getM20() * m2.getM10() + m1.getM21() * m2.getM11() + m1.getM22() * m2.getM12();
      double m22 = m1.getM20() * m2.getM20() + m1.getM21() * m2.getM21() + m1.getM22() * m2.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Prepend a rotation about the z-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    * 
    * <pre>
    *                / cos(yaw) -sin(yaw) 0 \              
    * matrixToPack = | sin(yaw)  cos(yaw) 0 | * matrixOriginal
    *                \    0         0     1 /
    * </pre>
    * 
    * @param yaw the angle to rotate about the z-axis.
    * @param matrixOriginal the matrix on which the yaw rotation is appended. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void prependYawRotation(double yaw, RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixToPack)
   {
      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);

      double m00 = cYaw * matrixOriginal.getM00() - sYaw * matrixOriginal.getM10();
      double m01 = cYaw * matrixOriginal.getM01() - sYaw * matrixOriginal.getM11();
      double m02 = cYaw * matrixOriginal.getM02() - sYaw * matrixOriginal.getM12();
      double m10 = sYaw * matrixOriginal.getM00() + cYaw * matrixOriginal.getM10();
      double m11 = sYaw * matrixOriginal.getM01() + cYaw * matrixOriginal.getM11();
      double m12 = sYaw * matrixOriginal.getM02() + cYaw * matrixOriginal.getM12();
      double m20 = matrixOriginal.getM20();
      double m21 = matrixOriginal.getM21();
      double m22 = matrixOriginal.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Append a rotation about the z-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    * 
    * <pre>
    *                                 / cos(yaw) -sin(yaw) 0 \
    * matrixToPack = matrixOriginal * | sin(yaw)  cos(yaw) 0 |
    *                                 \    0         0     1 /
    * </pre>
    * 
    * @param matrixOriginal the matrix on which the yaw rotation is appended. Not modified.
    * @param yaw the angle to rotate about the z-axis.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void appendYawRotation(RotationMatrixReadOnly matrixOriginal, double yaw, RotationMatrix matrixToPack)
   {
      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);

      double m00 = cYaw * matrixOriginal.getM00() + sYaw * matrixOriginal.getM01();
      double m01 = -sYaw * matrixOriginal.getM00() + cYaw * matrixOriginal.getM01();
      double m02 = matrixOriginal.getM02();
      double m10 = cYaw * matrixOriginal.getM10() + sYaw * matrixOriginal.getM11();
      double m11 = -sYaw * matrixOriginal.getM10() + cYaw * matrixOriginal.getM11();
      double m12 = matrixOriginal.getM12();
      double m20 = cYaw * matrixOriginal.getM20() + sYaw * matrixOriginal.getM21();
      double m21 = -sYaw * matrixOriginal.getM20() + cYaw * matrixOriginal.getM21();
      double m22 = matrixOriginal.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Prepend a rotation about the y-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    * 
    * <pre>
    *                /  cos(pitch) 0 sin(pitch) \              
    * matrixToPack = |      0      1     0      | * matrixOriginal
    *                \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * 
    * @param pitch the angle to rotate about the y-axis.
    * @param matrixOriginal the matrix on which the pitch rotation is appended. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void prependPitchRotation(double pitch, RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixToPack)
   {
      double cPitch = Math.cos(pitch);
      double sPitch = Math.sin(pitch);

      double m00 = cPitch * matrixOriginal.getM00() + sPitch * matrixOriginal.getM20();
      double m01 = cPitch * matrixOriginal.getM01() + sPitch * matrixOriginal.getM21();
      double m02 = cPitch * matrixOriginal.getM02() + sPitch * matrixOriginal.getM22();
      double m10 = matrixOriginal.getM10();
      double m11 = matrixOriginal.getM11();
      double m12 = matrixOriginal.getM12();
      double m20 = -sPitch * matrixOriginal.getM00() + cPitch * matrixOriginal.getM20();
      double m21 = -sPitch * matrixOriginal.getM01() + cPitch * matrixOriginal.getM21();
      double m22 = -sPitch * matrixOriginal.getM02() + cPitch * matrixOriginal.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Append a rotation about the y-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    * 
    * <pre>
    *                                 /  cos(pitch) 0 sin(pitch) \
    * matrixToPack = matrixOriginal * |      0      1     0      |
    *                                 \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * 
    * @param matrixOriginal the matrix on which the pitch rotation is appended. Not modified.
    * @param pitch the angle to rotate about the y-axis.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void appendPitchRotation(RotationMatrixReadOnly matrixOriginal, double pitch, RotationMatrix matrixToPack)
   {
      double cPitch = Math.cos(pitch);
      double sPitch = Math.sin(pitch);

      double m00 = cPitch * matrixOriginal.getM00() - sPitch * matrixOriginal.getM02();
      double m01 = matrixOriginal.getM01();
      double m02 = sPitch * matrixOriginal.getM00() + cPitch * matrixOriginal.getM02();
      double m10 = cPitch * matrixOriginal.getM10() - sPitch * matrixOriginal.getM12();
      double m11 = matrixOriginal.getM11();
      double m12 = sPitch * matrixOriginal.getM10() + cPitch * matrixOriginal.getM12();
      double m20 = cPitch * matrixOriginal.getM20() - sPitch * matrixOriginal.getM22();
      double m21 = matrixOriginal.getM21();
      double m22 = sPitch * matrixOriginal.getM20() + cPitch * matrixOriginal.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Prepend a rotation about the x-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    * 
    * <pre>
    *                / 1     0          0     \               
    * matrixToPack = | 0 cos(roll) -sin(roll) | * matrixOriginal
    *                \ 0 sin(roll)  cos(roll) /
    * </pre>
    * 
    * @param roll the angle to rotate about the x-axis.
    * @param matrixOriginal the matrix on which the roll rotation is appended. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void prependRollRotation(double roll, RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixToPack)
   {
      double cRoll = Math.cos(roll);
      double sRoll = Math.sin(roll);

      double m00 = matrixOriginal.getM00();
      double m01 = matrixOriginal.getM01();
      double m02 = matrixOriginal.getM02();
      double m10 = cRoll * matrixOriginal.getM10() - sRoll * matrixOriginal.getM20();
      double m11 = cRoll * matrixOriginal.getM11() - sRoll * matrixOriginal.getM21();
      double m12 = cRoll * matrixOriginal.getM12() - sRoll * matrixOriginal.getM22();
      double m20 = sRoll * matrixOriginal.getM10() + cRoll * matrixOriginal.getM20();
      double m21 = sRoll * matrixOriginal.getM11() + cRoll * matrixOriginal.getM21();
      double m22 = sRoll * matrixOriginal.getM12() + cRoll * matrixOriginal.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Append a rotation about the x-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    * 
    * <pre>
    *                                 / 1     0          0     \
    * matrixToPack = matrixOriginal * | 0 cos(roll) -sin(roll) |
    *                                 \ 0 sin(roll)  cos(roll) /
    * </pre>
    * 
    * @param matrixOriginal the matrix on which the roll rotation is appended. Not modified.
    * @param roll the angle to rotate about the x-axis.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void appendRollRotation(RotationMatrixReadOnly matrixOriginal, double roll, RotationMatrix matrixToPack)
   {
      double cRoll = Math.cos(roll);
      double sRoll = Math.sin(roll);

      double m00 = matrixOriginal.getM00();
      double m01 = cRoll * matrixOriginal.getM01() + sRoll * matrixOriginal.getM02();
      double m02 = -sRoll * matrixOriginal.getM01() + cRoll * matrixOriginal.getM02();
      double m10 = matrixOriginal.getM10();
      double m11 = cRoll * matrixOriginal.getM11() + sRoll * matrixOriginal.getM12();
      double m12 = -sRoll * matrixOriginal.getM11() + cRoll * matrixOriginal.getM12();
      double m20 = matrixOriginal.getM20();
      double m21 = cRoll * matrixOriginal.getM21() + sRoll * matrixOriginal.getM22();
      double m22 = -sRoll * matrixOriginal.getM21() + cRoll * matrixOriginal.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Rotates the given {@code tupleOriginal} by a rotation about the z-axis and stores the result
    * in {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in-place transformation.
    * </p>
    * 
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleTransformed = | sin(yaw)  cos(yaw) 0 | * tupleOriginal
    *                    \    0         0     1 /
    * </pre>
    * 
    * @param yaw the angle to rotate about the z-axis.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void applyYawRotation(double yaw, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);

      double x = tupleOriginal.getX() * cYaw - tupleOriginal.getY() * sYaw;
      double y = tupleOriginal.getX() * sYaw + tupleOriginal.getY() * cYaw;
      double z = tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   /**
    * Rotates the given {@code tupleOriginal} by a rotation about the z-axis and stores the result
    * in {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in-place transformation.
    * </p>
    * 
    * <pre>
    * tupleTransformed = / cos(yaw) -sin(yaw) \ * tupleOriginal
    *                    \ sin(yaw)  cos(yaw) /
    * </pre>
    * 
    * @param yaw the angle to rotate about the z-axis.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void applyYawRotation(double yaw, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);

      double x = tupleOriginal.getX() * cYaw - tupleOriginal.getY() * sYaw;
      double y = tupleOriginal.getX() * sYaw + tupleOriginal.getY() * cYaw;
      tupleTransformed.set(x, y);
   }

   /**
    * Rotates the given {@code tupleOriginal} by a rotation about the y-axis and stores the result
    * in {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in-place transformation.
    * </p>
    * 
    * <pre>
    *                    /  cos(pitch) 0 sin(pitch) \
    * tupleTransformed = |      0      1     0      | * tupleOriginal
    *                    \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * 
    * @param pitch the angle to rotate about the y-axis.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void applyPitchRotation(double pitch, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double cPitch = Math.cos(pitch);
      double sPitch = Math.sin(pitch);

      double x = tupleOriginal.getX() * cPitch + tupleOriginal.getZ() * sPitch;
      double y = tupleOriginal.getY();
      double z = -tupleOriginal.getX() * sPitch + tupleOriginal.getZ() * cPitch;
      tupleTransformed.set(x, y, z);
   }

   /**
    * Rotates the given {@code tupleOriginal} by a rotation about the x-axis and stores the result
    * in {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in-place transformation.
    * </p>
    * 
    * <pre>
    *                    / 1     0          0     \
    * tupleTransformed = | 0 cos(roll) -sin(roll) | * tupleOriginal
    *                    \ 0 sin(roll)  cos(roll) /
    * </pre>
    * 
    * @param roll the angle to rotate about the x-axis.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void applyRollRotation(double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double cRoll = Math.cos(roll);
      double sRoll = Math.sin(roll);

      double x = tupleOriginal.getX();
      double y = tupleOriginal.getY() * cRoll - tupleOriginal.getZ() * sRoll;
      double z = tupleOriginal.getY() * sRoll + tupleOriginal.getZ() * cRoll;
      tupleTransformed.set(x, y, z);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code r0} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions, see
    * {@link QuaternionBasics#interpolate(QuaternionReadOnly, QuaternionReadOnly, double)}.
    * </p>
    * 
    * @param r0 the first rotation matrix used in the interpolation. Not modified.
    * @param rf the second rotation matrix used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code matrixToPack} to {@code r0}, while a value of 1 is equivalent to setting
    *           {@code matrixToPack} to {@code rf}.
    * @param matrixToPack the rotation matrix in which the result of the interpolation is stored.
    *           Modified.
    */
   public static void interpolate(RotationMatrixReadOnly r0, RotationMatrixReadOnly rf, double alpha, RotationMatrix matrixToPack)
   {
      if (r0.containsNaN() || rf.containsNaN())
      {
         matrixToPack.setToNaN();
         return;
      }

      if (r0.epsilonEquals(rf, AxisAngleConversion.EPS))
      {
         matrixToPack.set(r0);
         return;
      }

      double m00 = r0.getM00() * rf.getM00() + r0.getM10() * rf.getM10() + r0.getM20() * rf.getM20();
      double m01 = r0.getM00() * rf.getM01() + r0.getM10() * rf.getM11() + r0.getM20() * rf.getM21();
      double m02 = r0.getM00() * rf.getM02() + r0.getM10() * rf.getM12() + r0.getM20() * rf.getM22();
      double m10 = r0.getM01() * rf.getM00() + r0.getM11() * rf.getM10() + r0.getM21() * rf.getM20();
      double m11 = r0.getM01() * rf.getM01() + r0.getM11() * rf.getM11() + r0.getM21() * rf.getM21();
      double m12 = r0.getM01() * rf.getM02() + r0.getM11() * rf.getM12() + r0.getM21() * rf.getM22();
      double m20 = r0.getM02() * rf.getM00() + r0.getM12() * rf.getM10() + r0.getM22() * rf.getM20();
      double m21 = r0.getM02() * rf.getM01() + r0.getM12() * rf.getM11() + r0.getM22() * rf.getM21();
      double m22 = r0.getM02() * rf.getM02() + r0.getM12() * rf.getM12() + r0.getM22() * rf.getM22();

      double angle, x, y, z; // variables for result

      x = m21 - m12;
      y = m02 - m20;
      z = m10 - m01;

      double s = Math.sqrt(x * x + y * y + z * z);

      if (s > AxisAngleConversion.EPS)
      {
         double sin = 0.5 * s;
         double cos = 0.5 * (m00 + m11 + m22 - 1.0);
         angle = Math.atan2(sin, cos);
         x /= s;
         y /= s;
         z /= s;
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

      angle *= alpha;

      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;

      double xz = x * z;
      double xy = x * y;
      double yz = y * z;

      m00 = t * x * x + cosTheta;
      m01 = t * xy - sinTheta * z;
      m02 = t * xz + sinTheta * y;
      m10 = t * xy + sinTheta * z;
      m11 = t * y * y + cosTheta;
      m12 = t * yz - sinTheta * x;
      m20 = t * xz - sinTheta * y;
      m21 = t * yz + sinTheta * x;
      m22 = t * z * z + cosTheta;

      double r00 = r0.getM00() * m00 + r0.getM01() * m10 + r0.getM02() * m20;
      double r01 = r0.getM00() * m01 + r0.getM01() * m11 + r0.getM02() * m21;
      double r02 = r0.getM00() * m02 + r0.getM01() * m12 + r0.getM02() * m22;
      double r10 = r0.getM10() * m00 + r0.getM11() * m10 + r0.getM12() * m20;
      double r11 = r0.getM10() * m01 + r0.getM11() * m11 + r0.getM12() * m21;
      double r12 = r0.getM10() * m02 + r0.getM11() * m12 + r0.getM12() * m22;
      double r20 = r0.getM20() * m00 + r0.getM21() * m10 + r0.getM22() * m20;
      double r21 = r0.getM20() * m01 + r0.getM21() * m11 + r0.getM22() * m21;
      double r22 = r0.getM20() * m02 + r0.getM21() * m12 + r0.getM22() * m22;

      matrixToPack.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);
   }

   /**
    * Computes and returns the distance from the rotation matrix {@code m1} to {@code m2}.
    * 
    * @param m1 the first rotation matrix. Not modified.
    * @param m2 the second rotation matrix. Not modified.
    * @return the angle representing the distance between the two rotation matrices. It is contained
    *         in [0, <i>pi</i>].
    */
   public static double distance(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM01() + m1.getM02() * m2.getM02();
      double m01 = m1.getM00() * m2.getM10() + m1.getM01() * m2.getM11() + m1.getM02() * m2.getM12();
      double m02 = m1.getM00() * m2.getM20() + m1.getM01() * m2.getM21() + m1.getM02() * m2.getM22();
      double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM01() + m1.getM12() * m2.getM02();
      double m11 = m1.getM10() * m2.getM10() + m1.getM11() * m2.getM11() + m1.getM12() * m2.getM12();
      double m12 = m1.getM10() * m2.getM20() + m1.getM11() * m2.getM21() + m1.getM12() * m2.getM22();
      double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM01() + m1.getM22() * m2.getM02();
      double m21 = m1.getM20() * m2.getM10() + m1.getM21() * m2.getM11() + m1.getM22() * m2.getM12();
      double m22 = m1.getM20() * m2.getM20() + m1.getM21() * m2.getM21() + m1.getM22() * m2.getM22();

      double angle, x, y, z; // variables for result

      x = m21 - m12;
      y = m02 - m20;
      z = m10 - m01;

      double s = EuclidCoreTools.norm(x, y, z);

      if (s > AxisAngleConversion.EPS)
      {
         double sin = 0.5 * s;
         double cos = 0.5 * (m00 + m11 + m22 - 1.0);
         angle = Math.atan2(sin, cos);
      }
      else if (m00 + m11 + m22 > 3.0 - 1.0e-7)
      { // At this point, the matrix has to be identity.
         return 0.0;
      }
      else
      {
         // otherwise this singularity is angle = 180
         angle = Math.PI;
      }

      return angle;
   }
}

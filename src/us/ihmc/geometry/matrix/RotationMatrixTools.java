package us.ihmc.geometry.matrix;

import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;

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
}

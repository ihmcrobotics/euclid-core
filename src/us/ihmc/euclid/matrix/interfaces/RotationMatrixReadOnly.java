package us.ihmc.euclid.matrix.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read interface used for 3-by-3 rotation matrices.
 * <p>
 * A rotation matrix is used to represent a 3D orientation through its 9 coefficients. A rotation
 * matrix has to comply to several constraints:
 * <ul>
 * <li>each column of the matrix represents a unitary vector,
 * <li>each row of the matrix represents a unitary vector,
 * <li>every pair of columns of the matrix represents two orthogonal vectors,
 * <li>every pair of rows of the matrix represents two orthogonal vectors,
 * <li>the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface RotationMatrixReadOnly extends Matrix3DReadOnly, Orientation3DReadOnly
{
   /**
    * Computes and returns the distance between this rotation matrix and the {@code other}.
    *
    * @param other the other rotation matrix to compute the distance. Not modified.
    * @return the angle representing the distance between the two rotation matrices. It is contained
    *         in [0, <i>pi</i>].
    */
   default double distance(RotationMatrixReadOnly other)
   {
      return RotationMatrixTools.distance(this, other);
   }

   /**
    * Computes and packs the orientation described by this rotation matrix as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector representing the same orientation as this.
    *           Modified.
    * @deprecated Use {@link #getRotationVector(Vector3DBasics)} instead
    */
   default void get(Vector3DBasics rotationVectorToPack)
   {
      getRotationVector(rotationVectorToPack);
   }

   @Override
   default void get(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(this);
   }

   @Override
   default void get(AxisAngleBasics axisAngleToPack)
   {
      AxisAngleConversion.convertMatrixToAxisAngle(this, axisAngleToPack);
   }

   @Override
   default void get(QuaternionBasics quaternionToPack)
   {
      QuaternionConversion.convertMatrixToQuaternion(this, quaternionToPack);
   }

   /**
    * Computes and packs the orientation described by this rotation matrix as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector representing the same orientation as this.
    *           Modified.
    */
   default void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertMatrixToRotationVector(this, rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this rotation matrix as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   default void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, eulerAnglesToPack);
   }

   /**
    * Computes and packs the orientation described by this rotation matrix as the yaw-pitch-roll
    * angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   default void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, yawPitchRollToPack);
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of this rotation
    * matrix.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   default double getYaw()
   {
      return YawPitchRollConversion.computeYaw(this);
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of this rotation
    * matrix.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   default double getPitch()
   {
      return YawPitchRollConversion.computePitch(this);
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of this rotation
    * matrix.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   default double getRoll()
   {
      return YawPitchRollConversion.computeRoll(this);
   }

   @Override
   default void addTransform(Tuple3DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.addTransform(tupleToTransform);
   }

   @Override
   default void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(Tuple3DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.transform(tupleToTransform);
   }

   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(Tuple2DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.transform(tupleToTransform);
   }

   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      Matrix3DReadOnly.super.transform(tupleToTransform, checkIfTransformInXYPlane);
   }

   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   @Override
   default void transform(Vector4DBasics vectorToTransform)
   {
      Matrix3DReadOnly.super.transform(vectorToTransform);
   }

   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given quaternion by this rotation matrix.
    * <p>
    * quaternionToTransform = Q(this) * quaternionToTransform <br>
    * where Q(this) is the equivalent quaternion for this rotation matrix.
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Modified.
    */
   default void transform(QuaternionBasics quaternionToTransform)
   {
      transform(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Transforms the given quaternion {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = Q(this) * quaternionOriginal <br>
    * where Q(this) is the equivalent quaternion for this rotation matrix.
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   default void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      QuaternionTools.multiply(this, false, quaternionOriginal, false, quaternionTransformed);
   }

   @Override
   default void transform(Matrix3D matrixToTransform)
   {
      Matrix3DReadOnly.super.transform(matrixToTransform);
   }

   /**
    * Transforms the given rotation matrix by this rotation matrix.
    * <p>
    * matrixToTransform = this * matrixToTransform
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given rotation matrix {@code matrixOriginal} by this rotation matrix and stores
    * the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this * matrixOriginal
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      RotationMatrixTools.multiply(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given rotation matrix by this rotation matrix.
    * <p>
    * matrixToTransform.rotationMatrix = this * matrixToTransform.rotationMatrix
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transform(RotationScaleMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the rotation part of the given rotation-scale matrix {@code matrixOriginal} by this
    * rotation matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed.scales = matrixOriginal.scales <br>
    * matrixTransformed.rotationMatrix = this * matrixOriginal.rotationMatrix
    * </p>
    *
    * @param matrixOriginal the rotation-scale matrix to transform. Not modified.
    * @param matrixTransformed the rotation-scale matrix in which the result is stored. Modified.
    */
   default void transform(RotationScaleMatrixReadOnly matrixOriginal, RotationScaleMatrix matrixTransformed)
   {
      matrixTransformed.set(matrixOriginal);
      matrixTransformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      Matrix3DTools.multiply(this, matrixOriginal, matrixTransformed);
      Matrix3DTools.multiplyTransposeRight(matrixTransformed, this, matrixTransformed);
   }

   @Override
   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.inverseTransform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = getM00() * tupleOriginal.getX() + getM10() * tupleOriginal.getY() + getM20() * tupleOriginal.getZ();
      double y = getM01() * tupleOriginal.getX() + getM11() * tupleOriginal.getY() + getM21() * tupleOriginal.getZ();
      double z = getM02() * tupleOriginal.getX() + getM12() * tupleOriginal.getY() + getM22() * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   @Override
   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.inverseTransform(tupleToTransform);
   }

   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      Matrix3DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      Matrix3DReadOnly.super.inverseTransform(tupleToTransform, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      if (checkIfTransformInXYPlane)
         checkIfMatrix2D();

      double x = getM00() * tupleOriginal.getX() + getM10() * tupleOriginal.getY();
      double y = getM01() * tupleOriginal.getX() + getM11() * tupleOriginal.getY();
      tupleTransformed.set(x, y);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} and
    * stores the result into {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = Q(this<sup>-1</sup>) * quaternionOriginal <br>
    * where Q(this<sup>-1</sup>) is the equivalent quaternion for the inverse of this rotation
    * matrix.
    * </p>
    * <p>
    * This operation uses the property: <br>
    * q<sup>-1</sup> = conjugate(q) </br>
    * of a quaternion preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   default void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      QuaternionTools.multiply(this, true, quaternionOriginal, false, quaternionTransformed);
   }

   @Override
   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      Matrix3DReadOnly.super.inverseTransform(vectorToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      double x = getM00() * vectorOriginal.getX() + getM10() * vectorOriginal.getY() + getM20() * vectorOriginal.getZ();
      double y = getM01() * vectorOriginal.getX() + getM11() * vectorOriginal.getY() + getM21() * vectorOriginal.getZ();
      double z = getM02() * vectorOriginal.getX() + getM12() * vectorOriginal.getY() + getM22() * vectorOriginal.getZ();
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }

   /**
    * Performs the inverse of the transform to the given rotation matrix {@code matrixOriginal} by
    * this rotation matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this<sup>-1</sup> * matrixOriginal
    * </p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      RotationMatrixTools.multiplyTransposeLeft(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the rotation part of the given rotation-scale matrix
    * {@code matrixOriginal} by this rotation matrix and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * matrixTransformed.scales = matrixOriginal.scales<br>
    * matrixTransformed.rotationMatrix = this<sup>-1</sup> * matrixOriginal.rotationMatrix
    * </p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param matrixOriginal the rotation-scale matrix to transform. Not modified.
    * @param matrixTransformed the rotation-scale matrix in which the result is stored. Modified.
    */
   default void inverseTransform(RotationScaleMatrixReadOnly matrixOriginal, RotationScaleMatrix matrixTransformed)
   {
      matrixTransformed.set(matrixOriginal);
      matrixTransformed.preMultiplyTransposeOther(this);
   }

   @Override
   default void inverseTransform(Matrix3D matrixToTransform)
   {
      Matrix3DReadOnly.super.inverseTransform(matrixToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      Matrix3DTools.multiplyTransposeLeft(this, matrixOriginal, matrixTransformed);
      Matrix3DTools.multiply(matrixTransformed, this, matrixTransformed);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two rotation matrices are considered geometrically equal if the magnitude of their difference
    * is less than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other rotation matrix to compare against this. Not modified.
    * @param epsilon the maximum angle between the two rotation matrices to be considered equal.
    * @return {@code true} if the two rotation matrices represent the same geometry, {@code false}
    *         otherwise.
    */
   default boolean geometricallyEquals(RotationMatrixReadOnly other, double epsilon)
   {
      return distance(other) <= epsilon;
   }

   /**
    * Provides a {@code String} representation of this rotation matrix converted to yaw-pitch-roll
    * angles as follows: yaw-pitch-roll: (yaw, pitch, roll).
    *
    * @return
    */
   default String toStringAsYawPitchRoll()
   {
      return EuclidCoreIOTools.getStringOf("yaw-pitch-roll: (", ")", ", ", getYaw(), getPitch(), getRoll());
   }
}

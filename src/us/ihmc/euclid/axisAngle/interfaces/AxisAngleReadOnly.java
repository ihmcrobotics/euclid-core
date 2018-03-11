package us.ihmc.euclid.axisAngle.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.AxisAngleTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
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
 * Read-only interface for an axis-angle object.
 * <p>
 * An axis-angle is used to represent a 3D orientation by a unitary axis of components (x, y, z) and
 * an angle of rotation usually expressed in radians.
 * </p>
 *
 * @author Sylvain
 * @param T the final type of the axis-angle used.
 */
public interface AxisAngleReadOnly extends Orientation3DReadOnly
{
   /**
    * Returns the angle of this axis-angle, usually expressed in radians.
    *
    * @return the angle.
    */
   double getAngle();

   /**
    * Returns the angle of this axis-angle, usually expressed in radians.
    *
    * @return the angle.
    */
   default float getAngle32()
   {
      return (float) getAngle();
   }

   /**
    * Returns the x-component of the unitary axis of this axis-angle.
    *
    * @return the x-component of the unitary axis.
    */
   double getX();

   /**
    * Returns the x-component of the unitary axis of this axis-angle.
    *
    * @return the x-component of the unitary axis.
    */
   default float getX32()
   {
      return (float) getX();
   }

   /**
    * Returns the y-component of the unitary axis of this axis-angle.
    *
    * @return the y-component of the unitary axis.
    */
   double getY();

   /**
    * Returns the y-component of the unitary axis of this axis-angle.
    *
    * @return the y-component of the unitary axis.
    */
   default float getY32()
   {
      return (float) getY();
   }

   /**
    * Returns the z-component of the unitary axis of this axis-angle.
    *
    * @return the z-component of the unitary axis.
    */
   double getZ();

   /**
    * Returns the z-component of the unitary axis of this axis-angle.
    *
    * @return the z-component of the unitary axis.
    */
   default float getZ32()
   {
      return (float) getZ();
   }

   /**
    * Tests if this axis-angle contains a {@link Double#NaN}.
    *
    * @return {@code true} if this axis-angle contains a {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   default boolean containsNaN()
   {
      return EuclidCoreTools.containsNaN(getX(), getY(), getZ(), getAngle());
   }

   /**
    * Calculates and returns the norm of the axis of this axis-angle.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>)
    * </p>
    *
    * @return the norm's value of the axis.
    */
   default double axisNorm()
   {
      return EuclidCoreTools.norm(getX(), getY(), getZ());
   }

   /**
    * Tests if the axis of this axis-angle is of unit-length.
    *
    * @param epsilon tolerance to use in this test.
    * @return {@code true} if the axis is unitary, {@code false} otherwise.
    */
   default boolean isAxisUnitary(double epsilon)
   {
      return Math.abs(1.0 - axisNorm()) < epsilon;
   }

   /**
    * Tests if this axis-angle represents a rotation around the z-axis.
    * <p>
    * This is commonly used to test if the axis-angle can be used to transform 2D geometry object.
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @return {@code true} if this axis-angle represents a rotation around the z-axis, {@code false}
    *         otherwise.
    */
   default boolean isZOnly(double epsilon)
   {
      return Math.abs(getAngle()) < epsilon || Math.abs(getX()) < epsilon && Math.abs(getY()) < epsilon;
   }

   /**
    * Asserts that this axis-angle represents a rotation around the z-axis.
    * <p>
    * This is commonly used to test if the axis-angle can be used to transform 2D geometry object.
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @throws NotAMatrix2DException if this axis-angle does not represent a rotation around the
    *            z-axis.
    */
   default void checkIfIsZOnly(double epsilon)
   {
      if (!isZOnly(epsilon))
         throw new NotAMatrix2DException("The axis-angle is not in XY plane: " + toString());
   }

   /**
    * Computes and returns the distance from this axis-angle to {@code other}.
    *
    * @param other the other axis-angle to measure the distance. Not modified.
    * @return the angle representing the distance between the two axis-angles. It is contained in
    *         [0, 2<i>pi</i>]
    */
   default double distance(AxisAngleReadOnly other)
   {
      return AxisAngleTools.distance(this, other);
   }

   @Override
   default void get(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.setAxisAngle(getX(), getY(), getZ(), getAngle());
   }

   @Override
   default void get(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.setAxisAngle(getX(), getY(), getZ(), getAngle());
   }

   @Override
   default void get(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.setAxisAngle(getX(), getY(), getZ(), getAngle());
   }

   /**
    * Converts and gets the orientation represented by this axis-angle as a rotation vector. See
    * {@link RotationVectorConversion#convertAxisAngleToRotationVector(AxisAngleReadOnly, Vector3DBasics)}.
    *
    * @param rotationVectorToPack rotation vector in which the orientation of this axis-angle is
    *           stored. Modified.
    */
   @Override
   default void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertAxisAngleToRotationVector(this, rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this axis-angle as the yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   @Override
   default void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(this, yawPitchRollToPack);
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of this axis-angle.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   @Override
   default double getYaw()
   {
      return YawPitchRollConversion.computeYaw(this);
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of this
    * axis-angle.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   @Override
   default double getPitch()
   {
      return YawPitchRollConversion.computePitch(this);
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of this axis-angle.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   @Override
   default double getRoll()
   {
      return YawPitchRollConversion.computeRoll(this);
   }

   /**
    * Packs the components of this axis-angle in an array starting from its first index. The
    * components are packed in the following order: x, y, z, and angle.
    *
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(double[] axisAngleArrayToPack)
   {
      get(0, axisAngleArrayToPack);
   }

   /**
    * Packs the components of this axis-angle in an array starting from {@code startIndex}. The
    * components are packed in the following order: x, y, z, and angle.
    *
    * @param startIndex the index in the array where the first component is stored.
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(int startIndex, double[] axisAngleArrayToPack)
   {
      axisAngleArrayToPack[startIndex++] = getX();
      axisAngleArrayToPack[startIndex++] = getY();
      axisAngleArrayToPack[startIndex++] = getZ();
      axisAngleArrayToPack[startIndex] = getAngle();
   }

   /**
    * Packs the components of this axis-angle in an array starting from its first index. The
    * components are packed in the following order: x, y, z, and angle.
    *
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(float[] axisAngleArrayToPack)
   {
      get(0, axisAngleArrayToPack);
   }

   /**
    * Packs the components of this axis-angle in an array starting from {@code startIndex}. The
    * components are packed in the following order: x, y, z, and angle.
    *
    * @param startIndex the index in the array where the first component is stored.
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(int startIndex, float[] axisAngleArrayToPack)
   {
      axisAngleArrayToPack[startIndex++] = getX32();
      axisAngleArrayToPack[startIndex++] = getY32();
      axisAngleArrayToPack[startIndex++] = getZ32();
      axisAngleArrayToPack[startIndex] = getAngle32();
   }

   /**
    * Selects a component of this axis-angle based on {@code index} and returns its value.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components are x, y, and z,
    * respectively, while 3 corresponds to the angle.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
   default double getElement(int index)
   {
      switch (index)
      {
      case 0:
         return getX();
      case 1:
         return getY();
      case 2:
         return getZ();
      case 3:
         return getAngle();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Selects a component of this axis-angle based on {@code index} and returns its value.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components are x, y, and z,
    * respectively, while 3 corresponds to the angle.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
   default float getElement32(int index)
   {
      switch (index)
      {
      case 0:
         return getX32();
      case 1:
         return getY32();
      case 2:
         return getZ32();
      case 3:
         return getAngle32();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this axis-angle and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      AxisAngleTools.transform(this, tupleOriginal, tupleTransformed);
   }

   @Override
   default void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      AxisAngleTools.addTransform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this matrix
    *            does not represent a transformation in the XY plane.
    */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      AxisAngleTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given 3D matrix {@code matrixOriginal} by this axis-angle and stores the result
    * in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = R(this) * matrixOriginal * R(this)<sup>-1</sup> <br>
    * where R(axisAngle) is the function that converts an axis-angle into a rotation matrix.
    * </p>
    *
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      AxisAngleTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given quaternion {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = Q(this) * quaternionOriginal <br>
    * where Q(axisAngle) is the function that converts an axis-angle into a quaternion.
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   @Override
   default void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      AxisAngleTools.transform(this, quaternionOriginal, quaternionTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      AxisAngleTools.transform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given rotation matrix {@code matrixOriginal} by this quaternion and stores the
    * result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = R(this) * matrixOriginal <br>
    * where R(axisAngle) is the function that converts an axis-angle to a rotation matrix.
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   @Override
   default void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      AxisAngleTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this axis-angle.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      AxisAngleTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * axis-angle and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this axis-angle
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this axis-angle
    *            does not represent a transformation in the XY plane.
    */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      AxisAngleTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} and
    * stores the result into {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal <br>
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   @Override
   default void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      AxisAngleTools.inverseTransform(this, quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this axis-angle and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz * this
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      AxisAngleTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * axis-angle and stores the result in {@code matrixTransformed}.
    * <p>
    * s matrixTransformed = this<sup>-1</sup> * matrixOriginal * this
    * </p>
    *
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      AxisAngleTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the given rotation matrix {@code matrixOriginal} by
    * this axis-angle and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this<sup>-1</sup> * matrixOriginal
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   @Override
   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      AxisAngleTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Tests on a per component basis, if this axis-angle is exactly equal to {@code other}. A
    * failing test does not necessarily mean that the two axis-angles represent two different
    * orientations.
    *
    * @param other the other axis-angle to compare against this. Not modified.
    * @return {@code true} if the two axis-angles are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(AxisAngleReadOnly other)
   {
      try
      {
         return getX() == other.getX() && getY() == other.getY() && getZ() == other.getZ() && getAngle() == other.getAngle();
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis, if this axis-angle is equal to {@code other} to an
    * {@code epsilon}. A failing test does not necessarily mean that the two axis-angles represent
    * two different orientations.
    *
    * @param other the other axis-angle to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two axis-angle are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(AxisAngleReadOnly other, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(getX(), other.getX(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(getY(), other.getY(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(getZ(), other.getZ(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(getAngle(), other.getAngle(), epsilon))
         return false;

      return true;
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two axis-angle are considered geometrically equal if the magnitude of their difference is less
    * than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other axis-angle to compare against this. Not modified.
    * @param epsilon the maximum angle for the two quaternions to be considered equal.
    * @return {@code true} if the two axis-angle represent the same geometry, {@code false}
    *         otherwise.
    */
   default boolean geometricallyEquals(AxisAngleReadOnly other, double epsilon)
   {
      return Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(distance(other))) <= epsilon;
   }

   /**
    * Provides a {@code String} representation of this axis-angle converted to yaw-pitch-roll angles
    * as follows: yaw-pitch-roll: (yaw, pitch, roll).
    *
    * @return
    */
   default String toStringAsYawPitchRoll()
   {
      return EuclidCoreIOTools.getStringOf("yaw-pitch-roll: (", ")", ", ", getYaw(), getPitch(), getRoll());
   }
}
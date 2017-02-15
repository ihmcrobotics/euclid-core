package us.ihmc.geometry.axisAngle.interfaces;

import us.ihmc.geometry.exceptions.NotAMatrix2DException;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.rotationConversion.RotationVectorConversion;
import us.ihmc.geometry.rotationConversion.YawPitchRollConversion;
import us.ihmc.geometry.tools.AxisAngleTools;
import us.ihmc.geometry.tools.EuclidCoreTools;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface for an axis-angle object.
 * <p>
 * An axis-angle is used to represent a 3D orientation by a unitary axis of components (x, y, z) and
 * an angle of rotation usually expressed in radians.
 * </p>
 *
 * @author Sylvain
 *
 * @param T the final type of the axis-angle used.
 */
public interface AxisAngleReadOnly
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
      return Math.abs(getAngle()) < epsilon || (Math.abs(getX()) < epsilon && Math.abs(getY()) < epsilon);
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
    * Converts and gets the orientation represented by this axis-angle as a rotation vector. See
    * {@link RotationVectorConversion#convertAxisAngleToRotationVector(AxisAngleReadOnly, Vector3DBasics)}.
    *
    * @param rotationVectorToPack rotation vector in which the orientation of this axis-angle is
    *           stored. Modified.
    */
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
   default double get(int index)
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
   default float get32(int index)
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
    * Transforms the given tuple by this axis-angle.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   default void transform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
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
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      AxisAngleTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple by this axis-angle.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void transform(Tuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform, true);
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
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      AxisAngleTools.transform(this, tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple by this quaternion.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    */
   default void transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
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
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      AxisAngleTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given 3D matrix by this axis-angle.
    * <p>
    * matrixToTransform = R(this) * matrixToTransform * R(this)<sup>-1</sup> <br>
    * where R(axisAngle) is the function that converts an axis-angle into a rotation matrix.
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    */
   default void transform(Matrix3D matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
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
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      AxisAngleTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given quaternion by this axis-angle.
    * <p>
    * quaternionToTransform = Q(this) * quaternionToTransform <br>
    * where Q(axisAngle) is the function that converts an axis-angle into a quaternion.
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
    * where Q(axisAngle) is the function that converts an axis-angle into a quaternion.
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   default void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      AxisAngleTools.transform(this, quaternionOriginal, quaternionTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this * vectorToTransform.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
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
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      AxisAngleTools.transform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given rotation matrix by this axis-angle.
    * <p>
    * matrixToTransform = R(this) * matrixToTransform <br>
    * where R(axisAngle) is the function that converts an axis-angle to a rotation matrix.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
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
   default void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      AxisAngleTools.transform(this, matrixOriginal, matrixTransformed);
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
    */
   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
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
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      AxisAngleTools.inverseTransform(this, tupleOriginal, tupleTransformed);
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
   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, true);
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
    * @throws NotAMatrix2DException if this axis-angle does not represent a transformation in the XY
    *            plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      AxisAngleTools.inverseTransform(this, tupleOriginal, tupleTransformed, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this axis-angle.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this axis-angle
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this axis-angle
    *            does not represent a transformation in the XY plane.
    */
   default void inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
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
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      AxisAngleTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionToTransform}.
    * <p>
    * quaternionToTransform = this<sup>-1</sup> * quaternionToTransform <br>
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Modified.
    */
   default void inverseTransform(QuaternionBasics quaternionToTransform)
   {
      inverseTransform(quaternionToTransform, quaternionToTransform);
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
   default void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      AxisAngleTools.inverseTransform(this, quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector by this
    * axis-angle.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this<sup>-1</sup> * vectorToTransform.xyz * this
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
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
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      AxisAngleTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * axis-angle.
    * <p>
    * matrixToTransform = this<sup>-1</sup> * matrixToTransform * this
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Not modified.
    */
   default void inverseTransform(Matrix3D matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
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
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      AxisAngleTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the given rotation matrix {@code matrixToTransform}
    * by this axis-angle.
    * <p>
    * matrixToTransform = this<sup>-1</sup> * matrixToTransform
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void inverseTransform(RotationMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
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
      double diff;

      diff = getX() - other.getX();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = getY() - other.getY();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = getZ() - other.getZ();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = getAngle() - other.getAngle();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      return true;
   }
}
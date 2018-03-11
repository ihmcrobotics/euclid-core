package us.ihmc.euclid.orientation.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
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
 * Base interface to easily identify implementations that represent a physical orientation in 3
 * dimensions.
 * <p>
 * Even though the representation used is unknown at this level of abstraction, this interface
 * allows to enforce a minimum set of features that all representations of an orientation should
 * provide, such as transformation functions.
 * </p>
 *
 * @author Sylvain
 */
public interface Orientation3DReadOnly
{
   static final double ORIENTATION_2D_EPSILON = 1.0e-8;

   /**
    * Tests if this orientation 3D actually represents a rotation strictly around the z-axis.
    * <p>
    * This is commonly used to test if this orientation can be used to transform 2D geometry object.
    * </p>
    * <p>
    * This test uses the default tolerance {@link #ORIENTATION_2D_EPSILON}, to specify explicitly
    * the tolerance to be use, see {@link #isOrientation2D(double)}.
    * </p>
    *
    * @return {@code true} if this orientation represents a 2D orientation in the XY-plane,
    *         {@code false} otherwise.
    */
   default boolean isOrientation2D()
   {
      return isOrientation2D(ORIENTATION_2D_EPSILON);
   }

   /**
    * Tests if this orientation 3D actually represents a rotation strictly around the z-axis.
    * <p>
    * This is commonly used to test if this orientation can be used to transform 2D geometry object.
    * </p>
    * <p>
    * The implementation of this test depends on the type of representation used for this
    * orientation.
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @return {@code true} if this orientation represents a 2D orientation in the XY-plane,
    *         {@code false} otherwise.
    */
   boolean isOrientation2D(double epsilon);

   /**
    * Tests if this orientation 3D actually represents a rotation strictly around the z-axis.
    * <p>
    * This is commonly used to test if this orientation can be used to transform 2D geometry object.
    * </p>
    *
    * @throws NotAnOrientation2DException if this orientation does not represent a rotation strictly
    *            around the z-axis.
    */
   default void checkIfOrientation2D()
   {
      checkIfOrientation2D(ORIENTATION_2D_EPSILON);
   }

   /**
    * Tests if this orientation 3D actually represents a rotation strictly around the z-axis.
    * <p>
    * This is commonly used to test if this orientation can be used to transform 2D geometry object.
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @throws NotAnOrientation2DException if this orientation does not represent a rotation strictly
    *            around the z-axis.
    */
   default void checkIfOrientation2D(double epsilon)
   {
      if (!isOrientation2D(epsilon))
         throw new NotAnOrientation2DException(this);
   }

   /**
    * Converts, if necessary, this orientation into a 3-by-3 rotation matrix.
    *
    * @param rotationMatrixToPack the rotation matrix into which this orientation is to be stored.
    *           Modified.
    */
   void get(RotationMatrix rotationMatrixToPack);

   /**
    * Converts, if necessary, this orientation into an axis-angle.
    *
    * @param axisAngleToPack the axis-angle into which this orientation is to be stored. Modified.
    */
   void get(AxisAngleBasics axisAngleToPack);

   /**
    * Converts, if necessary, this orientation in a quaternion.
    *
    * @param quaternionToPack the quaternion into which this orientation is to be stored. Modified.
    */
   void get(QuaternionBasics quaternionToPack);

   /**
    * Converts this orientation in a 3D rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector in which this orientation is to be stored.
    *           Modified.
    */
   void getRotationVector(Vector3DBasics rotationVectorToPack);

   /**
    * Converts this orientation in a yaw-pitch-roll representation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * <p>
    * The yaw-pitch-roll representation describes a 3D orientation as a succession of three
    * rotations around three axes:
    * <ol>
    * <li>yaw: rotation around the z-axis,
    * <li>pitch: rotation around the y-axis,
    * <li>roll: rotation around the x-axis.
    * </ol>
    * </p>
    * <p>
    * As an example, a rotation matrix can be computed from a yaw-pitch-roll representation as
    * follows:
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   void getYawPitchRoll(double[] yawPitchRollToPack);

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of this orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   double getYaw();

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of this
    * orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   double getPitch();

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of this
    * orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   double getRoll();

   /**
    * Transforms the given tuple by this orientation.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the
    * tuple is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the 3D tuple to be transformed. Modified.
    */
   default void transform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the
    * tuple is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    */
   void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed);

   default void addTransform(Tuple3DBasics tupleToTransform)
   {
      addTransform(tupleToTransform, tupleToTransform);
   }

   void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed);

   default void transform(Tuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform, true);
   }

   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   default void transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane);

   default void transform(Matrix3D matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed);

   default void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   default void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      RotationMatrixTools.multiply(this, false, matrixOriginal, false, matrixTransformed);
   }

   default void transform(RotationScaleMatrix matrixToTransform)
   {
      transform(matrixToTransform.getRotationMatrix());
   }

   default void transform(RotationScaleMatrixReadOnly matrixOriginal, RotationScaleMatrix matrixTransformed)
   {
      matrixTransformed.set(matrixOriginal);
      transform(matrixOriginal.getRotationMatrix(), matrixTransformed.getRotationMatrix());
   }

   void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed);

   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   default void transform(QuaternionBasics quaternionToTransform)
   {
      transform(quaternionToTransform, quaternionToTransform);
   }

   default void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      QuaternionTools.multiply(this, false, quaternionOriginal, false, quaternionTransformed);
   }

   void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed);

   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform, true);
   }

   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   default void inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane);

   default void inverseTransform(Matrix3D matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed);

   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   default void inverseTransform(RotationMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      RotationMatrixTools.multiply(this, true, matrixOriginal, false, matrixTransformed);
   }

   default void inverseTransform(RotationScaleMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform.getRotationMatrix());
   }

   default void inverseTransform(RotationScaleMatrixReadOnly matrixOriginal, RotationScaleMatrix matrixTransformed)
   {
      matrixTransformed.set(matrixOriginal);
      inverseTransform(matrixOriginal.getRotationMatrix(), matrixTransformed.getRotationMatrix());
   }

   void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed);

   default void inverseTransform(QuaternionBasics quaternionToTransform)
   {
      inverseTransform(quaternionToTransform, quaternionToTransform);
   }

   default void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      QuaternionTools.multiply(this, true, quaternionOriginal, false, quaternionTransformed);
   }

   /**
    * Provides a {@code String} representation of this orientation converted to yaw-pitch-roll
    * angles as follows: yaw-pitch-roll: (yaw, pitch, roll).
    *
    * @return a string representation of this orientation 3D.
    */
   default String toStringAsYawPitchRoll()
   {
      return EuclidCoreIOTools.getStringOf("yaw-pitch-roll: (", ")", ", ", getYaw(), getPitch(), getRoll());
   }
}

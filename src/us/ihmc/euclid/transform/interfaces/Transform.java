package us.ihmc.euclid.transform.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Minimal interface representing a geometric transformation.
 * <p>
 * In this library, a geometric transformation typically includes: scaling, rotation, and
 * translation. For instance, {@link RigidBodyTransform} represents a 4-by-4 homogeneous
 * transformation matrix that can rotate and translate a {@link Point3D}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Transform
{
   /**
    * Transforms the given {@code pointToTransform} by this transform.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    */
   default void transform(Point3DBasics pointToTransform)
   {
      transform(pointToTransform, pointToTransform);
   }

   /**
    * Transforms the given {@code pointOriginal} by this transform and stores the result in
    * {@code pointTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointOriginal the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    */
   void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed);

   /**
    * Transforms the given {@code vectorToTransform} by this transform.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void transform(Vector3DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this transform and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed);

   /**
    * Transforms the given {@code orientationToTransform} by this transform.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given quaternion. No scale or translation is applied to the
    * quaternion such that the output of this method is still a unit-quaternion.
    * </p>
    *
    * @param orientationToTransform the orientation to transform. Modified.
    */
   default void transform(Orientation3DBasics orientationToTransform)
   {
      transform(orientationToTransform, orientationToTransform);
   }

   /**
    * Transforms the given {@code orientationOriginal} by this transform and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given quaternion. No scale or translation is applied to the
    * quaternion such that the output of this method is still a unit-quaternion.
    * </p>
    *
    * @param orientationOriginal the orientation to transform. Not modified.
    * @param orientationTransformed the orientation in which the result is stored. Modified.
    */
   void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed);

   /**
    * Transforms the vector part (x, y, z) of the given {@code vector4DToTransform} as a 3D vector
    * and translates it by {@code s} times the translation part of the transform. The scalar part
    * (s) remains unchanged.
    * <p>
    * Note that for {@code s = 0}, a 4D vector behaves as a 3D vector, and for {@code s = 1} it
    * behaves as a 3D point.
    * </p>
    * <p>
    * <li>{@link RigidBodyTransform} rotates then translates a vector.
    * <li>{@link QuaternionBasedTransform} rotates then translates a vector.
    * <li>{@link AffineTransform} scales, rotates, then translates a vector.
    * </p>
    *
    * @param vectorToTransform the 4D vector to transform. Modified.
    */
   default void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   /**
    * Transforms the vector part (x, y, z) of the given {@code vector4DOriginal} as a 3D vector and
    * translates it by {@code s} times the translation part of the transform. The scalar part (s)
    * remains unchanged.
    * <p>
    * Note that for {@code s = 0}, a 4D vector behaves as a 3D vector, and for {@code s = 1} it
    * behaves as a 3D point.
    * </p>
    * <p>
    * <li>{@link RigidBodyTransform} rotates then translates a vector.
    * <li>{@link QuaternionBasedTransform} rotates then translates a vector.
    * <li>{@link AffineTransform} scales, rotates, then translates a vector.
    * </p>
    *
    * @param vectorOriginal the 4D vector to transform. Not modified.
    * @param vectorTransformed the 4D vector in which the result is stored. Modified.
    */
   void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed);

   /**
    * Transforms the given {@code point2DToTransform} by this transform.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transform(Point2DBasics pointToTransform)
   {
      transform(pointToTransform, true);
   }

   /**
    * Transforms the given {@code point2DOriginal} by this transform and stores the result in
    * {@code point2DTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointOriginal the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
   {
      transform(pointOriginal, pointTransformed, true);
   }

   /**
    * Transforms the given {@code point2DToTransform} by this transform.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of this transform is not a transformation in the XY plane.
    */
   default void transform(Point2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(pointToTransform, pointToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code point2DOriginal} by this transform and stores the result in
    * {@code point2DTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointOriginal the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of this transform is not a transformation in the XY plane.
    */
   void transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane);

   /**
    * Transforms the given {@code vector2DToTransform} by this transform.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transform(Vector2DBasics vectorToTransform)
   {
      transform(vectorToTransform, true);
   }

   /**
    * Transforms the given {@code vector2DOriginal} by this transform and stores the result in
    * {@code vector2DTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
   {
      transform(vectorOriginal, vectorTransformed, true);
   }

   /**
    * Transforms the given {@code vector2DToTransform} by this transform.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transform(Vector2DBasics vectorToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(vectorToTransform, vectorToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code vector2DOriginal} by this transform and stores the result in
    * {@code vector2DTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   void transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane);

   /**
    * Transforms the given {@code matrixToTransform} by this transform.
    * <p>
    * WARNING: <b> This is different from concatenating orientations.</b>
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a matrix.
    * <li>{@link QuaternionBasedTransform} rotates a matrix.
    * <li>{@link AffineTransform} scales then rotates a matrix.
    * </ul>
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    */
   default void transform(Matrix3DBasics matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given {@code matrixOriginal} by this transform and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * WARNING: <b> This is different from concatenating orientations.</b>
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a matrix.
    * <li>{@link QuaternionBasedTransform} rotates a matrix.
    * <li>{@link AffineTransform} scales then rotates a matrix.
    * </ul>
    * </p>
    *
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result in stored. Modified.
    */
   void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed);

   /**
    * Transforms the given {@code rotationMatrix} by this transform.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given rotation matrix. No scale or translation is applied to
    * the rotation matrix such that the output of this method is still a proper rotation matrix.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given {@code matrixOriginal} by this transform and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given rotation matrix. No scale or translation is applied to
    * the rotation matrix such that the output of this method is still a proper rotation matrix.
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      transform((Orientation3DReadOnly) matrixOriginal, (Orientation3DBasics) matrixTransformed);
   }

   /**
    * Transforms the given {@code rigidBodyTransformToTransform} by this transform.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param rigidBodyTransformToTransform the rigid-body transform to transform. Modified.
    */
   default void transform(RigidBodyTransform rigidBodyTransformToTransform)
   {
      transform(rigidBodyTransformToTransform, rigidBodyTransformToTransform);
   }

   /**
    * Transforms the given {@code original} by this transform and stores the result in
    * {@code transformed}.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param original the rigid-body transform to transform. Not modified.
    * @param transformed the rigid-body transform in which the result is stored. Modified.
    */
   void transform(RigidBodyTransform original, RigidBodyTransform transformed);

   /**
    * Transforms the given {@code quaternionBasedTransformToTransform} by this transform.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param quaternionBasedTransformToTransform the quaternion based transform to transform.
    *           Modified.
    */
   default void transform(QuaternionBasedTransform quaternionBasedTransformToTransform)
   {
      transform(quaternionBasedTransformToTransform, quaternionBasedTransformToTransform);
   }

   /**
    * Transforms the given {@code original} by this transform and stores the result in
    * {@code transformed}.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param original the quaternion based transform to transform. Not modified.
    * @param transformed the quaternion based transform in which the result is stored. Modified.
    */
   void transform(QuaternionBasedTransform original, QuaternionBasedTransform transformed);

   /**
    * Transforms the given {@code affineTransformToTransform} by this transform.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param affineTransformToTransform the affine transform to transform. Modified.
    */
   default void transform(AffineTransform affineTransformToTransform)
   {
      transform(affineTransformToTransform, affineTransformToTransform);
   }

   /**
    * Transforms the given {@code original} by this transform and stores the result in
    * {@code transformed}.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param original the affine transform to transform. Not modified.
    * @param transformed the affine transform in which the result is stored. Modified.
    */
   void transform(AffineTransform original, AffineTransform transformed);

   /**
    * Performs the inverse of the transform on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Point3DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    */
   default void inverseTransform(Point3DBasics pointToTransform)
   {
      inverseTransform(pointToTransform, pointToTransform);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Point3DReadOnly, Point3DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param pointOriginal the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    */
   void inverseTransform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed);

   /**
    * Performs the inverse of the transform on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector3DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void inverseTransform(Vector3DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores
    * the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector3DReadOnly, Vector3DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed);

   /**
    * Performs the inverse of the transform on the given orientation {@code orientationToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Orientation3DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param orientationToTransform the orientation to transform. Modified.
    */
   default void inverseTransform(Orientation3DBasics orientationToTransform)
   {
      inverseTransform(orientationToTransform, orientationToTransform);
   }

   /**
    * Performs the inverse of the transform on the given orientation {@code orientationOriginal} and
    * stores the result in {@code orientationTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Orientation3DReadOnly, Orientation3DBasics)} with
    * the inverse of this transform.
    * </p>
    *
    * @param orientationOriginal the orientation to transform. Not modified.
    * @param orientationTransformed the orientation in which the result is stored. Modified.
    */
   void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed);

   /**
    * Performs the inverse of the transform on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector4DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param vectorToTransform the 4D vector to transform. Modified.
    */
   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores
    * the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector4DReadOnly, Vector4DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param vectorOriginal the 4D vector to transform. Not modified.
    * @param vectorTransformed the 4D vector in which the result is stored. Modified.
    */
   void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed);

   /**
    * Performs the inverse of the transform on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Point2DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param point2DToTransform the point to transform. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void inverseTransform(Point2DBasics point2DToTransform)
   {
      inverseTransform(point2DToTransform, true);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Point2DReadOnly, Point2DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param point2DOriginal the point to transform. Not modified.
    * @param point2DTransformed the point in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void inverseTransform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed)
   {
      inverseTransform(point2DOriginal, point2DTransformed, true);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Point2DBasics, boolean)} with the inverse of
    * this transform.
    * </p>
    *
    * @param point2DToTransform the point to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of this transform is not a transformation in the XY plane.
    */
   default void inverseTransform(Point2DBasics point2DToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(point2DToTransform, point2DToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Point2DReadOnly, Point2DBasics, boolean)} with
    * the inverse of this transform.
    * </p>
    *
    * @param point2DOriginal the point to transform. Not modified.
    * @param point2DTransformed the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of this transform is not a transformation in the XY plane.
    */
   void inverseTransform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane);

   /**
    * Performs the inverse of the transform on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector2DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param vector2DToTransform the vector to transform. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void inverseTransform(Vector2DBasics vector2DToTransform)
   {
      inverseTransform(vector2DToTransform, true);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores
    * the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector2DReadOnly, Vector2DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param vector2DOriginal the vector to transform. Not modified.
    * @param vector2DTransformed the vector in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void inverseTransform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed)
   {
      inverseTransform(vector2DOriginal, vector2DTransformed, true);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector2DBasics, boolean)} with the inverse of
    * this transform.
    * </p>
    *
    * @param vector2DToTransform the vector to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void inverseTransform(Vector2DBasics vector2DToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(vector2DToTransform, vector2DToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores
    * the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector2DReadOnly, Vector2DBasics, boolean)}
    * with the inverse of this transform.
    * </p>
    *
    * @param vector2DOriginal the vector to transform. Not modified.
    * @param vector2DTransformed the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   void inverseTransform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane);

   /**
    * Performs the inverse of the transform on the given matrix {@code matrixToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Matrix3DBasics)} with the inverse of this transform.
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    */
   default void inverseTransform(Matrix3DBasics matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the inverse of the transform on the given matrix {@code matrixOriginal} and stores
    * the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Matrix3DReadOnly, Matrix3DBasics)} with the inverse
    * of this transform.
    * </p>
    *
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result in stored. Modified.
    */
   void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed);

   /**
    * Performs the inverse of the transform on the given rotation matrix {@code matrixToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(RotationMatrix)} with the inverse of this
    * transform.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void inverseTransform(RotationMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the inverse of the transform on the given matrix {@code matrixOriginal} and stores
    * the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(RotationMatrixReadOnly, RotationMatrix)} with
    * the inverse of this transform.
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      inverseTransform((Orientation3DReadOnly) matrixOriginal, (Orientation3DBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transform on the given {@code rigidBodyTransformToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(RigidBodyTransform)} with the inverse of this
    * transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param rigidBodyTransformToTransform the rigid-body transform to transform. Modified.
    */
   default void inverseTransform(RigidBodyTransform rigidBodyTransformToTransform)
   {
      inverseTransform(rigidBodyTransformToTransform, rigidBodyTransformToTransform);
   }

   /**
    * Performs the inverse of the transform on the given {@code original} and stores the result in
    * {@code transformed}.
    * <p>
    * This is equivalent to calling {@link #transform(RigidBodyTransform, RigidBodyTransform)} with
    * the inverse of this transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param original the rigid-body transform to transform. Not modified.
    * @param transformed the rigid-body transform in which the result is stored. Modified.
    */
   void inverseTransform(RigidBodyTransform original, RigidBodyTransform transformed);

   /**
    * Performs the inverse of the transform on the given
    * {@code quaternionBasedTransformToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(QuaternionBasedTransform)} with the inverse of
    * this transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param quaternionBasedTransformToTransform the quaternion based transform to transform.
    *           Modified.
    */
   default void inverseTransform(QuaternionBasedTransform quaternionBasedTransformToTransform)
   {
      inverseTransform(quaternionBasedTransformToTransform, quaternionBasedTransformToTransform);
   }

   /**
    * Performs the inverse of the transform on the given {@code original} stores the result in
    * {@code transformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(QuaternionBasedTransform, QuaternionBasedTransform)} with the inverse of
    * this transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param original the quaternion based transform to transform. Not modified.
    * @param transformed the quaternion based transform in which the result is stored. Modified.
    */
   void inverseTransform(QuaternionBasedTransform original, QuaternionBasedTransform transformed);

   /**
    * Performs the inverse of the transform on the given {@code affineTransformToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(AffineTransform)} with the inverse of this
    * transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param affineTransformToTransform the affine transform to transform. Modified.
    */
   default void inverseTransform(AffineTransform affineTransformToTransform)
   {
      inverseTransform(affineTransformToTransform, affineTransformToTransform);
   }

   /**
    * Performs the inverse of the transform on the given {@code original} and stores the result in
    * {@code transformed}.
    * <p>
    * This is equivalent to calling {@link #transform(AffineTransform, AffineTransform)} with the
    * inverse of this transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    *
    * @param original the affine transform to transform. Not modified.
    * @param transformed the affine transform in which the result is stored. Modified.
    */
   void inverseTransform(AffineTransform original, AffineTransform transformed);
}

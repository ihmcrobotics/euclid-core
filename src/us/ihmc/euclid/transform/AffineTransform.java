package us.ihmc.euclid.transform;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.exceptions.NotARotationScaleMatrixException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * An {@code AffineTransform} represents a 4-by-4 transformation matrix that can scale, rotate, and
 * translate.
 * <p>
 * For efficiency and readability, the transform is never stored in a 4-by-4 matrix.
 * </p>
 * <p>
 * The {@code AffineTransform} is composed of {@link RotationScaleMatrix} to scale and rotate, and a
 * {@link Vector3D} to translate.
 * </p>
 * <p>
 * Because the {@code RotationScaleMatrix} a restrictive type of matrix, the algebra available with
 * this is somewhat restricted to keep the rotation-scale matrix proper at all time. For instance,
 * an affine transform cannot be inverted. However, it can still perform the inverse of the
 * transform it represents on geometry objects.
 * </p>
 * <p>
 * A few special cases to keep in mind:
 * <ul>
 * <li>when transforming a {@link QuaternionBasics}, the rotation part of this transform is prepend
 * to the quaternion, such that the output remains a proper unit-quaternion that still only
 * describes a rotation.
 * <li>when transforming a {@link RotationMatrix}, the rotation part of this transform is prepend to
 * the rotation matrix, such that the output remains a proper rotation matrix.
 * <li>when applying this transform on a {@link Point3DBasics} or {@link Point2DBasics}, this object
 * is, in order, scaled, rotated, and then translated.
 * <li>when applying this transform on a {@link Vector3DBasics} or {@link Vector2DBasics}, this
 * object is, in order, scaled, and then rotated. It is NOT translated.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class AffineTransform implements Transform, EpsilonComparable<AffineTransform>, Settable<AffineTransform>, Clearable
{
   /** The rotation plus scaling part of this transform. */
   private final RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
   /** The translation part of this transform. */
   private final Vector3D translationVector = new Vector3D();

   /**
    * Creates a new affine transform set to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public AffineTransform()
   {
   }

   /**
    * Creates a new affine transform and sets it to {@code other}.
    *
    * @param other the other affine transform to copy. Not modified.
    */
   public AffineTransform(AffineTransform other)
   {
      set(other);
   }

   /**
    * Creates a new affine transform and sets it to {@code rigidBodyTransform}.
    * <p>
    * This affine transform has no scaling (1.0, 1.0, 1.0).
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to copy. Not modified.
    */
   public AffineTransform(RigidBodyTransform rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new affine transform and initializes it from the given rotation-scale matrix and the
    * given translation.
    *
    * @param rotationScaleMatrix the rotation-scale matrix to copy. Not modified.
    * @param translation the translation to copy. Not modified.
    */
   public AffineTransform(RotationScaleMatrixReadOnly rotationScaleMatrix, Tuple3DReadOnly translation)
   {
      set(rotationScaleMatrix, translation);
   }

   /**
    * Resets this affine transform to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public void setIdentity()
   {
      rotationScaleMatrix.setIdentity();
      translationVector.setToZero();
   }

   /**
    * Resets this affine transform to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   @Override
   public void setToZero()
   {
      setIdentity();
   }

   /**
    * Sets the rotation part to represent a 'zero' rotation.
    * <p>
    * This method does NOT affect the scale part of this transform.
    * </p>
    */
   public void setRotationToZero()
   {
      rotationScaleMatrix.setRotationToZero();
   }

   /**
    * Sets all the scale factors to 1.0.
    */
   public void resetScale()
   {
      rotationScaleMatrix.resetScale();
   }

   /**
    * Sets the translation part to zero.
    */
   public void setTranslationToZero()
   {
      translationVector.setToZero();
   }

   /**
    * Sets all the components of this affine transform making it invalid.
    */
   @Override
   public void setToNaN()
   {
      setRotationToNaN();
      setTranslationToNaN();
   }

   /**
    * Sets all the components of the rotation-scale matrix to {@link Double#NaN}.
    * <p>
    * See {@link RotationScaleMatrix#setToNaN()}.
    * </p>
    */
   public void setRotationToNaN()
   {
      rotationScaleMatrix.setToNaN();
   }

   /**
    * Sets all the components of the translation vector to {@link Double#NaN}.
    * <p>
    * See {@link Vector3D#setToNaN()}.
    * </p>
    */
   public void setTranslationToNaN()
   {
      translationVector.setToNaN();
   }

   /**
    * Tests if at least one element of this transform is equal to {@linkplain Double#NaN}.
    *
    * @return {@code true} if at least one element of this transform is equal to
    *         {@linkplain Double#NaN}, {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return rotationScaleMatrix.containsNaN() || translationVector.containsNaN();
   }

   /**
    * Normalize the rotation part of this transform.
    * <p>
    * This method does NOT affect the scale part of this transform.
    * </p>
    *
    * @throws NotARotationMatrixException if the orthonormalization failed.
    */
   public void normalizeRotationPart()
   {
      rotationScaleMatrix.normalizeRotationMatrix();
   }

   /**
    * Sets this affine transform from the given 12 coefficients.
    *
    * @param m00 the 1st row 1st column component of the rotation-scale part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation-scale part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation-scale part of this transform.
    * @param m03 the x-component of the translation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation-scale part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation-scale part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation-scale part of this transform.
    * @param m13 the y-component of the translation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation-scale part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation-scale part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation-scale part of this transform.
    * @param m23 the z-component of the translation part of this transform.
    * @throws NotARotationScaleMatrixException if the components for the rotation-scale part do not
    *            represent a rotation-scale matrix.
    */
   public void set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                   double m23)
   {
      rotationScaleMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   /**
    * Sets this affine transform to the {@code other}.
    *
    * @param other the other affine transform to copy the values from. Not modified.
    */
   @Override
   public void set(AffineTransform other)
   {
      rotationScaleMatrix.set(other.rotationScaleMatrix);
      translationVector.set(other.translationVector);
   }

   /**
    * Sets this affine transform to the given rigid-body transform.
    * <p>
    * The scaling part of this transform will be reset.
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to copy the values from. Not modified.
    */
   public void set(RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.get(rotationScaleMatrix, translationVector);
   }

   /**
    * Sets the raw components of this affine transform from the given {@code matrix}.
    * <p>
    * The rotation-scale part M is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 0) matrix.get(0, 1) matrix.get(0, 2) \
    * M = | matrix.get(1, 0) matrix.get(1, 1) matrix.get(1, 2) |
    *     \ matrix.get(2, 0) matrix.get(2, 1) matrix.get(2, 2) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 3) \
    * T = | matrix.get(1, 3) |
    *     \ matrix.get(2, 3) /
    * </pre>
    * </p>
    *
    * @param matrix the matrix to get this transform's components from. Not modified.
    * @throws NotARotationScaleMatrixException if the resulting matrix for the rotation-scale part
    *            of this transform is not a rotation-scale matrix.
    */
   public void set(DenseMatrix64F matrix)
   {
      rotationScaleMatrix.set(matrix);
      translationVector.set(0, 3, matrix);
   }

   /**
    * Sets the raw components of this affine transform from the given {@code matrix}.
    * <p>
    * The rotation-scale part M is set as follows:
    *
    * <pre>
    *     / matrix.get(startRow + 0, startColumn + 0) matrix.get(startRow + 0, startColumn + 1) matrix.get(startRow + 0, startColumn + 2) \
    * M = | matrix.get(startRow + 1, startColumn + 0) matrix.get(startRow + 1, startColumn + 1) matrix.get(startRow + 1, startColumn + 2) |
    *     \ matrix.get(startRow + 2, startColumn + 0) matrix.get(startRow + 2, startColumn + 1) matrix.get(startRow + 2, startColumn + 2) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(startRow + 0, startColumn + 3) \
    * T = | matrix.get(startRow + 1, startColumn + 3) |
    *     \ matrix.get(startRow + 2, startColumn + 3) /
    * </pre>
    * </p>
    *
    * @param matrix the matrix to get this transform's components from. Not modified.
    * @param startRow the row index of the first component to read.
    * @param startColumn the column index of the first component to read.
    * @throws NotARotationScaleMatrixException if the resulting matrix for the rotation-scale part
    *            of this transform is not a rotation-scale matrix.
    */
   public void set(DenseMatrix64F matrix, int startRow, int startColumn)
   {
      rotationScaleMatrix.set(startRow, startColumn, matrix);
      translationVector.set(startRow, startColumn + 3, matrix);
   }

   /**
    * Sets the raw components of this affine transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part M is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[1] transformArray[ 2] \
    * M = | transformArray[4] transformArray[5] transformArray[ 6] |
    *     \ transformArray[8] transformArray[9] transformArray[10] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / transformArray[ 3] \
    * T = | transformArray[ 7] |
    *     \ transformArray[11] /
    * </pre>
    * </p>
    *
    * @param transformArray the 1D row-major array to get this transform's components from. Not
    *           modified.
    * @throws NotARotationScaleMatrixException if the resulting matrix for the rotation-scale part
    *            of this transform is not a rotation-scale matrix.
    */
   public void set(double[] transformArray)
   {
      double m00 = transformArray[0];
      double m01 = transformArray[1];
      double m02 = transformArray[2];
      double m03 = transformArray[3];
      double m10 = transformArray[4];
      double m11 = transformArray[5];
      double m12 = transformArray[6];
      double m13 = transformArray[7];
      double m20 = transformArray[8];
      double m21 = transformArray[9];
      double m22 = transformArray[10];
      double m23 = transformArray[11];

      rotationScaleMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   /**
    * Sets the rotation-scale and translation parts of this transform separately.
    *
    * @param rotationScaleMatrix the matrix used to set the rotation-scale part of this transform.
    *           Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if the given {@code rotationScaleMatrix} is not a
    *            rotation-scale matrix.
    */
   public void set(Matrix3DReadOnly rotationScaleMatrix, Tuple3DReadOnly translation)
   {
      this.rotationScaleMatrix.set(rotationScaleMatrix);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation-scale and translation parts of this transform separately.
    *
    * @param rotationScaleMatrix the matrix used to set the rotation-scale part of this transform.
    *           Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    */
   public void set(RotationScaleMatrixReadOnly rotationScaleMatrix, Tuple3DReadOnly translation)
   {
      this.rotationScaleMatrix.set(rotationScaleMatrix);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param scale the scalar used to set the scale part of this transform.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(Matrix3DReadOnly rotationMatrix, double scale, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scale);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param scaleX the new x-component of the scale part of this transform.
    * @param scaleY the new y-component of the scale part of this transform.
    * @param scaleZ the new z-component of the scale part of this transform.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(Matrix3DReadOnly rotationMatrix, double scaleX, double scaleY, double scaleZ, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scaleX, scaleY, scaleZ);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param scales the tuple used to set the scale part of this transform. Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(Matrix3DReadOnly rotationMatrix, Tuple3DReadOnly scales, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scales);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param scale the scalar used to set the scale part of this transform.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, double scale, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scale);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param scaleX the new x-component of the scale part of this transform.
    * @param scaleY the new y-component of the scale part of this transform.
    * @param scaleZ the new z-component of the scale part of this transform.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, double scaleX, double scaleY, double scaleZ, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scaleX, scaleY, scaleZ);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param scales the tuple used to set the scale part of this transform. Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly scales, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scales);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param axisAngle the axis-angle used to set the rotation part of this transform. Not modified.
    * @param scale the scalar used to set the scale part of this transform.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(AxisAngleReadOnly axisAngle, double scale, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(axisAngle, scale);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param axisAngle the axis-angle used to set the rotation part of this transform. Not modified.
    * @param scaleX the new x-component of the scale part of this transform.
    * @param scaleY the new y-component of the scale part of this transform.
    * @param scaleZ the new z-component of the scale part of this transform.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(AxisAngleReadOnly axisAngle, double scaleX, double scaleY, double scaleZ, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(axisAngle, scaleX, scaleY, scaleZ);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param axisAngle the axis-angle used to set the rotation part of this transform. Not modified.
    * @param scales the tuple used to set the scale part of this transform. Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(AxisAngleReadOnly axisAngle, Tuple3DReadOnly scales, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(axisAngle, scales);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param quaternion the quaternion used to set the rotation part of this transform. Not
    *           modified.
    * @param scale the scalar used to set the scale part of this transform.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(QuaternionReadOnly quaternion, double scale, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(quaternion, scale);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param quaternion the quaternion used to set the rotation part of this transform. Not
    *           modified.
    * @param scaleX the new x-component of the scale part of this transform.
    * @param scaleY the new y-component of the scale part of this transform.
    * @param scaleZ the new z-component of the scale part of this transform.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(QuaternionReadOnly quaternion, double scaleX, double scaleY, double scaleZ, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(quaternion, scaleX, scaleY, scaleZ);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation, scale, and translation parts of this transform separately.
    *
    * @param quaternion the quaternion used to set the rotation part of this transform. Not
    *           modified.
    * @param scales the tuple used to set the scale part of this transform. Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(QuaternionReadOnly quaternion, Tuple3DReadOnly scales, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.set(quaternion, scales);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation part of this transform to the given axis-angle.
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param axisAngle the axis-angle used to set the rotation part of this transform. Not modified.
    */
   public void setRotation(AxisAngleReadOnly axisAngle)
   {
      rotationScaleMatrix.setRotation(axisAngle);
   }

   /**
    * Sets the rotation part of this transform to the given rotation vector.
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to set the rotation part of this transform. Not
    *           modified.
    */
   public void setRotation(Vector3DReadOnly rotationVector)
   {
      rotationScaleMatrix.setRotation(rotationVector);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotation(DenseMatrix64F rotationMatrix)
   {
      rotationScaleMatrix.setRotation(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to the given quaternion.
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param quaternion the quaternion used to set the rotation part of this transform. Not
    *           modified.
    */
   public void setRotation(QuaternionReadOnly quaternion)
   {
      rotationScaleMatrix.setRotation(quaternion);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotation(Matrix3DReadOnly rotationMatrix)
   {
      rotationScaleMatrix.setRotation(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    */
   public void setRotation(RotationMatrix rotationMatrix)
   {
      rotationScaleMatrix.setRotation(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * z-axis of an angle {@code yaw}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \
    * R = | sin(yaw)  cos(yaw) 0 |
    *     \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setRotationYaw(double yaw)
   {
      rotationScaleMatrix.setRotationYaw(yaw);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * y-axis of an angle {@code pitch}.
    *
    * <pre>
    *     /  cos(pitch) 0 sin(pitch) \
    * R = |      0      1     0      |
    *     \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setRotationPitch(double pitch)
   {
      rotationScaleMatrix.setRotationPitch(pitch);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * x-axis of an angle {@code roll}.
    *
    * <pre>
    *     / 1     0          0     \
    * R = | 0 cos(roll) -sin(roll) |
    *     \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationRoll(double roll)
   {
      rotationScaleMatrix.setRotationRoll(roll);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given
    * yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    */
   public void setRotationYawPitchRoll(double[] yawPitchRoll)
   {
      rotationScaleMatrix.setRotationYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given
    * yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      rotationScaleMatrix.setRotationYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code eulerAngles}.
    *
    * <pre>
    *     / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * R = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *     \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    * <p>
    * This is equivalent to
    * {@code this.setRotationYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   public void setRotationEuler(Vector3DReadOnly eulerAngles)
   {
      rotationScaleMatrix.setRotationEuler(eulerAngles);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code rotX}, {@code rotY}, and {@code rotZ}.
    *
    * <pre>
    *     / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * R = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *     \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    * <p>
    * This is equivalent to {@code this.setRotationYawPitchRoll(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   public void setRotationEuler(double rotX, double rotY, double rotZ)
   {
      rotationScaleMatrix.setRotationEuler(rotX, rotY, rotZ);
   }

   /**
    * Sets each component of the scale part of this transform to {@code scale}.
    * <p>
    * This method does not affect the rotation part nor the translation part of this transform.
    * </p>
    *
    * @param scale the scalar used to set the scale part of this transform.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void setScale(double scale)
   {
      rotationScaleMatrix.setScale(scale);
   }

   /**
    * Sets the scale part of this transform to {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    * <p>
    * This method does not affect the rotation part nor the translation part of this transform.
    * </p>
    *
    * @param scaleX the new x-component of the scale part of this transform.
    * @param scaleY the new y-component of the scale part of this transform.
    * @param scaleZ the new z-component of the scale part of this transform.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void setScale(double scaleX, double scaleY, double scaleZ)
   {
      rotationScaleMatrix.setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the scale part of this transform to {@code scales}.
    * <p>
    * This method does not affect the rotation part nor the translation part of this transform.
    * </p>
    *
    * @param scales the tuple used to set the scale part of this transform. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void setScale(Tuple3DReadOnly scales)
   {
      rotationScaleMatrix.setScale(scales);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param x the x-component of the translation part.
    * @param y the y-component of the translation part.
    * @param z the z-component of the translation part.
    */
   public void setTranslation(double x, double y, double z)
   {
      translationVector.set(x, y, z);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param translation tuple used to set the translation part of this transform. Not modified.
    */
   public void setTranslation(Tuple3DReadOnly translation)
   {
      translationVector.set(translation);
   }

   /**
    * Performs the multiplication of two 4-by-4 transformation matrices.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other transform to multiply with this. Not modified.
    */
   public void preMultiply(RigidBodyTransform other)
   {
      other.transform(translationVector);
      translationVector.add(other.getTranslationVector());
      rotationScaleMatrix.preMultiply(other.getRotationMatrix());
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      rotationScaleMatrix.transform(pointOriginal, pointTransformed);
      pointTransformed.add(translationVector);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      rotationScaleMatrix.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      rotationScaleMatrix.transform(quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      rotationScaleMatrix.transform(vectorOriginal, vectorTransformed);
      vectorTransformed.addX(vectorTransformed.getS() * translationVector.getX());
      vectorTransformed.addY(vectorTransformed.getS() * translationVector.getY());
      vectorTransformed.addZ(vectorTransformed.getS() * translationVector.getZ());
   }

   /** {@inheritDoc} */
   @Override
   public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      rotationScaleMatrix.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      rotationScaleMatrix.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationScaleMatrix.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
      pointTransformed.add(translationVector.getX(), translationVector.getY());
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationScaleMatrix.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector);
      rotationScaleMatrix.inverseTransform(pointTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      rotationScaleMatrix.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      rotationScaleMatrix.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      vectorTransformed.set(vectorOriginal);
      vectorTransformed.subX(vectorTransformed.getS() * translationVector.getX());
      vectorTransformed.subY(vectorTransformed.getS() * translationVector.getY());
      vectorTransformed.subZ(vectorTransformed.getS() * translationVector.getZ());
      rotationScaleMatrix.inverseTransform(vectorTransformed, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      rotationScaleMatrix.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      rotationScaleMatrix.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector.getX(), translationVector.getY());
      rotationScaleMatrix.inverseTransform(pointTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationScaleMatrix.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Packs the rotation and translation parts of this transform in the given rigid-body transform.
    *
    * @param rigidBodyTransformToPack the transform in which the rotation and translation parts of
    *           this affine transform are stored. Modified.
    */
   public void getRigidBodyTransform(RigidBodyTransform rigidBodyTransformToPack)
   {
      rigidBodyTransformToPack.setRotation(rotationScaleMatrix.getRotationMatrix());
      rigidBodyTransformToPack.setTranslation(translationVector);
   }

   /**
    * Packs this transform as a 4-by-4 matrix.
    *
    * <pre>
    *     / M(0, 0) M(0, 1) M(0, 2) Tx \
    * H = | M(1, 0) M(1, 1) M(1, 2) Ty |
    *     | M(2, 0) M(2, 1) M(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where M is the 3-by-3 rotation-scale matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param matrixToPack the matrix in which this transform is stored. Modified.
    */
   public void get(DenseMatrix64F matrixToPack)
   {
      rotationScaleMatrix.get(matrixToPack);
      translationVector.get(0, 3, matrixToPack);
      matrixToPack.set(3, 0, 0.0);
      matrixToPack.set(3, 1, 0.0);
      matrixToPack.set(3, 2, 0.0);
      matrixToPack.set(3, 3, 1.0);
   }

   /**
    * Packs this transform as a 4-by-4 matrix.
    *
    * <pre>
    *     / M(0, 0) M(0, 1) M(0, 2) Tx \
    * H = | M(1, 0) M(1, 1) M(1, 2) Ty |
    *     | M(2, 0) M(2, 1) M(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where M is the 3-by-3 rotation-scale matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param startRow the first row index to start writing in {@code matrixToPack}.
    * @param startColumn the first column index to start writing in {@code matrixToPack}.
    * @param matrixToPack the matrix in which this transform is stored. Modified.
    */
   public void get(int startRow, int startColumn, DenseMatrix64F matrixToPack)
   {
      rotationScaleMatrix.get(startRow, startColumn, matrixToPack);
      translationVector.get(startRow, startColumn + 3, matrixToPack);
      startRow += 3;
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn, 1.0);
   }

   /**
    * Packs this transform as a 4-by-4 matrix into a 1D row-major array.
    *
    * <pre>
    *     / M(0, 0) M(0, 1) M(0, 2) Tx \
    * H = | M(1, 0) M(1, 1) M(1, 2) Ty |
    *     | M(2, 0) M(2, 1) M(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where M is the 3-by-3 rotation-scale matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param transformArrayToPack the array in which this transform is stored. Modified.
    */
   public void get(double[] transformArrayToPack)
   {
      transformArrayToPack[0] = getM00();
      transformArrayToPack[1] = getM01();
      transformArrayToPack[2] = getM02();
      transformArrayToPack[3] = getM03();
      transformArrayToPack[4] = getM10();
      transformArrayToPack[5] = getM11();
      transformArrayToPack[6] = getM12();
      transformArrayToPack[7] = getM13();
      transformArrayToPack[8] = getM20();
      transformArrayToPack[9] = getM21();
      transformArrayToPack[10] = getM22();
      transformArrayToPack[11] = getM23();
      transformArrayToPack[12] = getM30();
      transformArrayToPack[13] = getM31();
      transformArrayToPack[14] = getM32();
      transformArrayToPack[15] = getM33();
   }

   /**
    * Packs the rotation-scale matrix and the translation vector of this affine transform.
    *
    * @param rotationScaleMarixToPack matrix in which the rotation-scale matrix of this affine
    *           transform is stored. Modified.
    * @param translationToPack tuple in which the translation vector of this affine transform is
    *           stored. Modified.
    */
   public void get(Matrix3DBasics rotationScaleMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationScaleMarixToPack.set(rotationScaleMatrix);
      translationToPack.set(translationVector);
   }

   /**
    * Packs the rotation-scale matrix and the translation vector of this affine transform.
    *
    * @param rotationScaleMarixToPack matrix in which the rotation-scale matrix of this affine
    *           transform is stored. Modified.
    * @param translationToPack tuple in which the translation vector of this affine transform is
    *           stored. Modified.
    */
   public void get(RotationScaleMatrix rotationScaleMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationScaleMarixToPack.set(rotationScaleMatrix);
      translationToPack.set(translationVector);
   }

   /**
    * Gets the read-only reference to the rotation part of this transform.
    *
    * @return the rotation part of this transform.
    */
   public RotationMatrixReadOnly getRotationMatrix()
   {
      return rotationScaleMatrix.getRotationMatrix();
   }

   /**
    * Packs the rotation part of this affine transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotation(Matrix3DBasics rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationScaleMatrix.getRotationMatrix());
   }

   /**
    * Packs the rotation part of this affine transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationScaleMatrix.getRotation(rotationMatrixToPack);
   }

   /**
    * Packs the rotation part of this affine transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotation(DenseMatrix64F rotationMatrixToPack)
   {
      rotationScaleMatrix.getRotation(rotationMatrixToPack);
   }

   /**
    * Packs the rotation part of this affine transform in 1D row-major array.
    *
    * @param rotationMatrixArrayToPack the array in which the rotation part of this transform is
    *           stored. Modified.
    */
   public void getRotation(double[] rotationMatrixArrayToPack)
   {
      rotationScaleMatrix.getRotation(rotationMatrixArrayToPack);
   }

   /**
    * Packs the rotation part of this affine transform as a quaternion.
    *
    * @param quaternionToPack the quaternion that is set to the rotation part of this transform.
    *           Modified.
    */
   public void getRotation(QuaternionBasics quaternionToPack)
   {
      rotationScaleMatrix.getRotation(quaternionToPack);
   }

   /**
    * Packs the rotation part of this affine transform as an axis-angle.
    *
    * @param axisAngleToPack the axis-angle that is set to the rotation part of this transform.
    *           Modified.
    */
   public void getRotation(AxisAngleBasics axisAngleToPack)
   {
      rotationScaleMatrix.getRotation(axisAngleToPack);
   }

   /**
    * Packs the rotation part of this affine transform as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector that is set to the rotation part of this
    *           transform. Modified.
    */
   public void getRotation(Vector3DBasics rotationVectorToPack)
   {
      rotationScaleMatrix.getRotation(rotationVectorToPack);
   }

   /**
    * Gets the read-only reference to the scale part of this transform.
    *
    * @return the scale part of this transform.
    */
   public Tuple3DReadOnly getScale()
   {
      return rotationScaleMatrix.getScale();
   }

   /**
    * Returns the current value of the x-axis scale factor of this affine transform.
    *
    * @return the value of the first scale factor.
    */
   public double getScaleX()
   {
      return rotationScaleMatrix.getScaleX();
   }

   /**
    * Returns the current value of the y-axis scale factor of this affine transform.
    *
    * @return the value of the second scale factor.
    */
   public double getScaleY()
   {
      return rotationScaleMatrix.getScaleY();
   }

   /**
    * Returns the current value of the z-axis scale factor of this affine transform.
    *
    * @return the value of the third scale factor.
    */
   public double getScaleZ()
   {
      return rotationScaleMatrix.getScaleZ();
   }

   public void getScale(Tuple3DBasics scaleToPack)
   {
      scaleToPack.set(rotationScaleMatrix.getScale());
   }

   /**
    * Gets the read-only reference to the rotation-scale part of this transform.
    *
    * @return the rotation-scale part of this transform.
    */
   public RotationScaleMatrixReadOnly getRotationScaleMatrix()
   {
      return rotationScaleMatrix;
   }

   /**
    * Packs the rotation part of this affine transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotationScale(Matrix3DBasics rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationScaleMatrix);
   }

   /**
    * Packs the rotation part of this affine transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotationScale(RotationScaleMatrix rotationScaleMatrixToPack)
   {
      rotationScaleMatrixToPack.set(rotationScaleMatrix);
   }

   /**
    * Packs the rotation part of this affine transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotationScale(DenseMatrix64F rotationScaleMatrixToPack)
   {
      rotationScaleMatrix.get(rotationScaleMatrixToPack);
   }

   /**
    * Gets the read-only reference of the translation part of this affine transform.
    *
    * @return the translation part of this transform.
    */
   public Vector3DReadOnly getTranslationVector()
   {
      return translationVector;
   }

   /**
    * Packs the translation part of this affine transform.
    *
    * @param translationToPack the tuple in which the translation part of this transform is stored.
    *           Modified.
    */
   public void getTranslation(Tuple3DBasics translationToPack)
   {
      translationToPack.set(translationVector);
   }

   /**
    * Retrieves and returns a coefficient of this transform given its row and column indices.
    *
    * @param row the row of the coefficient to return.
    * @param column the column of the coefficient to return.
    * @return the coefficient's value.
    * @throws ArrayIndexOutOfBoundsException if either {@code row} &notin; [0, 3] or {@code column}
    *            &notin; [0, 3].
    */
   public double getElement(int row, int column)
   {
      if (row < 3)
      {
         if (column < 3)
         {
            return rotationScaleMatrix.getElement(row, column);
         }
         else if (column < 4)
         {
            return translationVector.get(row);
         }
         else
         {
            throw Matrix3DTools.columnOutOfBoundsException(3, column);
         }
      }
      else if (row < 4)
      {
         if (column < 3)
         {
            return 0.0;
         }
         else if (column < 4)
         {
            return 1.0;
         }
         else
         {
            throw Matrix3DTools.columnOutOfBoundsException(3, column);
         }
      }
      else
      {
         throw Matrix3DTools.rowOutOfBoundsException(3, row);
      }
   }

   /**
    * Gets the 1st row 1st column coefficient of this transform.
    *
    * @return the 1st row 1st column coefficient.
    */
   public double getM00()
   {
      return rotationScaleMatrix.getM00();
   }

   /**
    * Gets the 1st row 2nd column coefficient of this transform.
    *
    * @return the 1st row 2nd column coefficient.
    */
   public double getM01()
   {
      return rotationScaleMatrix.getM01();
   }

   /**
    * Gets the 1st row 3rd column coefficient of this transform.
    *
    * @return the 1st row 3rd column coefficient.
    */
   public double getM02()
   {
      return rotationScaleMatrix.getM02();
   }

   /**
    * Gets the 1st row 4th column coefficient of this transform.
    *
    * @return the 1st row 4th column coefficient.
    */
   public double getM03()
   {
      return translationVector.getX();
   }

   /**
    * Gets the 2nd row 1st column coefficient of this transform.
    *
    * @return the 2nd row 1st column coefficient.
    */
   public double getM10()
   {
      return rotationScaleMatrix.getM10();
   }

   /**
    * Gets the 2nd row 2nd column coefficient of this transform.
    *
    * @return the 2nd row 2nd column coefficient.
    */
   public double getM11()
   {
      return rotationScaleMatrix.getM11();
   }

   /**
    * Gets the 2nd row 3rd column coefficient of this transform.
    *
    * @return the 2nd row 3rd column coefficient.
    */
   public double getM12()
   {
      return rotationScaleMatrix.getM12();
   }

   /**
    * Gets the 2nd row 4th column coefficient of this transform.
    *
    * @return the 2nd row 4th column coefficient.
    */
   public double getM13()
   {
      return translationVector.getY();
   }

   /**
    * Gets the 3rd row 1st column coefficient of this transform.
    *
    * @return the 3rd row 1st column coefficient.
    */
   public double getM20()
   {
      return rotationScaleMatrix.getM20();
   }

   /**
    * Gets the 3rd row 2nd column coefficient of this transform.
    *
    * @return the 3rd row 2nd column coefficient.
    */
   public double getM21()
   {
      return rotationScaleMatrix.getM21();
   }

   /**
    * Gets the 3rd row 3rd column coefficient of this transform.
    *
    * @return the 3rd row 3rd column coefficient.
    */
   public double getM22()
   {
      return rotationScaleMatrix.getM22();
   }

   /**
    * Gets the 3rd row 4th column coefficient of this transform.
    *
    * @return the 3rd row 4th column coefficient.
    */
   public double getM23()
   {
      return translationVector.getZ();
   }

   /**
    * Gets the 4th row 1st column coefficient of this transform.
    * <p>
    * Note: {@code m30 = 0.0}.
    * </p>
    *
    * @return the 4th row 1st column coefficient.
    */
   public double getM30()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 2nd column coefficient of this transform.
    * <p>
    * Note: {@code m31 = 0.0}.
    * </p>
    *
    * @return the 4th row 2nd column coefficient.
    */
   public double getM31()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 3rd column coefficient of this transform.
    * <p>
    * Note: {@code m32 = 0.0}.
    * </p>
    *
    * @return the 4th row 3rd column coefficient.
    */
   public double getM32()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 4th column coefficient of this transform.
    * <p>
    * Note: {@code m33 = 1.0}.
    * </p>
    *
    * @return the 4th row 4th column coefficient.
    */
   public double getM33()
   {
      return 1.0;
   }

   /**
    * Tests separately and on a per component basis if the rotation part, the scale part, and the
    * translation part of this transform and {@code other} are equal to an {@code epsilon}.
    *
    * @param other the other affine transform to compare against this. Not modified.
    */
   @Override
   public boolean epsilonEquals(AffineTransform other, double epsilon)
   {
      return rotationScaleMatrix.epsilonEquals(other.rotationScaleMatrix, epsilon) && translationVector.epsilonEquals(other.translationVector, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(AffineTransform)}, it returns {@code false} otherwise or if the
    * {@code object} is {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((AffineTransform) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests separately and on a per component basis if the rotation part, the scale part, and the
    * translation part of this transform and {@code other} are exactly equal.
    * <p>
    * The method returns {@code false} if the given transform is {@code null}.
    * </p>
    *
    * @param other the other transform to compare against this. Not modified.
    * @return {@code true} if the two transforms are exactly equal, {@code false} otherwise.
    */
   public boolean equals(AffineTransform other)
   {
      if (other == null)
         return false;
      else
         return rotationScaleMatrix.equals(other.rotationScaleMatrix) && translationVector.equals(other.translationVector);
   }

   /**
    * Provides a {@code String} representation of this transform as follows: <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this transform.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getAffineTransformString(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 31L * rotationScaleMatrix.hashCode() + translationVector.hashCode();
      return (int) (bits ^ bits >> 32);
   }
}

package us.ihmc.euclid.transform;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
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
 * A {@code RigidBodyTransform} represents a 4-by-4 transformation matrix that can rotate and
 * translate.
 * <p>
 * For efficiency and readability, the transform is never stored in a 4-by-4 matrix.
 * </p>
 * <p>
 * The {@code RigidBodyTransform} is composed of {@link RotationMatrix} to rotate, and a
 * {@link Vector3D} to translate.
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
 * is, in order, rotated and then translated.
 * <li>when applying this transform on a {@link Vector3DBasics} or {@link Vector2DBasics}, this
 * object is only rotated. It is NOT translated.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class RigidBodyTransform
      implements Transform, EpsilonComparable<RigidBodyTransform>, GeometricallyComparable<RigidBodyTransform>, Settable<RigidBodyTransform>, Clearable
{
   /** The rotation part of this transform. */
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   /** The translation part of this transform. */
   private final Vector3D translationVector = new Vector3D();

   /**
    * Creates a new rigid-body transform set to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public RigidBodyTransform()
   {
   }

   /**
    * Creates a new rigid-body transform and sets it to {@code other}.
    *
    * @param other the other rigid-body transform to copy. Not modified.
    */
   public RigidBodyTransform(RigidBodyTransform other)
   {
      set(other);
   }

   /**
    * Creates a new rigid-body transform and sets it to {@code quaternionBasedTransform}.
    *
    * @param quaternionBasedTransform the {@link QuaternionBasedTransform} to initialize to.
    */
   public RigidBodyTransform(QuaternionBasedTransform quaternionBasedTransform)
   {
      set(quaternionBasedTransform);
   }

   /**
    * Creates a new rigid-body transform and sets its raw components from the given {@code matrix}.
    * <p>
    * The rotation part R is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 0) matrix.get(0, 1) matrix.get(0, 2) \
    * R = | matrix.get(1, 0) matrix.get(1, 1) matrix.get(1, 2) |
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
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *            transform is not a rotation matrix.
    */
   public RigidBodyTransform(DenseMatrix64F matrix)
   {
      set(matrix);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[1] transformArray[ 2] \
    * R = | transformArray[4] transformArray[5] transformArray[ 6] |
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
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *            transform is not a rotation matrix.
    */
   public RigidBodyTransform(double[] transformArray)
   {
      set(transformArray);
   }

   /**
    * Creates a new rigid-body transform and sets it to the given {@code rotationMatrix} and
    * {@code translation}.
    *
    * @param rotationMatrix the rotation matrix used to set this transform's rotation part. Not
    *           modified.
    * @param translation the tuple used to set this transform's translation part. Not modified.
    */
   public RigidBodyTransform(RotationMatrix rotationMatrix, Tuple3DReadOnly translation)
   {
      set(rotationMatrix, translation);
   }

   /**
    * Creates a new rigid-body transform and sets it to the given {@code quaternion} and
    * {@code translation}.
    *
    * @param quaternion the quaternion used to set this transform's rotation part. Not modified.
    * @param translation the tuple used to set this transform's translation part. Not modified.
    */
   public RigidBodyTransform(QuaternionReadOnly quaternion, Tuple3DReadOnly translation)
   {
      set(quaternion, translation);
   }

   /**
    * Creates a new rigid-body transform and sets it to the given {@code axisAngle} and
    * {@code translation}.
    *
    * @param axisAngle the axis-angle used to set this transform's rotation part. Not modified.
    * @param translation the tuple used to set this transform's translation part. Not modified.
    */
   public RigidBodyTransform(AxisAngleReadOnly axisAngle, Tuple3DReadOnly translation)
   {
      set(axisAngle, translation);
   }

   /**
    * Creates a new rigid-body transform and sets it from the given 12 coefficients.
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m03 the x-component of the translation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m13 the y-component of the translation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @param m23 the z-component of the translation part of this transform.
    * @throws NotARotationMatrixException if the components for the rotation part do not represent a
    *            rotation matrix.
    */
   public RigidBodyTransform(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                             double m23)
   {
      set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
   }

   /**
    * Resets this rigid-body transform to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public void setIdentity()
   {
      rotationMatrix.setIdentity();
      translationVector.setToZero();
   }

   /**
    * Resets this rigid-body transform to identity.
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
    */
   public void setRotationToZero()
   {
      rotationMatrix.setIdentity();
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
      rotationMatrix.setToNaN();
      translationVector.setToNaN();
   }

   /**
    * Sets all the components of the rotation matrix to {@link Double#NaN}.
    * <p>
    * See {@link RotationScaleMatrix#setToNaN()}.
    * </p>
    */
   public void setRotationToNaN()
   {
      rotationMatrix.setToNaN();
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
      return rotationMatrix.containsNaN() || translationVector.containsNaN();
   }

   /**
    * Tests if the rotation part of this transform describes a transformation in the XY plane.
    * <p>
    * The rotation part is considered to be a 2D transformation in the XY plane if:
    * <ul>
    * <li>the last diagonal coefficient m22 is equal to 1.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_2D},
    * <li>the coefficients {@code m20}, {@code m02}, {@code m21}, and {@code m12} are equal to 0.0
    * +/- {@link Matrix3DFeatures#EPS_CHECK_2D}.
    * </ul>
    * </p>
    *
    * @return {@code true} if the rotation part describes a 2D transformation in the XY plane,
    *         {@code false} otherwise.
    */
   public boolean isRotation2D()
   {
      return rotationMatrix.isMatrix2D();
   }

   /**
    * Asserts that the rotation part of this transform describes a transformation in the XY plane.
    * <p>
    * The rotation part is considered to be a 2D transformation in the XY plane if:
    * <ul>
    * <li>the last diagonal coefficient m22 is equal to 1.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_2D},
    * <li>the coefficients {@code m20}, {@code m02}, {@code m21}, and {@code m12} are equal to 0.0
    * +/- {@link Matrix3DFeatures#EPS_CHECK_2D}.
    * </ul>
    * </p>
    *
    * @throws NotAMatrix2DException if the rotation part represents a 3D transformation.
    */
   public void checkIfRotation2D()
   {
      rotationMatrix.checkIfMatrix2D();
   }

   /**
    * Normalize the rotation part of this transform.
    */
   public void normalizeRotationPart()
   {
      rotationMatrix.normalize();
   }

   /**
    * Computes the determinant of the rotation part of this transform.
    *
    * @return the determinant's value.
    */
   public double determinantRotationPart()
   {
      return rotationMatrix.determinant();
   }

   /**
    * Sets this rigid-body transform from the given 12 coefficients.
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m03 the x-component of the translation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m13 the y-component of the translation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @param m23 the z-component of the translation part of this transform.
    * @throws NotARotationMatrixException if the components for the rotation part do not represent a
    *            rotation matrix.
    */
   public void set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                   double m23)
   {
      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   /**
    * Sets this rigid-body transform from the given 12 coefficients.
    * <p>
    * Prefer using the method
    * {@link #set(double, double, double, double, double, double, double, double, double, double, double, double)}
    * as it asserts that the coefficients for the rotation part represent a rotation matrix.
    * </p>
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m03 the x-component of the translation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m13 the y-component of the translation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @param m23 the z-component of the translation part of this transform.
    */
   public void setUnsafe(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                         double m23)
   {
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   /**
    * Sets this rigid-body transform to {@code other}.
    *
    * @param other the other rigid-body transform to copy the values from. Not modified.
    */
   @Override
   public void set(RigidBodyTransform other)
   {
      rotationMatrix.set(other.rotationMatrix);
      translationVector.set(other.translationVector);
   }

   /**
    * Sets this rigid-body transform to {@code other} and then inverts it.
    *
    * @param other the other rigid-body transform to copy the values from. Not modified.
    */
   public void setAndInvert(RigidBodyTransform other)
   {
      set(other);
      invert();
   }

   /**
    * Sets this rigid-body transform to the given {@code quaternionBasedTransform}.
    *
    * @param quaternionBasedTransform the quaternion-based transform to copy the values from. Not
    *           modified.
    */
   public void set(QuaternionBasedTransform quaternionBasedTransform)
   {
      rotationMatrix.set(quaternionBasedTransform.getQuaternion());
      translationVector.set(quaternionBasedTransform.getTranslationVector());
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code matrix}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 0) matrix.get(0, 1) matrix.get(0, 2) \
    * R = | matrix.get(1, 0) matrix.get(1, 1) matrix.get(1, 2) |
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
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *            transform is not a rotation matrix.
    */
   public void set(DenseMatrix64F matrix)
   {
      rotationMatrix.set(matrix);
      translationVector.set(0, 3, matrix);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code matrix}.
    * <p>
    * The rotation part R is set as follows:
    *
    * <pre>
    *     / matrix.get(startRow + 0, startColumn + 0) matrix.get(startRow + 0, startColumn + 1) matrix.get(startRow + 0, startColumn + 2) \
    * R = | matrix.get(startRow + 1, startColumn + 0) matrix.get(startRow + 1, startColumn + 1) matrix.get(startRow + 1, startColumn + 2) |
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
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *            transform is not a rotation matrix.
    */
   public void set(DenseMatrix64F matrix, int startRow, int startColumn)
   {
      rotationMatrix.set(startRow, startColumn, matrix);
      translationVector.set(startRow, startColumn + 3, matrix);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[1] transformArray[ 2] \
    * R = | transformArray[4] transformArray[5] transformArray[ 6] |
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
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *            transform is not a rotation matrix.
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

      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[4] transformArray[ 8] \
    * R = | transformArray[1] transformArray[5] transformArray[ 9] |
    *     \ transformArray[2] transformArray[6] transformArray[10] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / transformArray[12] \
    * T = | transformArray[13] |
    *     \ transformArray[14] /
    * </pre>
    * </p>
    *
    * @param transformArray the 1D column-major array to get this transform's components from. Not
    *           modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *            transform is not a rotation matrix.
    */
   public void setAsTranspose(double[] transformArray)
   {
      double m00 = transformArray[0];
      double m01 = transformArray[4];
      double m02 = transformArray[8];
      double m03 = transformArray[12];
      double m10 = transformArray[1];
      double m11 = transformArray[5];
      double m12 = transformArray[9];
      double m13 = transformArray[13];
      double m20 = transformArray[2];
      double m21 = transformArray[6];
      double m22 = transformArray[10];
      double m23 = transformArray[14];

      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[1] transformArray[ 2] \
    * R = | transformArray[4] transformArray[5] transformArray[ 6] |
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
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *            transform is not a rotation matrix.
    */
   public void set(float[] transformArray)
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

      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[4] transformArray[ 8] \
    * R = | transformArray[1] transformArray[5] transformArray[ 9] |
    *     \ transformArray[2] transformArray[6] transformArray[10] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / transformArray[12] \
    * T = | transformArray[13] |
    *     \ transformArray[14] /
    * </pre>
    * </p>
    *
    * @param transformArray the 1D column-major array to get this transform's components from. Not
    *           modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *            transform is not a rotation matrix.
    */
   public void setAsTranspose(float[] transformArray)
   {
      double m00 = transformArray[0];
      double m01 = transformArray[4];
      double m02 = transformArray[8];
      double m03 = transformArray[12];
      double m10 = transformArray[1];
      double m11 = transformArray[5];
      double m12 = transformArray[9];
      double m13 = transformArray[13];
      double m20 = transformArray[2];
      double m21 = transformArray[6];
      double m22 = transformArray[10];
      double m23 = transformArray[14];

      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   /**
    * Sets the rotation and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void set(Matrix3DReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      this.rotationMatrix.set(rotationMatrix);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    */
   public void set(RotationMatrix rotationMatrix, Tuple3DReadOnly translation)
   {
      this.rotationMatrix.set(rotationMatrix);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation and translation parts of this transform separately.
    * <p>
    * Only the rotation matrix from {@code rotationScaleMatrix} is used to set the rotation part of
    * this transform.
    * </p>
    *
    * @param rotationScaleMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    */
   public void set(RotationScaleMatrix rotationScaleMatrix, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.getRotation(rotationMatrix);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation and translation parts of this transform separately.
    *
    * @param axisAngle the axis-angle used to set the rotation part of this transform. Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    */
   public void set(AxisAngleReadOnly axisAngle, Tuple3DReadOnly translation)
   {
      rotationMatrix.set(axisAngle);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation and translation parts of this transform separately.
    *
    * @param quaternion the quaternion used to set the rotation part of this transform. Not
    *           modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    */
   public void set(QuaternionReadOnly quaternion, Tuple3DReadOnly translation)
   {
      rotationMatrix.set(quaternion);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation part of this transform from the given 9 coefficients.
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @throws NotARotationMatrixException if the resulting matrix does not represent a rotation
    *            matrix.
    */
   public void setRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Sets the rotation part of this transform from the given 9 coefficients.
    * <p>
    * Prefer using the method
    * {@link #setRotation(double, double, double, double, double, double, double, double, double)}
    * as it asserts that the coefficients 0represent a rotation matrix.
    * </p>
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    */
   public void setRotationUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Sets the rotation part of this transform to the given axis-angle.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param axisAngle the axis-angle used to set the rotation part of this transform. Not modified.
    */
   public void setRotation(AxisAngleReadOnly axisAngle)
   {
      rotationMatrix.set(axisAngle);
   }

   /**
    * Sets the rotation part of this transform to the given rotation vector.
    * <p>
    * This method does not affect the translation part of this transform.
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
      rotationMatrix.set(rotationVector);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotation(DenseMatrix64F rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to the given quaternion.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param quaternion the quaternion used to set the rotation part of this transform. Not
    *           modified.
    */
   public void setRotation(QuaternionReadOnly quaternion)
   {
      rotationMatrix.set(quaternion);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotation(Matrix3DReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotation(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
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
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setRotationYaw(double yaw)
   {
      rotationMatrix.setToYawMatrix(yaw);
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
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setRotationPitch(double pitch)
   {
      rotationMatrix.setToPitchMatrix(pitch);
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
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationRoll(double roll)
   {
      rotationMatrix.setToRollMatrix(roll);
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
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    */
   public void setRotationYawPitchRoll(double[] yawPitchRoll)
   {
      rotationMatrix.setYawPitchRoll(yawPitchRoll);
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
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      rotationMatrix.setYawPitchRoll(yaw, pitch, roll);
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
    * This method does not affect the translation part of this transform.
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
      rotationMatrix.setEuler(eulerAngles);
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
    * This method does not affect the translation part of this transform.
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
      rotationMatrix.setEuler(rotX, rotY, rotZ);
   }

   /**
    * Sets the rotation part of this transform to the given axis-angle and sets the translation part
    * to zero.
    *
    * @param axisAngle the axis-angle used to set the rotation part of this transform. Not modified.
    */
   public void setRotationAndZeroTranslation(AxisAngleReadOnly axisAngle)
   {
      setRotation(axisAngle);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to the given rotation vector and sets the translation
    * part to zero.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to set the rotation part of this transform. Not
    *           modified.
    */
   public void setRotationAndZeroTranslation(Vector3DReadOnly rotationVector)
   {
      rotationMatrix.set(rotationVector);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to the given matrix and sets the translation part to
    * zero.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotationAndZeroTranslation(DenseMatrix64F matrix)
   {
      setRotation(matrix);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to the given quaternion and sets the translation part
    * to zero.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param quaternion the quaternion used to set the rotation part of this transform. Not
    *           modified.
    */
   public void setRotationAndZeroTranslation(QuaternionReadOnly quaternion)
   {
      setRotation(quaternion);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to the given matrix and sets the translation part to
    * zero.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not
    *           modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotationAndZeroTranslation(Matrix3DReadOnly rotationMatrix)
   {
      setRotation(rotationMatrix);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * z-axis of an angle {@code yaw} and sets the translation part to zero.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \
    * R = | sin(yaw)  cos(yaw) 0 |
    *     \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setRotationYawAndZeroTranslation(double yaw)
   {
      setRotationYaw(yaw);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * y-axis of an angle {@code pitch} and sets the translation part to zero.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setRotationPitchAndZeroTranslation(double pitch)
   {
      setRotationPitch(pitch);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * x-axis of an angle {@code roll} and sets the translation part to zero.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationRollAndZeroTranslation(double roll)
   {
      setRotationRoll(roll);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given
    * yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll} and sets the translation
    * part to zero.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    */
   public void setRotationYawPitchRollAndZeroTranslation(double[] yawPitchRoll)
   {
      setRotationYawPitchRoll(yawPitchRoll);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given
    * yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll} and sets the translation
    * part to zero.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationYawPitchRollAndZeroTranslation(double yaw, double pitch, double roll)
   {
      setRotationYawPitchRoll(yaw, pitch, roll);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code eulerAngles} and sets the translation part to zero.
    *
    * <pre>
    *     / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * R = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *     \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This is equivalent to
    * {@code this.setRotationYawPitchRollAndZeroTranslation(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   public void setRotationEulerAndZeroTranslation(Vector3DReadOnly eulerAngles)
   {
      setRotationEuler(eulerAngles);
      translationVector.setToZero();
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code rotX}, {@code rotY}, and {@code rotZ} and sets the translation part to zero.
    *
    * <pre>
    *     / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * R = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *     \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
    * <p>
    * This is equivalent to
    * {@code this.setRotationYawPitchRollAndZeroTranslation(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   public void setRotationEulerAndZeroTranslation(double rotX, double rotY, double rotZ)
   {
      setRotationEuler(rotX, rotY, rotZ);
      translationVector.setToZero();
   }

   /**
    * Sets the x-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the x-component of the translation part.
    */
   public void setTranslationX(double x)
   {
      translationVector.setX(x);
   }

   /**
    * Sets the y-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param y the y-component of the translation part.
    */
   public void setTranslationY(double y)
   {
      translationVector.setY(y);
   }

   /**
    * Sets the z-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param z the z-component of the translation part.
    */
   public void setTranslationZ(double z)
   {
      translationVector.setZ(z);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
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
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation tuple used to set the translation part of this transform. Not modified.
    */
   public void setTranslation(Tuple3DReadOnly translation)
   {
      translationVector.set(translation);
   }

   /**
    * Sets the translation part of this transform and sets the rotation part to identity.
    *
    * @param x the x-component of the translation part.
    * @param y the y-component of the translation part.
    * @param z the z-component of the translation part.
    */
   public void setTranslationAndIdentityRotation(double x, double y, double z)
   {
      setTranslation(x, y, z);
      rotationMatrix.setIdentity();
   }

   /**
    * Sets the translation part of this transform and sets the rotation part to identity.
    *
    * @param translation tuple used to set the translation part of this transform. Not modified.
    */
   public void setTranslationAndIdentityRotation(Tuple3DReadOnly translation)
   {
      setTranslation(translation);
      rotationMatrix.setIdentity();
   }

   /**
    * Inverts this rigid-body transform.
    */
   public void invert()
   {
      rotationMatrix.invert();
      rotationMatrix.transform(translationVector);
      translationVector.negate();
   }

   /**
    * Inverts only the rotation part of this transform, the translation remains unchanged.
    */
   public void invertRotation()
   {
      rotationMatrix.invert();
   }

   /**
    * Performs the multiplication of this transform with {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void multiply(RigidBodyTransform other)
   {
      Matrix3DTools.addTransform(rotationMatrix, other.translationVector, translationVector);
      rotationMatrix.multiply(other.rotationMatrix);
   }

   /**
    * Performs the multiplication of this transform with {@code quaternionBasedTransform}.
    * <p>
    * this = this * H(quaternionBasedTransform) <br>
    * where H(q) is the function converting a quaternion-based transform into a 4-by-4
    * transformation matrix.
    * </p>
    *
    * @param quaternionBasedTransform the quaternion-based transform to multiply this with. Not
    *           modified.
    */
   public void multiply(QuaternionBasedTransform quaternionBasedTransform)
   {
      Matrix3DTools.addTransform(rotationMatrix, quaternionBasedTransform.getTranslationVector(), translationVector);
      rotationMatrix.multiply(quaternionBasedTransform.getQuaternion());
   }

   /**
    * Performs the multiplication of this transform with {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the
    * multiplication to conserve a proper rigid-body transform describing only a rotation and a
    * translation.
    * </p>
    * <p>
    * this = this * S(affineTransform) <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of
    * the affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void multiply(AffineTransform affineTransform)
   {
      Matrix3DTools.addTransform(rotationMatrix, affineTransform.getTranslationVector(), translationVector);
      rotationMatrix.multiply(affineTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of the inverse of this transform with {@code other}.
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void multiplyInvertThis(RigidBodyTransform other)
   {
      translationVector.sub(other.getTranslationVector(), translationVector);
      rotationMatrix.inverseTransform(translationVector, translationVector);
      rotationMatrix.inverseTransform(other.rotationMatrix, rotationMatrix);
   }

   /**
    * Performs the multiplication of this transform with the inverse of {@code other}.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void multiplyInvertOther(RigidBodyTransform other)
   {
      rotationMatrix.multiplyTransposeOther(other.getRotationMatrix());
      Matrix3DTools.subTransform(rotationMatrix, other.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of this transform with
    * {@code quaternionBasedTransform}.
    * <p>
    * this = this<sup>-1</sup> * H(quaternionBasedTransform) <br>
    * where H(q) is the function converting a quaternion-based transform into a 4-by-4
    * transformation matrix.
    * </p>
    *
    * @param quaternionBasedTransform the quaternion-based transform to multiply this with. Not
    *           modified.
    */
   public void multiplyInvertThis(QuaternionBasedTransform quaternionBasedTransform)
   {
      translationVector.sub(quaternionBasedTransform.getTranslationVector(), translationVector);
      rotationMatrix.inverseTransform(translationVector, translationVector);
      rotationMatrix.preMultiplyInvertThis(quaternionBasedTransform.getQuaternion());
   }

   /**
    * Performs the multiplication of this transform with the inverse of
    * {@code quaternionBasedTransform}.
    * <p>
    * this = this * H(quaternionBasedTransform)<sup>-1</sup> <br>
    * where H(q) is the function converting a quaternion-based transform into a 4-by-4
    * transformation matrix.
    * </p>
    *
    * @param quaternionBasedTransform the quaternion-based transform to multiply this with. Not
    *           modified.
    */
   public void multiplyInvertOther(QuaternionBasedTransform quaternionBasedTransform)
   {
      rotationMatrix.preMultiplyInvertOther(quaternionBasedTransform.getQuaternion());
      Matrix3DTools.subTransform(rotationMatrix, quaternionBasedTransform.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of this transform with {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the
    * multiplication to conserve a proper rigid-body transform describing only a rotation and a
    * translation.
    * </p>
    * <p>
    * this = this<sup>-1</sup> * S(affineTransform) <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of
    * the affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void multiplyInvertThis(AffineTransform affineTransform)
   {
      translationVector.sub(affineTransform.getTranslationVector(), translationVector);
      rotationMatrix.inverseTransform(translationVector, translationVector);
      rotationMatrix.inverseTransform(affineTransform.getRotationMatrix(), rotationMatrix);
   }

   /**
    * Performs the multiplication of this transform with the inverse of {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the
    * multiplication to conserve a proper rigid-body transform describing only a rotation and a
    * translation.
    * </p>
    * <p>
    * this = this * S(affineTransform)<sup>-1</sup> <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of
    * the affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void multiplyInvertOther(AffineTransform affineTransform)
   {
      rotationMatrix.multiplyTransposeOther(affineTransform.getRotationMatrix());
      Matrix3DTools.subTransform(rotationMatrix, affineTransform.getTranslationVector(), translationVector);
   }

   /**
    * Append a translation transform to this transform.
    *
    * <pre>
    *               / 1 0 0 translation.x \
    * this = this * | 0 1 0 translation.y |
    *               | 0 0 1 translation.z |
    *               \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation the translation to append to this transform. Not modified.
    */
   public void appendTranslation(Tuple3DReadOnly translation)
   {
      rotationMatrix.addTransform(translation, translationVector);
   }

   /**
    * Append a translation transform to this transform.
    *
    * <pre>
    *               / 1 0 0 x \
    * this = this * | 0 1 0 y |
    *               | 0 0 1 z |
    *               \ 0 0 0 1 /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the translation along the x-axis to apply.
    * @param y the translation along the y-axis to apply.
    * @param z the translation along the z-axis to apply.
    */
   public void appendTranslation(double x, double y, double z)
   {
      double thisX = translationVector.getX();
      double thisY = translationVector.getY();
      double thisZ = translationVector.getZ();

      translationVector.set(x, y, z);
      rotationMatrix.transform(translationVector);
      translationVector.add(thisX, thisY, thisZ);
   }

   /**
    * Append a rotation about the z-axis to the rotation part of this transform.
    *
    * <pre>
    *         / cos(yaw) -sin(yaw) 0 \
    * R = R * | sin(yaw)  cos(yaw) 0 |
    *         \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void appendYawRotation(double yaw)
   {
      rotationMatrix.appendYawRotation(yaw);
   }

   /**
    * Append a rotation about the y-axis to the rotation part of this transform.
    *
    * <pre>
    *         /  cos(pitch) 0 sin(pitch) \
    * R = R * |      0      1     0      |
    *         \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void appendPitchRotation(double pitch)
   {
      rotationMatrix.appendPitchRotation(pitch);
   }

   /**
    * Append a rotation about the x-axis to the rotation part of this transform.
    *
    * <pre>
    *         /  cos(pitch) 0 sin(pitch) \
    * R = R * |      0      1     0      |
    *         \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the x-axis.
    */
   public void appendRollRotation(double roll)
   {
      rotationMatrix.appendRollRotation(roll);
   }

   /**
    * Performs the multiplication of {@code other} with this transform.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void preMultiply(RigidBodyTransform other)
   {
      other.rotationMatrix.transform(translationVector);
      translationVector.add(other.translationVector);
      rotationMatrix.preMultiply(other.rotationMatrix);
   }

   /**
    * Performs the multiplication of {@code quaternionBasedTransform} with this transform.
    * <p>
    * this = H(quaternionBasedTransform) * this <br>
    * where H(q) is the function converting a quaternion-based transform into a 4-by-4
    * transformation matrix.
    * </p>
    *
    * @param quaternionBasedTransform the quaternion-based transform to multiply this with. Not
    *           modified.
    */
   public void preMultiply(QuaternionBasedTransform quaternionBasedTransform)
   {
      quaternionBasedTransform.getQuaternion().transform(translationVector);
      translationVector.add(quaternionBasedTransform.getTranslationVector());
      rotationMatrix.preMultiply(quaternionBasedTransform.getQuaternion());
   }

   /**
    * Performs the multiplication of {@code affineTransform} with this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the
    * multiplication to conserve a proper rigid-body transform describing only a rotation and a
    * translation.
    * </p>
    * <p>
    * this = S(affineTransform) * this <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of
    * the affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void preMultiply(AffineTransform affineTransform)
   {
      affineTransform.getRotationMatrix().transform(translationVector);
      translationVector.add(affineTransform.getTranslationVector());
      rotationMatrix.preMultiply(affineTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of {@code other} with the inverse of this transform.
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertThis(RigidBodyTransform other)
   {
      rotationMatrix.preMultiplyTransposeThis(other.getRotationMatrix());
      rotationMatrix.transform(translationVector);
      translationVector.sub(other.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of {@code other} with this transform.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertOther(RigidBodyTransform other)
   {
      translationVector.sub(other.getTranslationVector());
      other.getRotationMatrix().inverseTransform(translationVector);
      rotationMatrix.preMultiplyTransposeOther(other.getRotationMatrix());
   }

   /**
    * Performs the multiplication of {@code quaternionBasedTransform} with the inverse of this
    * transform.
    * <p>
    * this = H(quaternionBasedTransform) * this<sup>-1</sup> <br>
    * where H(q) is the function converting a quaternion-based transform into a 4-by-4
    * transformation matrix.
    * </p>
    *
    * @param quaternionBasedTransform the quaternion-based transform to multiply this with. Not
    *           modified.
    */
   public void preMultiplyInvertThis(QuaternionBasedTransform quaternionBasedTransform)
   {
      rotationMatrix.preMultiplyInvertThis(quaternionBasedTransform.getQuaternion());
      rotationMatrix.transform(translationVector);
      translationVector.sub(quaternionBasedTransform.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of {@code quaternionBasedTransform} with this
    * transform.
    * <p>
    * this = H(quaternionBasedTransform)<sup>-1</sup> * this <br>
    * where H(q) is the function converting a quaternion-based transform into a 4-by-4
    * transformation matrix.
    * </p>
    *
    * @param quaternionBasedTransform the quaternion-based transform to multiply this with. Not
    *           modified.
    */
   public void preMultiplyInvertOther(QuaternionBasedTransform quaternionBasedTransform)
   {
      translationVector.sub(quaternionBasedTransform.getTranslationVector());
      quaternionBasedTransform.getQuaternion().inverseTransform(translationVector);
      rotationMatrix.preMultiplyInvertOther(quaternionBasedTransform.getQuaternion());
   }

   /**
    * Performs the multiplication of {@code affineTransform} with the inverse of this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the
    * multiplication to conserve a proper rigid-body transform describing only a rotation and a
    * translation.
    * </p>
    * <p>
    * this = S(affineTransform) * this<sup>-1</sup> <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of
    * the affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertThis(AffineTransform affineTransform)
   {
      rotationMatrix.preMultiplyTransposeThis(affineTransform.getRotationMatrix());
      rotationMatrix.transform(translationVector);
      translationVector.sub(affineTransform.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of {@code affineTransform} with this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the
    * multiplication to conserve a proper rigid-body transform describing only a rotation and a
    * translation.
    * </p>
    * <p>
    * this = S(affineTransform)<sup>-1</sup> * this <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of
    * the affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertOther(AffineTransform affineTransform)
   {
      translationVector.sub(affineTransform.getTranslationVector());
      affineTransform.getRotationMatrix().inverseTransform(translationVector);
      rotationMatrix.preMultiplyTransposeOther(affineTransform.getRotationMatrix());
   }

   /**
    * Prepend a translation transform to this transform.
    *
    * <pre>
    *        / 1 0 0 translation.x \
    * this = | 0 1 0 translation.y | * this
    *        | 0 0 1 translation.z |
    *        \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation the translation to prepend to this transform. Not modified.
    */
   public void prependTranslation(Tuple3DReadOnly translation)
   {
      translationVector.add(translation);
   }

   /**
    * Prepend a translation transform to this transform.
    *
    * <pre>
    *        / 1 0 0 x \
    * this = | 0 1 0 y | * this
    *        | 0 0 1 z |
    *        \ 0 0 0 1 /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the translation along the x-axis to apply.
    * @param y the translation along the y-axis to apply.
    * @param z the translation along the z-axis to apply.
    */
   public void prependTranslation(double x, double y, double z)
   {
      translationVector.add(x, y, z);
   }

   /**
    * Prepend a rotation about the z-axis to this transform.
    * <p>
    * This method first rotates the translation part and then prepend the yaw-rotation to the
    * rotation part of this transform.
    * </p>
    *
    * <pre>
    *        / cos(yaw) -sin(yaw)  0   0 \
    * this = | sin(yaw)  cos(yaw)  0   0 | * this
    *        |    0         0      1   0 |
    *        \    0         0      0   1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, translationVector, translationVector);
      rotationMatrix.prependYawRotation(yaw);
   }

   /**
    * Prepend a rotation about the y-axis to this transform.
    * <p>
    * This method first rotates the translation part and then prepend the pitch-rotation to the
    * rotation part of this transform.
    * </p>
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch)  0 \
    * this = |      0      1     0       0 | * this
    *        | -sin(pitch) 0 cos(pitch)  0 |
    *        \      0      0     0       1 /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, translationVector, translationVector);
      rotationMatrix.prependPitchRotation(pitch);
   }

   /**
    * Prepend a rotation about the x-axis to this transform.
    * <p>
    * This method first rotates the translation part and then prepend the roll-rotation to the
    * rotation part of this transform.
    * </p>
    *
    * <pre>
    *        / 1     0          0     0 \
    * this = | 0 cos(roll) -sin(roll) 0 | * this
    *        | 0 sin(roll)  cos(roll) 0 |
    *        \ 0     0          0     1 /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, translationVector, translationVector);
      rotationMatrix.prependRollRotation(roll);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      rotationMatrix.transform(pointOriginal, pointTransformed);
      pointTransformed.add(translationVector);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      rotationMatrix.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      rotationMatrix.transform(quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      rotationMatrix.transform(vectorOriginal, vectorTransformed);
      vectorTransformed.addX(vectorTransformed.getS() * translationVector.getX());
      vectorTransformed.addY(vectorTransformed.getS() * translationVector.getY());
      vectorTransformed.addZ(vectorTransformed.getS() * translationVector.getZ());
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationMatrix.transform(point2DOriginal, point2DTransformed, checkIfTransformInXYPlane);
      point2DTransformed.add(translationVector.getX(), translationVector.getY());
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationMatrix.transform(vector2DOriginal, vector2DTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      rotationMatrix.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      rotationMatrix.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(AffineTransform original, AffineTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector);
      rotationMatrix.inverseTransform(pointTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      rotationMatrix.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      rotationMatrix.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      vectorTransformed.set(vectorOriginal);
      vectorTransformed.subX(vectorTransformed.getS() * translationVector.getX());
      vectorTransformed.subY(vectorTransformed.getS() * translationVector.getY());
      vectorTransformed.subZ(vectorTransformed.getS() * translationVector.getZ());
      rotationMatrix.inverseTransform(vectorTransformed, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector.getX(), translationVector.getY());
      rotationMatrix.inverseTransform(pointTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationMatrix.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      rotationMatrix.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      rotationMatrix.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiplyInvertOther(this);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiplyInvertOther(this);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(AffineTransform original, AffineTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiplyInvertOther(this);
   }

   /**
    * Packs this transform as a 4-by-4 matrix.
    *
    * <pre>
    *     / R(0, 0) R(0, 1) R(0, 2) Tx \
    * H = | R(1, 0) R(1, 1) R(1, 2) Ty |
    *     | R(2, 0) R(2, 1) R(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where R is the 3-by-3 rotation matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param matrixToPack the matrix in which this transform is stored. Modified.
    */
   public void get(DenseMatrix64F matrixToPack)
   {
      rotationMatrix.get(matrixToPack);
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
    *     / R(0, 0) R(0, 1) R(0, 2) Tx \
    * H = | R(1, 0) R(1, 1) R(1, 2) Ty |
    *     | R(2, 0) R(2, 1) R(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where R is the 3-by-3 rotation matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param startRow the first row index to start writing in {@code matrixToPack}.
    * @param startColumn the first column index to start writing in {@code matrixToPack}.
    * @param matrixToPack the matrix in which this transform is stored. Modified.
    */
   public void get(int startRow, int startColumn, DenseMatrix64F matrixToPack)
   {
      rotationMatrix.get(startRow, startColumn, matrixToPack);
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
    *     / R(0, 0) R(0, 1) R(0, 2) Tx \
    * H = | R(1, 0) R(1, 1) R(1, 2) Ty |
    *     | R(2, 0) R(2, 1) R(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where R is the 3-by-3 rotation matrix and (Tx, Ty, Tz) is the translation part of this
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
    * Packs this transform as a 4-by-4 matrix into a 1D row-major array.
    *
    * <pre>
    *     / R(0, 0) R(0, 1) R(0, 2) Tx \
    * H = | R(1, 0) R(1, 1) R(1, 2) Ty |
    *     | R(2, 0) R(2, 1) R(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where R is the 3-by-3 rotation matrix and (Tx, Ty, Tz) is the translation part of this
    * transform.
    *
    * @param transformArrayToPack the array in which this transform is stored. Modified.
    */
   public void get(float[] transformArrayToPack)
   {
      transformArrayToPack[0] = (float) getM00();
      transformArrayToPack[1] = (float) getM01();
      transformArrayToPack[2] = (float) getM02();
      transformArrayToPack[3] = (float) getM03();
      transformArrayToPack[4] = (float) getM10();
      transformArrayToPack[5] = (float) getM11();
      transformArrayToPack[6] = (float) getM12();
      transformArrayToPack[7] = (float) getM13();
      transformArrayToPack[8] = (float) getM20();
      transformArrayToPack[9] = (float) getM21();
      transformArrayToPack[10] = (float) getM22();
      transformArrayToPack[11] = (float) getM23();
      transformArrayToPack[12] = (float) getM30();
      transformArrayToPack[13] = (float) getM31();
      transformArrayToPack[14] = (float) getM32();
      transformArrayToPack[15] = (float) getM33();
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param quaternionToPack the quaternion to set to the rotation of this transform. Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    */
   public void get(QuaternionBasics quaternionToPack, Tuple3DBasics translationToPack)
   {
      quaternionToPack.set(rotationMatrix);
      translationToPack.set(translationVector);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param axisAngleToPack the axis-angle to set to the rotation of this transform. Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    */
   public void get(AxisAngleBasics axisAngleToPack, Tuple3DBasics translationToPack)
   {
      axisAngleToPack.set(rotationMatrix);
      translationToPack.set(translationVector);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector to set to the rotation of this transform.
    *           Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    */
   public void get(Vector3DBasics rotationVectorToPack, Tuple3DBasics translationToPack)
   {
      rotationMatrix.getRotationVector(rotationVectorToPack);
      translationToPack.set(translationVector);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param rotationMarixToPack the matrix to set to the rotation of this transform. Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    */
   public void get(Matrix3DBasics rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(rotationMatrix);
      translationToPack.set(translationVector);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param rotationMarixToPack the matrix to set to the rotation of this transform. Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    */
   public void get(RotationMatrix rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(rotationMatrix);
      translationToPack.set(translationVector);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param rotationMarixToPack the matrix to set to the rotation of this transform. The scale part
    *           is reset. Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    */
   public void get(RotationScaleMatrix rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(rotationMatrix);
      translationToPack.set(translationVector);
   }

   /**
    * Gets the read-only reference to the rotation part of this transform.
    *
    * @return the rotation part of this transform.
    */
   public RotationMatrixReadOnly getRotationMatrix()
   {
      return rotationMatrix;
   }

   /**
    * Packs the rotation part of this rigid-body transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotation(Matrix3DBasics rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationMatrix);
   }

   /**
    * Packs the rotation part of this rigid-body transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationMatrix);
   }

   /**
    * Packs the rotation part of this rigid-body transform.
    *
    * @param rotationMarixToPack the rotation-scale matrix that is set to this transform's rotation.
    *           The scale part is reset. Modified.
    */
   public void getRotation(RotationScaleMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationMatrix);
   }

   /**
    * Packs the rotation part of this rigid-body transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotation(DenseMatrix64F matrixToPack)
   {
      rotationMatrix.get(matrixToPack);
   }

   /**
    * Packs the rotation part of this rigid-body transform in 1D row-major array.
    *
    * @param rotationMatrixArrayToPack the array in which the rotation part of this transform is
    *           stored. Modified.
    */
   public void getRotation(double[] rotationMatrixArrayToPack)
   {
      rotationMatrix.get(rotationMatrixArrayToPack);
   }

   /**
    * Packs the rotation part of this rigid-body transform as a quaternion.
    *
    * @param quaternionToPack the quaternion that is set to the rotation part of this transform.
    *           Modified.
    */
   public void getRotation(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.set(rotationMatrix);
   }

   /**
    * Packs the rotation part of this rigid-body transform as an axis-angle.
    *
    * @param axisAngleToPack the axis-angle that is set to the rotation part of this transform.
    *           Modified.
    */
   public void getRotation(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.set(rotationMatrix);
   }

   /**
    * Packs the rotation part of this rigid-body transform as a rotation vector.
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
      rotationMatrix.getRotationVector(rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by the rotation part of this transform as the
    * yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   public void getRotationYawPitchRoll(double[] yawPitchRollToPack)
   {
      rotationMatrix.getYawPitchRoll(yawPitchRollToPack);
   }

   /**
    * Computes and packs the orientation described by the rotation part of this transform as the
    * Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   public void getRotationEuler(Vector3DBasics eulerAngles)
   {
      rotationMatrix.getEuler(eulerAngles);
   }

   /**
    * Gets the read-only reference of the translation part of this rigid-body transform.
    *
    * @return the translation part of this transform.
    */
   public Vector3DReadOnly getTranslationVector()
   {
      return translationVector;
   }

   /**
    * Packs the translation part of this rigid-body transform.
    *
    * @param translationToPack the tuple in which the translation part of this transform is stored.
    *           Modified.
    */
   public void getTranslation(Tuple3DBasics translationToPack)
   {
      translationToPack.set(translationVector);
   }

   /**
    * Gets the x-component of the translation part of this transform.
    *
    * @return the x-component of the translation part.
    */
   public double getTranslationX()
   {
      return translationVector.getX();
   }

   /**
    * Gets the y-component of the translation part of this transform.
    *
    * @return the y-component of the translation part.
    */
   public double getTranslationY()
   {
      return translationVector.getY();
   }

   /**
    * Gets the z-component of the translation part of this transform.
    *
    * @return the z-component of the translation part.
    */
   public double getTranslationZ()
   {
      return translationVector.getZ();
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
            return rotationMatrix.getElement(row, column);
         }
         else if (column < 4)
         {
            return translationVector.getElement(row);
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
      return rotationMatrix.getM00();
   }

   /**
    * Gets the 1st row 2nd column coefficient of this transform.
    *
    * @return the 1st row 2nd column coefficient.
    */
   public double getM01()
   {
      return rotationMatrix.getM01();
   }

   /**
    * Gets the 1st row 3rd column coefficient of this transform.
    *
    * @return the 1st row 3rd column coefficient.
    */
   public double getM02()
   {
      return rotationMatrix.getM02();
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
      return rotationMatrix.getM10();
   }

   /**
    * Gets the 2nd row 2nd column coefficient of this transform.
    *
    * @return the 2nd row 2nd column coefficient.
    */
   public double getM11()
   {
      return rotationMatrix.getM11();
   }

   /**
    * Gets the 2nd row 3rd column coefficient of this transform.
    *
    * @return the 2nd row 3rd column coefficient.
    */
   public double getM12()
   {
      return rotationMatrix.getM12();
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
      return rotationMatrix.getM20();
   }

   /**
    * Gets the 3rd row 2nd column coefficient of this transform.
    *
    * @return the 3rd row 2nd column coefficient.
    */
   public double getM21()
   {
      return rotationMatrix.getM21();
   }

   /**
    * Gets the 3rd row 3rd column coefficient of this transform.
    *
    * @return the 3rd row 3rd column coefficient.
    */
   public double getM22()
   {
      return rotationMatrix.getM22();
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
    * Tests separately and on a per component basis if the rotation part and the translation part of
    * this transform and {@code other} are equal to an {@code epsilon}.
    *
    * @param other the other rigid-body transform to compare against this. Not modified.
    */
   @Override
   public boolean epsilonEquals(RigidBodyTransform other, double epsilon)
   {
      return rotationMatrix.epsilonEquals(other.rotationMatrix, epsilon) && translationVector.epsilonEquals(other.translationVector, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(RigidBodyTransform)}, it returns {@code false} otherwise or if the
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
         return equals((RigidBodyTransform) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests separately and on a per component basis if the rotation part and the translation part of
    * this transform and {@code other} are exactly equal.
    * <p>
    * The method returns {@code false} if the given transform is {@code null}.
    * </p>
    *
    * @param other the other transform to compare against this. Not modified.
    * @return {@code true} if the two transforms are exactly equal, {@code false} otherwise.
    */
   public boolean equals(RigidBodyTransform other)
   {
      if (other == null)
         return false;
      else
         return rotationMatrix.equals(other.rotationMatrix) && translationVector.equals(other.translationVector);
   }

   /**
    * Two rigid body transforms are considered geometrically equal if both the rotation matrices and
    * translation vectors are equal.
    *
    * @param other the other rigid body transform to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two rigid body transforms are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(RigidBodyTransform other, double epsilon)
   {
      return other.rotationMatrix.geometricallyEquals(rotationMatrix, epsilon) && other.translationVector.geometricallyEquals(translationVector, epsilon);
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
      return EuclidCoreIOTools.getRigidBodyTransformString(this);
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(rotationMatrix.hashCode(), translationVector.hashCode());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}

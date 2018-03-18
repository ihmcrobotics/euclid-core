package us.ihmc.euclid.matrix;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.exceptions.NotARotationScaleMatrixException;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * A {@code RotationScaleMatrix} is a 3-by-3 matrix that represents a 3D orientation times a
 * diagonal matrix holding on scale factors.
 * <p>
 * The application is mostly for 3D graphics where objects are scaled in their local coordinates and
 * rotated.
 * </p>
 * <p>
 * A rotation-scale matrix <i>M</i> is equal to: <i> M = R * S </i>. Where <i>R</i> is a rotation
 * matrix, and <i>S</i> is a scaling matrix as follows:
 *
 * <pre>
 *     / s<sub>x</sub> 0 0 \
 * <i>S</i> = | 0 s<sub>y</sub> 0 |
 *     \ 0 0 s<sub>z</sub> /
 * </pre>
 *
 * where s<sub>x</sub>, s<sub>y</sub>, and s<sub>z</sub> three non-zero positive scale factors.
 * </p>
 * <p>
 * Note: To conserve the form <i> M = R * S </i>, the algebra with a rotation-scale matrix is rather
 * restrictive. For instance, an rotation-scale matrix cannot be inverted. However, it can still
 * perform the inverse of the transform it represents on geometry objects.
 * </p>
 * <p>
 * A best effort has been put in the interface of {@code RotationScaleMatrix} to maximize the use of
 * the inherent properties of its composition and to minimize manipulation errors resulting in an
 * improper rotation-scale matrix.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class RotationScaleMatrix implements Matrix3DBasics, RotationScaleMatrixReadOnly, Settable<RotationScaleMatrix>, EpsilonComparable<RotationScaleMatrix>,
      GeometricallyComparable<RotationScaleMatrix>
{
   /** The rotation part of this rotation-scale matrix. */
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   /** The scale part of this rotation-scale matrix. */
   private final Vector3D scale = new Vector3D(1.0, 1.0, 1.0);

   /**
    * Create a new rotation-scale matrix initialized to identity.
    */
   public RotationScaleMatrix()
   {
      setIdentity();
   }

   /**
    * Creates a new rotation-scale matrix that is the same as {@code other}.
    *
    * @param other the other 3D matrix to copy the values from. Not modified.
    */
   public RotationScaleMatrix(RotationScaleMatrix other)
   {
      set(other);
   }

   /**
    * Creates a new rotation-scale matrix that is the same as {@code rotationScaleMatrix}.
    *
    * @param rotationScaleMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationScaleMatrixException if the resulting matrix is not a rotation-scale
    *            matrix.
    */
   public RotationScaleMatrix(Matrix3DReadOnly rotationScaleMatrix)
   {
      set(rotationScaleMatrix);
   }

   /**
    * Creates a new rotation-scale matrix that is the same as {@code rotationScaleMatrix}.
    *
    * @param rotationScaleMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationScaleMatrixException if the resulting matrix is not a rotation-scale
    *            matrix.
    */
   public RotationScaleMatrix(DenseMatrix64F rotationScaleMatrix)
   {
      set(rotationScaleMatrix);
   }

   /**
    * Creates a new rotation-scale matrix and initializes it from the given array.
    *
    * <pre>
    *        / rotationScaleMatrixArray[0]  rotationScaleMatrixArray[1]  rotationScaleMatrixArray[2] \
    * this = | rotationScaleMatrixArray[3]  rotationScaleMatrixArray[4]  rotationScaleMatrixArray[5] |
    *        \ rotationScaleMatrixArray[6]  rotationScaleMatrixArray[7]  rotationScaleMatrixArray[8] /
    * </pre>
    *
    * @param rotationScaleMatrixArray the array containing the values for this matrix. Not modified.
    * @throws NotARotationScaleMatrixException if the resulting matrix is not a rotation-scale
    *            matrix.
    */
   public RotationScaleMatrix(double[] rotationScaleMatrixArray)
   {
      set(rotationScaleMatrixArray);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code axisAngle} and all three scale factors initialized to {@code scale}.
    *
    * @param axisAngle the axis-angle used to initialized the rotation part. Not modified.
    * @param scale the non-zero and positive scalar used to initialized the scale factors.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public RotationScaleMatrix(AxisAngleReadOnly axisAngle, double scale)
   {
      set(axisAngle, scale);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code axisAngle} and all three scale factors initialized to {@code scaleX}, {@code scaleY},
    * and {@code scaleZ}.
    *
    * @param axisAngle the axis-angle used to initialized the rotation part. Not modified.
    * @param scaleX the non-zero and positive scalar used to initialized the x-axis scale factor.
    * @param scaleY the non-zero and positive scalar used to initialized the y-axis scale factor.
    * @param scaleZ the non-zero and positive scalar used to initialized the z-axis scale factor.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(AxisAngleReadOnly axisAngle, double scaleX, double scaleY, double scaleZ)
   {
      set(axisAngle, scaleX, scaleY, scaleZ);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code axisAngle} and all three scale factors initialized to {@code scales}.
    *
    * @param axisAngle the axis-angle used to initialized the rotation part. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to initialized the scale
    *           factors. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(AxisAngleReadOnly axisAngle, Tuple3DReadOnly scales)
   {
      set(axisAngle, scales);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scale}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to initialized the rotation part. Not modified.
    * @param scales non-zero and positive scalar used to initialized the scale factors.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public RotationScaleMatrix(DenseMatrix64F rotationMatrix, double scale)
   {
      set(rotationMatrix, scale);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scaleX},
    * {@code scaleY}, and {@code scaleZ}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to initialized the rotation part. Not modified.
    * @param scaleX the non-zero and positive scalar used to initialized the x-axis scale factor.
    * @param scaleY the non-zero and positive scalar used to initialized the y-axis scale factor.
    * @param scaleZ the non-zero and positive scalar used to initialized the z-axis scale factor.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(DenseMatrix64F rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      set(rotationMatrix, scaleX, scaleY, scaleZ);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scales}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to initialized the rotation part. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to initialized the scale
    *           factors. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(DenseMatrix64F rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code quaternion} and all three scale factors initialized to {@code scale}.
    *
    * @param quaternion the quaternion used to initialized the rotation part. Not modified.
    * @param scales non-zero and positive scalar used to initialized the scale factors.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public RotationScaleMatrix(QuaternionReadOnly quaternion, double scale)
   {
      set(quaternion, scale);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code quaternion} and all three scale factors initialized to {@code scaleX}, {@code scaleY},
    * and {@code scaleZ}.
    *
    * @param quaternion the quaternion used to initialized the rotation part. Not modified.
    * @param scaleX the non-zero and positive scalar used to initialized the x-axis scale factor.
    * @param scaleY the non-zero and positive scalar used to initialized the y-axis scale factor.
    * @param scaleZ the non-zero and positive scalar used to initialized the z-axis scale factor.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(QuaternionReadOnly quaternion, double scaleX, double scaleY, double scaleZ)
   {
      set(quaternion, scaleX, scaleY, scaleZ);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code quaternion} and all three scale factors initialized to {@code scales}.
    *
    * @param quaternion the quaternion used to initialized the rotation part. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to initialized the scale
    *           factors. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(QuaternionReadOnly quaternion, Tuple3DReadOnly scales)
   {
      set(quaternion, scales);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scale}.
    *
    * @param rotationMatrix the rotation matrix used to initialized the rotation part. Not modified.
    * @param scales non-zero and positive scalar used to initialized the scale factors.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scaleX},
    * {@code scaleY}, and {@code scaleZ}.
    *
    * @param rotationMatrix the rotation matrix used to initialized the rotation part. Not modified.
    * @param scaleX the non-zero and positive scalar used to initialized the x-axis scale factor.
    * @param scaleY the non-zero and positive scalar used to initialized the y-axis scale factor.
    * @param scaleZ the non-zero and positive scalar used to initialized the z-axis scale factor.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      set(rotationMatrix, scaleX, scaleY, scaleZ);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scales}.
    *
    * @param rotationMatrix the rotation matrix used to initialized the rotation part. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to initialized the scale
    *           factors. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales);
   }

   /**
    * Asserts that this rotation-scale matrix is proper.
    *
    * @throws NotARotationScaleMatrixException if the rotation part is not a rotation matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void checkIfRotationScaleMatrixProper()
   {
      checkIfRotationMatrixProper();
      checkIfScalesProper();
   }

   /**
    * Asserts that the rotation part of this rotation-scale matrix is proper.
    *
    * @throws NotARotationScaleMatrixException if the rotation part is not a rotation matrix.
    */
   public void checkIfRotationMatrixProper()
   {
      if (!rotationMatrix.isRotationMatrix())
         throw new NotARotationScaleMatrixException(this);
   }

   /**
    * Asserts that the scale part of this rotation-scale matrix is proper.
    *
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void checkIfScalesProper()
   {
      checkIfScalesProper(scale.getX(), scale.getY(), scale.getZ());
   }

   /**
    * Orthonormalization of the rotation part of this rotation-scale matrix using the
    * <a href="https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process"> Gram-Schmidt method</a>.
    *
    * @throws NotARotationMatrixException if the orthonormalization failed.
    */
   public void normalizeRotationMatrix()
   {
      rotationMatrix.normalize();
   }

   /**
    * Resets all the scale factors to 1.0.
    */
   public void resetScale()
   {
      scale.set(1.0, 1.0, 1.0);
   }

   /**
    * Sets this rotation-scale matrix to identity representing a 'zero' rotation without scale.
    */
   @Override
   public void setToZero()
   {
      setIdentity();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      rotationMatrix.setToNaN();
      scale.setToNaN();
   }

   @Override
   public void setIdentity()
   {
      rotationMatrix.setIdentity();
      resetScale();
   }

   /**
    * Sets the rotation part to represent a 'zero' rotation.
    */
   public void setRotationToZero()
   {
      rotationMatrix.setToZero();
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return rotationMatrix.containsNaN() || scale.containsNaN();
   }

   /**
    * Sets this rotation-scale matrix to equal the given one {@code other}.
    *
    * @param other the other rotation-scale matrix to copy the values from. Not modified.
    */
   @Override
   public void set(RotationScaleMatrix other)
   {
      setRotation(other.rotationMatrix);
      setScale(other.scale);
   }

   /**
    * Sets this rotation-scale matrix to equal the given one {@code other}.
    *
    * @param other the other rotation-scale matrix to copy the values from. Not modified.
    */
   public void set(RotationScaleMatrixReadOnly other)
   {
      setRotation(other.getRotationMatrix());
      setScale(other.getScale());
   }

   /**
    * Sets the rotation part to the given rotation matrix and resets the scales.
    *
    * @param other the other rotation matrix to copy the values from. Not modified.
    */
   public void set(RotationMatrixReadOnly rotationMatrix)
   {
      setRotation(rotationMatrix);
      resetScale();
   }

   /**
    * {@inheritDoc}
    *
    * @throws NotARotationScaleMatrixException if the resulting matrix is not a rotation-scale
    *            matrix.
    */
   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      if (Matrix3DFeatures.determinant(m00, m01, m02, m10, m11, m12, m20, m21, m22) <= 0.0)
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      scale.setX(Math.sqrt(m00 * m00 + m10 * m10 + m20 * m20));
      scale.setY(Math.sqrt(m01 * m01 + m11 * m11 + m21 * m21));
      scale.setZ(Math.sqrt(m02 * m02 + m12 * m12 + m22 * m22));

      if (scale.getX() == 0.0 || scale.getY() == 0.0 || scale.getZ() == 0.0)
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      double rot00 = m00 / scale.getX();
      double rot01 = m01 / scale.getY();
      double rot02 = m02 / scale.getZ();
      double rot10 = m10 / scale.getX();
      double rot11 = m11 / scale.getY();
      double rot12 = m12 / scale.getZ();
      double rot20 = m20 / scale.getX();
      double rot21 = m21 / scale.getY();
      double rot22 = m22 / scale.getZ();

      if (!Matrix3DFeatures.isRotationMatrix(rot00, rot01, rot02, rot10, rot11, rot12, rot20, rot21, rot22))
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      rotationMatrix.setUnsafe(rot00, rot01, rot02, rot10, rot11, rot12, rot20, rot21, rot22);
   }

   /**
    * Sets the rotation part to the {@code axisAngle} and all three scale factors to {@code scale}.
    *
    * @param axisAngle the axis-angle used to set the rotation part to. Not modified.
    * @param scale the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(AxisAngleReadOnly axisAngle, double scale)
   {
      set(axisAngle, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code axisAngle} and all three scale factors to {@code scaleX},
    * {@code scaleY}, and {@code scaleZ}.
    *
    * @param axisAngle the axis-angle used to set the rotation part to. Not modified.
    * @param scaleX the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(AxisAngleReadOnly axisAngle, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(axisAngle);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code axisAngle} and all three scale factors to {@code scales}.
    *
    * @param axisAngle the axis-angle used to set the rotation part to. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to set the scale factors
    *           to. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(AxisAngleReadOnly axisAngle, Tuple3DReadOnly scales)
   {
      set(axisAngle, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scale}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to set the rotation part to. Not modified.
    * @param scales non-zero and positive scalar used to initialized the scale factors.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(DenseMatrix64F rotationMatrix, double scale)
   {
      set(rotationMatrix, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to set the rotation part to. Not modified.
    * @param scaleX the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(DenseMatrix64F rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(rotationMatrix);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scales}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to set the rotation part to. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to set the scale factors
    *           to. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(DenseMatrix64F rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to the {@code quaternion} and all three scale factors to {@code scale}.
    *
    * @param quaternion the quaternion used to set the rotation part to. Not modified.
    * @param scale the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(QuaternionReadOnly quaternion, double scale)
   {
      set(quaternion, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code quaternion} and all three scale factors to
    * {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    *
    * @param quaternion the quaternion used to set the rotation part to. Not modified.
    * @param scaleX the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(QuaternionReadOnly quaternion, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(quaternion);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code quaternion} and all three scale factors to
    * {@code scales}.
    *
    * @param quaternion the quaternion used to set the rotation part to. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to set the scale factors
    *           to. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(QuaternionReadOnly quaternion, Tuple3DReadOnly scales)
   {
      set(quaternion, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scale}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    * @param scale the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(Matrix3DReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part to. Not modified.
    * @param scaleX the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(Matrix3DReadOnly rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(rotationMatrix);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scales}.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part to. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to set the scale factors
    *           to. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(Matrix3DReadOnly rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scale}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    * @param scale the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part to. Not modified.
    * @param scaleX the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(rotationMatrix);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scales}.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part to. Not modified.
    * @param scales tuple holding on the non-zero and positive scalars used to set the scale factors
    *           to. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to {@code rotationMatrix}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotation(DenseMatrix64F rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   /**
    * Sets the rotation part to {@code rotationMatrixArray}.
    *
    * @param rotationMatrixArray the array containing the rotation part values. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrixArray} is not a rotation
    *            matrix.
    */
   public void setRotation(double[] rotationMatrixArray)
   {
      rotationMatrix.set(rotationMatrixArray);
   }

   /**
    * Sets the rotation part to {@code axisAngle}.
    *
    * @param axisAngle the axis-angle used to set the rotation part to. Not modified.
    */
   public void setRotation(AxisAngleReadOnly axisAngle)
   {
      rotationMatrix.set(axisAngle);
   }

   /**
    * Sets the rotation part to {@code quaternion}.
    *
    * @param quaternion the quaternion used to set the rotation part to. Not modified.
    */
   public void setRotation(QuaternionReadOnly quaternion)
   {
      rotationMatrix.set(quaternion);
   }

   /**
    * Sets the rotation part to {@code rotationMatrix}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation
    *            matrix.
    */
   public void setRotation(Matrix3DReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   /**
    * Sets the rotation part to {@code rotationMatrix}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    */
   public void setRotation(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   /**
    * Sets the rotation part to the rotation vector {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param axisAngle the axis-angle used to set the rotation part to. Not modified.
    */
   public void setRotation(Vector3DReadOnly rotationVector)
   {
      rotationMatrix.set(rotationVector);
   }

   /**
    * Sets the rotation part to represent a counter clockwise rotation around the z-axis of an angle
    * {@code yaw}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \
    * R = | sin(yaw)  cos(yaw) 0 |
    *     \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setRotationYaw(double yaw)
   {
      rotationMatrix.setToYawMatrix(yaw);
   }

   /**
    * Sets the rotation part to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setRotationPitch(double pitch)
   {
      rotationMatrix.setToPitchMatrix(pitch);
   }

   /**
    * Sets the rotation part to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationRoll(double roll)
   {
      rotationMatrix.setToRollMatrix(roll);
   }

   /**
    * Sets the rotation part to represent the same orientation as the given yaw-pitch-roll angles
    * {@code yawPitchRoll}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not
    *           modified.
    */
   public void setRotationYawPitchRoll(double[] yawPitchRoll)
   {
      setRotationYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   /**
    * Sets the rotation part to represent the same orientation as the given yaw-pitch-roll angles
    * {@code yaw}, {@code pitch}, and {@code roll}.
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
   public void setRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      rotationMatrix.setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets the rotation part to represent the same orientation as the given Euler angles
    * {@code eulerAngles}.
    *
    * <pre>
    *     / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * R = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *     \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
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
    * Sets the rotation part to represent the same orientation as the given Euler angles
    * {@code rotX}, {@code rotY}, and {@code rotZ}.
    *
    * <pre>
    *        / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * this = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *        \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
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
    * Sets all the scale factors to {@code scale}.
    *
    * @param scale the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void setScale(double scale)
   {
      setScale(scale, scale, scale);
   }

   /**
    * Sets all the scale factors to {@code scale}.
    *
    * @param scaleX the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void setScale(double scaleX, double scaleY, double scaleZ)
   {
      checkIfScalesProper(scaleX, scaleY, scaleZ);

      scale.set(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the scale factors to {@code scales}.
    *
    * @param scales tuple holding on the non-zero and positive scalars used to set the scale factors
    *           to. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void setScale(Tuple3DReadOnly scales)
   {
      setScale(scales.getX(), scales.getY(), scales.getZ());
   }

   private void checkIfScalesProper(double scaleX, double scaleY, double scaleZ)
   {
      if (scaleX < 0.0 || scaleY < 0.0 || scaleZ < 0.0)
         throw new NotARotationScaleMatrixException("Mirroring is not handled, scale values: " + scaleX + ", " + scaleY + ", " + scaleZ + ".");
   }

   /**
    * Resets the scale factors and sets the rotation part to represent a counter clockwise rotation
    * around the z-axis of an angle {@code yaw}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 |
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setToYawMatrix(double yaw)
   {
      setRotationYaw(yaw);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent a counter clockwise rotation
    * around the y-axis of an angle {@code pitch}.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setToPitchMatrix(double pitch)
   {
      setRotationPitch(pitch);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent a counter clockwise rotation
    * around the x-axis of an angle {@code roll}.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setToRollMatrix(double roll)
   {
      setRotationRoll(roll);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent the same orientation as the
    * given yaw-pitch-roll angles {@code yawPitchRoll}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * this = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *        \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not
    *           modified.
    */
   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   /**
    * Resets the scale factors and sets the rotation part to represent the same orientation as the
    * given yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * this = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *        \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      setRotationYawPitchRoll(yaw, pitch, roll);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent the same orientation as the
    * given Euler angles {@code eulerAngles}.
    *
    * <pre>
    *        / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * this = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *        \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   public void setEuler(Vector3DReadOnly eulerAngles)
   {
      setRotationEuler(eulerAngles);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent the same orientation as the
    * given Euler angles {@code rotX}, {@code rotY}, and {@code rotZ}.
    *
    * <pre>
    *        / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * this = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *        \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
    * <p>
    * This is equivalent to {@code this.setYawPitchRoll(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   public void setEuler(double rotX, double rotY, double rotZ)
   {
      setRotationEuler(rotX, rotY, rotZ);
      resetScale();
   }

   /**
    * Multiplies the given {@code rotationMatrix} to the rotation part of this rotation-scale
    * matrix.
    * <p>
    * R = R * rotationMatrix <br>
    * with R being the rotation part of this matrix.
    * </p>
    *
    * @param rotationMatrix the rotation matrix to multiply this with. Not modified
    */
   public void multiply(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.multiply(rotationMatrix);
   }

   /**
    * Multiplies the given {@code quaternion} to the rotation part of this rotation-scale matrix.
    * <p>
    * R = R * R(quaternion) <br>
    * with R being the rotation part of this matrix and R(quaternion) is the function to convert a
    * quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this with. Not modified.
    */
   public void multiply(QuaternionReadOnly quaternion)
   {
      rotationMatrix.multiply(quaternion);
   }

   /**
    * Multiplies the given {@code rotationMatrix} to the transpose of the rotation part of this
    * rotation-scale matrix.
    * <p>
    * R = R<sup>T</sup> * rotationMatrix <br>
    * with R being the rotation part of this matrix.
    * </p>
    *
    * @param rotationMatrix the rotation matrix to multiply this with. Not modified
    */
   public void multiplyTransposeThis(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.multiplyTransposeThis(rotationMatrix);
   }

   /**
    * Multiplies the transpose of the given {@code rotationMatrix} to the rotation part of this
    * rotation-scale matrix.
    * <p>
    * R = R * rotationMatrix<sup>T</sup> <br>
    * with R being the rotation part of this matrix.
    * </p>
    *
    * @param rotationMatrix the rotation matrix to multiply this with. Not modified
    */
   public void multiplyTransposeOther(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.multiplyTransposeOther(rotationMatrix);
   }

   /**
    * Multiplies the given {@code quaternion} to the transpose of the rotation part of this
    * rotation-scale matrix.
    * <p>
    * R = R<sup>T</sup> * R(quaternion) <br>
    * with R being the rotation part of this matrix and R(quaternion) is the function to convert a
    * quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this with. Not modified
    */
   public void multiplyTransposeThis(QuaternionReadOnly quaternion)
   {
      rotationMatrix.multiplyTransposeThis(quaternion);
   }

   /**
    * Multiplies the conjugate of the given {@code quaternion} to the rotation part of this
    * rotation-scale matrix.
    * <p>
    * R = R * R(quaternion)<sup>T</sup> <br>
    * with R being the rotation part of this matrix and R(quaternion) is the function to convert a
    * quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this with. Not modified
    */
   public void multiplyConjugateQuaternion(QuaternionReadOnly quaternion)
   {
      rotationMatrix.multiplyConjugateQuaternion(quaternion);
   }

   /**
    * Append a rotation about the z-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *         / cos(yaw) -sin(yaw) 0 \
    * R = R * | sin(yaw)  cos(yaw) 0 |
    *         \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void appendYawRotation(double yaw)
   {
      rotationMatrix.appendYawRotation(yaw);
   }

   /**
    * Append a rotation about the y-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *         /  cos(pitch) 0 sin(pitch) \
    * R = R * |      0      1     0      |
    *         \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void appendPitchRotation(double pitch)
   {
      rotationMatrix.appendPitchRotation(pitch);
   }

   /**
    * Append a rotation about the x-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *         /  cos(pitch) 0 sin(pitch) \
    * R = R * |      0      1     0      |
    *         \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param yaw the angle to rotate about the x-axis.
    */
   public void appendRollRotation(double roll)
   {
      rotationMatrix.appendRollRotation(roll);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this
    * </p>
    *
    * @param rotationMatrix the rotation matrix to multiply with by. Not modified.
    */
   public void preMultiply(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.preMultiply(rotationMatrix);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = R(quaternion) * this where R(quaternion) is the function to convert a quaternion into a
    * rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply with by. Not modified.
    */
   public void preMultiply(QuaternionReadOnly quaternion)
   {
      rotationMatrix.preMultiply(quaternion);
   }

   /**
    * Sets the rotation part of this to the multiplication of the transpose of the rotation part of
    * this with the given {@code rotationMatrix}.
    * <p>
    * R = rotationMatrix * R<sup>T</sup> <br>
    * with R being the rotation part of this matrix.
    * </p>
    *
    * @param rotationMatrix the rotation matrix to multiply this with. Not modified
    */
   public void preMultiplyTransposeThis(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.preMultiplyTransposeThis(rotationMatrix);
   }

   /**
    * Sets the rotation part of this to the multiplication of the rotation part of this with the
    * transpose of the given {@code rotationMatrix}.
    * <p>
    * this = other<sup>T</sup> * this
    * </p>
    *
    * @param rotationMatrix the rotation matrix to multiply with by. Not modified.
    */
   public void preMultiplyTransposeOther(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.preMultiplyTransposeOther(rotationMatrix);
   }

   /**
    * Sets the rotation part of this to the multiplication of the transpose of the rotation part of
    * this with the given {@code quaternion}.
    * <p>
    * R = R(quaternion) * R<sup>T</sup> <br>
    * with R being the rotation part of this matrix and R(quaternion) is the function to convert a
    * quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this with. Not modified
    */
   public void preMultiplyTransposeThis(QuaternionReadOnly quaternion)
   {
      rotationMatrix.preMultiplyTransposeThis(quaternion);
   }

   /**
    * Sets the rotation part of this to the multiplication of the rotation part of this with the
    * conjugate of the given {@code quaternion}.
    * <p>
    * R = R(quaternion)<sup>T</sup> * R <br>
    * with R being the rotation part of this matrix and R(quaternion) is the function to convert a
    * quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this with. Not modified
    */
   public void preMultiplyConjugateQuaternion(QuaternionReadOnly quaternion)
   {
      rotationMatrix.preMultiplyConjugateQuaternion(quaternion);
   }

   /**
    * Prepend a rotation about the z-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \
    * R = | sin(yaw)  cos(yaw) 0 | * R
    *     \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void prependYawRotation(double yaw)
   {
      rotationMatrix.prependYawRotation(yaw);
   }

   /**
    * Prepend a rotation about the y-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *     /  cos(pitch) 0 sin(pitch) \
    * R = |      0      1     0      | * R
    *     \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void prependPitchRotation(double pitch)
   {
      rotationMatrix.prependPitchRotation(pitch);
   }

   /**
    * Prepend a rotation about the x-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *     /  cos(pitch) 0 sin(pitch) \
    * R = |      0      1     0      | * R
    *     \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param yaw the angle to rotate about the x-axis.
    */
   public void prependRollRotation(double roll)
   {
      rotationMatrix.prependRollRotation(roll);
   }

   /**
    * Retrieves the scale factor with the maximum value and returns it.
    *
    * @return the maximum value among the scale factors.
    */
   public double getMaxScale()
   {
      return EuclidCoreTools.max(scale.getX(), scale.getY(), scale.getZ());
   }

   /**
    * Packs the rotation part as a rotation matrix.
    *
    * @param rotationMatrixToPack the rotation matrix in which the rotation part is stored.
    *           Modified.
    */
   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationMatrix);
   }

   /**
    * Packs the rotation part as a rotation matrix and stores it into a row-major 1D array.
    *
    * @param rotationMatrixArrayToPack the array in which the coefficients of the rotation part are
    *           stored. Modified.
    */
   public void getRotation(double[] rotationMatrixArrayToPack)
   {
      rotationMatrix.get(rotationMatrixArrayToPack);
   }

   /**
    * Packs the rotation part as a rotation matrix.
    *
    * @param rotationMatrixToPack the rotation matrix in which the rotation part is stored.
    *           Modified.
    */
   public void getRotation(DenseMatrix64F rotationMatrixToPack)
   {
      rotationMatrix.get(rotationMatrixToPack);
   }

   /**
    * Packs the rotation part as a quaternion.
    *
    * @param quaternionToPack the quaternion in which the rotation part is stored. Modified.
    */
   public void getRotation(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.set(rotationMatrix);
   }

   /**
    * Packs the rotation part as an axis-angle.
    *
    * @param axisAngleToPack the axis-angle in which the rotation part is stored. Modified.
    */
   public void getRotation(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.set(rotationMatrix);
   }

   /**
    * Packs the rotation part as an rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector in which the rotation part is stored.
    *           Modified.
    */
   public void getRotation(Vector3DBasics rotationVectorToPack)
   {
      rotationMatrix.get(rotationVectorToPack);
   }

   /**
    * Packs the orientation described by the rotation part as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   public void getRotationEuler(Tuple3DBasics eulerAnglesToPack)
   {
      rotationMatrix.getEuler(eulerAnglesToPack);
   }

   /**
    * Packs the orientation described by the rotation part as the yaw-pitch-roll angles.
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
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of the rotation
    * part.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   public double getRotationYaw()
   {
      return rotationMatrix.getYaw();
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of the rotation
    * part.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   public double getRotationPitch()
   {
      return rotationMatrix.getPitch();
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of the rotation
    * part.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   public double getRotationRoll()
   {
      return rotationMatrix.getRoll();
   }

   /**
    * Packs the scale factors in a tuple.
    *
    * @param scaleToPack the tuple in which the scale factors are stored. Modified.
    */
   public void getScale(Tuple3DBasics scaleToPack)
   {
      scaleToPack.set(scale);
   }

   /** {@inheritDoc} */
   @Override
   public RotationMatrixReadOnly getRotationMatrix()
   {
      return rotationMatrix;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DReadOnly getScale()
   {
      return scale;
   }

   /** {@inheritDoc} */
   @Override
   public double getM00()
   {
      return rotationMatrix.getM00() * scale.getX();
   }

   /** {@inheritDoc} */
   @Override
   public double getM01()
   {
      return rotationMatrix.getM01() * scale.getY();
   }

   /** {@inheritDoc} */
   @Override
   public double getM02()
   {
      return rotationMatrix.getM02() * scale.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public double getM10()
   {
      return rotationMatrix.getM10() * scale.getX();
   }

   /** {@inheritDoc} */
   @Override
   public double getM11()
   {
      return rotationMatrix.getM11() * scale.getY();
   }

   /** {@inheritDoc} */
   @Override
   public double getM12()
   {
      return rotationMatrix.getM12() * scale.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public double getM20()
   {
      return rotationMatrix.getM20() * scale.getX();
   }

   /** {@inheritDoc} */
   @Override
   public double getM21()
   {
      return rotationMatrix.getM21() * scale.getY();
   }

   /** {@inheritDoc} */
   @Override
   public double getM22()
   {
      return rotationMatrix.getM22() * scale.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public double getScaleX()
   {
      return scale.getX();
   }

   /** {@inheritDoc} */
   @Override
   public double getScaleY()
   {
      return scale.getY();
   }

   /** {@inheritDoc} */
   @Override
   public double getScaleZ()
   {
      return scale.getZ();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(RotationScaleMatrix)}, it returns {@code false} otherwise or if the
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
         return equals((RotationScaleMatrix) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests if the rotation parts and scales of both matrices are exactly equal.
    * <p>
    * The method returns {@code false} if the given matrix is {@code null}.
    * </p>
    *
    * @param other the other matrix to compare against this. Not modified.
    * @return {@code true} if the two matrices are exactly equal, {@code false} otherwise.
    */
   public boolean equals(RotationScaleMatrix other)
   {
      if (other == null)
         return false;
      else
         return rotationMatrix.equals(other.rotationMatrix) && scale.equals(other.scale);
   }

   /**
    * Provides a {@code String} representation of this matrix as follows: <br>
    * m00, m01, m02 <br>
    * m10, m11, m12 <br>
    * m20, m21, m22
    *
    * @return the {@code String} representing this matrix.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getMatrixString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this matrix.
    *
    * @return the hash code value for this matrix.
    */
   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.combineHashCode(rotationMatrix.hashCode(), scale.hashCode());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   /**
    * Tests the rotation parts and scales of both matrices are equal to an {@code epsilon}.
    *
    * @param other the other matrix to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two matrix are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(RotationScaleMatrix other, double epsilon)
   {
      return RotationScaleMatrixReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same rotation-scale to an
    * {@code epsilon}.
    * <p>
    * Two rotation-scale matrices are considered geometrically equal if the their respective
    * rotation matrices and scale vectors are geometrically equal.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other rotation-scale matrix to compare against this. Not modified.
    * @param epsilon the threshold used when comparing the internal rotation and scale to
    *           {@code other}'s rotation and scale.
    * @return {@code true} if the two rotation-scale matrices represent the same geometry,
    *         {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(RotationScaleMatrix other, double epsilon)
   {
      return RotationScaleMatrixReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
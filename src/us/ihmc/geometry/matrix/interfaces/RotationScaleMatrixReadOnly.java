package us.ihmc.geometry.matrix.interfaces;

import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.tuple.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read interface for 3-by-3 rotation-scale matrices.
 * <p>
 * A rotation-scale matrix <i>M</i> is equal to: <i> M = R * S </i>.
 * Where <i>R</i> is a rotation matrix, and <i>S</i> is a scaling
 * matrix as follows:
 * <pre>
 *     / s<sub>x</sub> 0 0 \
 * <i>S</i> = | 0 s<sub>y</sub> 0 |
 *     \ 0 0 s<sub>z</sub> /
 * </pre>
 * where s<sub>x</sub>, s<sub>y</sub>, and s<sub>z</sub> three non-zero positive scale factors.
 * </p>
 * <p>
 * Note: To conserve the form <i> M = R * S </i>, the algebra with a rotation-scale
 * matrix is rather restrictive.
 * For instance, an rotation-scale matrix cannot be inverted.
 * However, it can still perform the inverse of the transform it represents on geometry objects.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> the final type of matrix used.
 */
public interface RotationScaleMatrixReadOnly<T extends RotationScaleMatrixReadOnly<T>> extends Matrix3DReadOnly<T>
{
   /**
    * Returns the read-only reference to the rotation matrix
    * used to compose this rotation-scale matrix. 
    * 
    * @return the read-only reference to the rotation matrix.
    */
   public RotationMatrixReadOnly<?> getRotationMatrix();

   /**
    * Returns the read-only reference to the scale factors
    * used to compose this rotation-scale matrix.
    * 
    * @return the read-only reference to the scale factors.
    */
   public TupleReadOnly getScale();

   /**
    * Returns the current value of the first scale factor
    * of this rotation-scale matrix.
    * 
    * @return the value of the first scale factor.
    */
   public double getScaleX();

   /**
    * Returns the current value of the second scale factor
    * of this rotation-scale matrix.
    * 
    * @return the value of the second scale factor.
    */
   public double getScaleY();

   /**
    * Returns the current value of the third scale factor
    * of this rotation-scale matrix.
    * 
    * @return the value of the third scale factor.
    */
   public double getScaleZ();

   /** {@inheritDoc} */
   @Override
   default void transform(TupleReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      tupleTransformed.setX(getScaleX() * tupleOriginal.getX());
      tupleTransformed.setY(getScaleY() * tupleOriginal.getY());
      tupleTransformed.setZ(getScaleZ() * tupleOriginal.getZ());

      getRotationMatrix().transform(tupleTransformed, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(TupleReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = tupleTransformed.getX();
      double y = tupleTransformed.getY();
      double z = tupleTransformed.getZ();
      transform(tupleOriginal, tupleTransformed);
      tupleTransformed.add(x, y, z);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.setX(getScaleX() * tupleOriginal.getX());
      tupleTransformed.setY(getScaleY() * tupleOriginal.getY());

      getRotationMatrix().transform(tupleTransformed, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given quaternion by the rotation part
    * of this rotation-scale matrix.
    * <p>
    * quaternionToTransform = Q(this.getRotationMatrix()) * quaternionToTransform <br>
    * where Q(this.getRotationMatrix()) is the equivalent quaternion
    * for the rotation part of this rotation-scale matrix.
    * </p>
    * 
    * @param quaternionToTransform the quaternion to transform. Modified.
    */
   default void transform(QuaternionBasics quaternionToTransform)
   {
      transform(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Transforms the given quaternion
    * {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionToTransform = Q(this.getRotationMatrix()) * quaternionToTransform <br>
    * where Q(this.getRotationMatrix()) is the equivalent quaternion
    * for the rotation part of this rotation-scale matrix.
    * </p>
    * 
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   default void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      getRotationMatrix().transform(quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      vectorTransformed.setX(getScaleX() * vectorOriginal.getX());
      vectorTransformed.setY(getScaleY() * vectorOriginal.getY());
      vectorTransformed.setZ(getScaleZ() * vectorOriginal.getZ());
      vectorTransformed.setS(vectorOriginal.getS());

      getRotationMatrix().transform(vectorTransformed, vectorTransformed);
   }

   /**
    * Transforms the given rotation matrix by the
    * rotation part of this rotation-scale matrix.
    * <p>
    * matrixToTransform = this.getRotationMatrix() * matrixToTransform
    * </p>
    * 
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given rotation matrix {@code matrixOriginal} by the
    * rotation part of this rotation-scale matrix and stores the result
    * in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this.getRotationMatrix() * matrixOriginal
    * </p>
    * 
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void transform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTransformed)
   {
      getRotationMatrix().transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      matrixTransformed.set(matrixOriginal);
      // Equivalent to: M = S * M
      matrixTransformed.scaleRows(getScaleX(), getScaleY(), getScaleZ());
      // Equivalent to: M = M * S^-1
      matrixTransformed.scaleColumns(1.0 / getScaleX(), 1.0 / getScaleY(), 1.0 / getScaleZ());
      getRotationMatrix().transform(matrixTransformed, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(TupleReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      getRotationMatrix().inverseTransform(tupleOriginal, tupleTransformed);

      tupleTransformed.setX(tupleTransformed.getX() / getScaleX());
      tupleTransformed.setY(tupleTransformed.getY() / getScaleY());
      tupleTransformed.setZ(tupleTransformed.getZ() / getScaleZ());
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      getRotationMatrix().inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);

      tupleTransformed.setX(tupleTransformed.getX() / getScaleX());
      tupleTransformed.setY(tupleTransformed.getY() / getScaleY());
   }

   /**
    * Performs the inverse of the transform to the given quaternion by the rotation part
    * of this rotation-scale matrix.
    * <p>
    * quaternionToTransform = Q(this.getRotationMatrix()<sup>-1</sup>) * quaternionToTransform <br>
    * where Q(this.getRotationMatrix()<sup>-1</sup>) is the equivalent quaternion
    * for the inverse of the rotation part of this rotation-scale matrix.
    * </p>
    * <p>
    * This operation uses the property:
    * <br> q<sup>-1</sup> = conjugate(q) </br>
    * of a quaternion preventing to actually compute the inverse of the matrix.
    * </p>
    * 
    * @param quaternionToTransform the quaternion to transform. Modified.
    */
   default void inverseTransform(QuaternionBasics quaternionToTransform)
   {
      inverseTransform(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Performs the inverse of the transform to the given quaternion
    * {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionToTransform = Q(this.getRotationMatrix()<sup>-1</sup>) * quaternionToTransform <br>
    * where Q(this.getRotationMatrix()<sup>-1</sup>) is the equivalent quaternion
    * for the inverse of the rotation part of this rotation-scale matrix.
    * </p>
    * <p>
    * This operation uses the property:
    * <br> q<sup>-1</sup> = conjugate(q) </br>
    * of a quaternion preventing to actually compute the inverse of the matrix.
    * </p>
    * 
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   default void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      getRotationMatrix().inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      getRotationMatrix().inverseTransform(vectorOriginal, vectorTransformed);

      vectorTransformed.setX(vectorTransformed.getX() / getScaleX());
      vectorTransformed.setY(vectorTransformed.getY() / getScaleY());
      vectorTransformed.setZ(vectorTransformed.getZ() / getScaleZ());
   }

   /**
    * Performs the inverse of the transform to the given rotation matrix by the
    * rotation part of this rotation-scale matrix.
    * <p>
    * matrixToTransform = this.getRotationMatrix()<sup>-1</sup> * matrixToTransform
    * </p>
    * <p>
    * This operation uses the property:
    * <br> R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    * 
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void inverseTransform(RotationMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given rotation matrix {@code matrixOriginal} by the
    * rotation part of this rotation-scale matrix and stores the result
    * in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this.getRotationMatrix()<sup>-1</sup> * matrixOriginal
    * </p>
    * <p>
    * This operation uses the property:
    * <br> R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    * 
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void inverseTransform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTransformed)
   {
      getRotationMatrix().inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      getRotationMatrix().inverseTransform(matrixOriginal, matrixTransformed);
      // Equivalent to: M = S^-1 * M
      matrixTransformed.scaleRows(1.0 / getScaleX(), 1.0 / getScaleY(), 1.0 / getScaleZ());
      // Equivalent to: M = M * S
      matrixTransformed.scaleColumns(getScaleX(), getScaleY(), getScaleZ());
   }
}

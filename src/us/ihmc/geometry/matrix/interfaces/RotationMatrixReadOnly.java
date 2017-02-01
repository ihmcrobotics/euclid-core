package us.ihmc.geometry.matrix.interfaces;

import us.ihmc.geometry.exceptions.NotARotationMatrixException;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.Matrix3DTools;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationMatrixTools;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.QuaternionTools;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read interface used for 3-by-3 rotation matrices.
 * <p>
 * A rotation matrix is used to represent a 3D orientation
 * through its 9 coefficients.
 * A rotation matrix has to comply to several constraints:
 * <ul>
 *    <li> each column of the matrix represents a unitary vector,
 *    <li> each row of the matrix represents a unitary vector,
 *    <li> every pair of columns of the matrix represents two orthogonal vectors,
 *    <li> every pair of rows of the matrix represents two orthogonal vectors,
 *    <li> the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> the final type of matrix used.
 */
public interface RotationMatrixReadOnly<T extends RotationMatrixReadOnly<T>> extends Matrix3DReadOnly<T>
{
   /**
    * Orthonormalization of the rotation matrix using the
    * <a href="https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process"> Gram-Schmidt method</a>.
    * 
    * @throws NotARotationMatrixException if the orthonormalization failed.
    */
   void normalize();

   /** {@inheritDoc} */
   @Override
   default void transform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      normalize();
      Matrix3DTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      normalize();
      Matrix3DTools.addTransform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      normalize();
      Matrix3DTools.transform(this, tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
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
    * Transforms the given quaternion
    * {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionToTransform = Q(this) * quaternionToTransform <br>
    * where Q(this) is the equivalent quaternion for this rotation matrix.
    * </p>
    * 
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   default void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      normalize();
      QuaternionTools.multiply(this, quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      normalize();
      Matrix3DTools.transform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given rotation matrix by this
    * rotation matrix.
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
    * Transforms the given rotation matrix {@code matrixOriginal} by this
    * rotation matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this * matrixOriginal
    * </p>
    * 
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void transform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTransformed)
   {
      normalize();
      RotationMatrixTools.multiply(this, matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      normalize();
      Matrix3DTools.multiply(this, matrixOriginal, matrixTransformed);
      Matrix3DTools.multiplyTransposeRight(matrixTransformed, this, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      normalize();
      double x = getM00() * tupleOriginal.getX() + getM10() * tupleOriginal.getY() + getM20() * tupleOriginal.getZ();
      double y = getM01() * tupleOriginal.getX() + getM11() * tupleOriginal.getY() + getM21() * tupleOriginal.getZ();
      double z = getM02() * tupleOriginal.getX() + getM12() * tupleOriginal.getY() + getM22() * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      normalize();

      if (checkIfTransformInXYPlane)
         checkIfMatrix2D();

      double x = getM00() * tupleOriginal.getX() + getM10() * tupleOriginal.getY();
      double y = getM01() * tupleOriginal.getX() + getM11() * tupleOriginal.getY();
      tupleTransformed.set(x, y);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      normalize();
      double x = getM00() * vectorOriginal.getX() + getM10() * vectorOriginal.getY() + getM20() * vectorOriginal.getZ();
      double y = getM01() * vectorOriginal.getX() + getM11() * vectorOriginal.getY() + getM21() * vectorOriginal.getZ();
      double z = getM02() * vectorOriginal.getX() + getM12() * vectorOriginal.getY() + getM22() * vectorOriginal.getZ();
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }
}

package us.ihmc.geometry.axisAngle;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.exceptions.NotAMatrix2DException;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.QuaternionTools;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public abstract class AxisAngleTools
{
   private static final double EPS = 1.0e-12;

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transformImpl(axisAngle, false, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using
    * {@code axisAngle} and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, Tuple3DReadOnly, Tuple3DBasics)} with an axis-angle that
    * has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transformImpl(axisAngle, true, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param negateAngle whether to negate the angle of the axis-angle to perform an inverse
    *           transform or not.
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   private static void transformImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double axisNorm = axisAngle.axisNorm();

      if (axisNorm < EPS)
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      double angle = axisAngle.getAngle();

      if (negateAngle)
         angle = -angle;

      double cos = Math.cos(angle);
      double oneMinusCos = 1.0 - cos;
      double sin = Math.sin(angle);

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();

      axisNorm = 1.0 / axisNorm;
      ux *= axisNorm;
      uy *= axisNorm;
      uz *= axisNorm;

      double crossX = uy * tupleOriginal.getZ() - uz * tupleOriginal.getY();
      double crossY = uz * tupleOriginal.getX() - ux * tupleOriginal.getZ();
      double crossZ = ux * tupleOriginal.getY() - uy * tupleOriginal.getX();
      double crossCrossX = uy * crossZ - uz * crossY;
      double crossCrossY = uz * crossX - ux * crossZ;
      double crossCrossZ = ux * crossY - uy * crossX;

      double x = tupleOriginal.getX() + sin * crossX + oneMinusCos * crossCrossX;
      double y = tupleOriginal.getY() + sin * crossY + oneMinusCos * crossCrossY;
      double z = tupleOriginal.getZ() + sin * crossZ + oneMinusCos * crossCrossZ;
      tupleTransformed.set(x, y, z);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the axis-angle
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the axis-angle
    *            does not represent a transformation in the XY plane.
    */
   public static void transform(AxisAngleReadOnly axisAngle, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      transformImpl(axisAngle, false, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using
    * {@code axisAngle} and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, Tuple2DReadOnly, Tuple2DBasics)} with an axis-angle that
    * has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the axis-angle
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the axis-angle
    *            does not represent a transformation in the XY plane.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                       boolean checkIfTransformInXYPlane)
   {
      transformImpl(axisAngle, true, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code axisAngle} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param negateAngle whether to negate the angle of the axis-angle to perform an inverse
    *           transform or not.
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the axis-angle
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the axis-angle
    */
   private static void transformImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                     boolean checkIfTransformInXYPlane)
   {
      if (checkIfTransformInXYPlane)
         axisAngle.checkIfIsZOnly(EPS);

      double axisNorm = axisAngle.axisNorm();

      if (axisNorm < EPS)
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      double angle = axisAngle.getAngle();

      if (negateAngle)
         angle = -angle;

      double cos = Math.cos(angle);
      double oneMinusCos = 1.0 - cos;
      double sin = Math.sin(angle);

      double uz = axisAngle.getZ();

      axisNorm = 1.0 / axisNorm;
      uz *= axisNorm;

      double crossX = -uz * tupleOriginal.getY();
      double crossY = uz * tupleOriginal.getX();
      double crossCrossX = -uz * crossY;
      double crossCrossY = uz * crossX;

      double x = tupleOriginal.getX() + sin * crossX + oneMinusCos * crossCrossX;
      double y = tupleOriginal.getY() + sin * crossY + oneMinusCos * crossCrossY;
      tupleTransformed.set(x, y);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code axisAngle} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(axisAngle) * matrixOriginal * R(axisAngle)<sup>-1</sup> <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the matrix. Not modified.
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      transformImpl(axisAngle, false, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the matrix {@code matrixOriginal} using
    * {@code axisAngle} and stores the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, Matrix3DReadOnly, Matrix3D)} with an axis-angle that has
    * an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(axisAngle)<sup>-1</sup> * matrixOriginal * R(axisAngle) <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the matrix. Not modified.
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      transformImpl(axisAngle, true, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code axisAngle} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(axisAngle) * matrixOriginal * R(axisAngle)<sup>-1</sup> <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the matrix. Not modified.
    * @param negateAngle whether to negate the angle of the axis-angle to perform an inverse
    *           transform or not.
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   private static void transformImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      double axisNorm = axisAngle.axisNorm();

      if (axisNorm < EPS)
      {
         matrixTransformed.set(matrixOriginal);
         return;
      }
      
      double cos = Math.cos(0.5 * axisAngle.getAngle());
      double sin = Math.sin(0.5 * axisAngle.getAngle()) / axisNorm;

      double qx = axisAngle.getX() * sin;
      double qy = axisAngle.getY() * sin;
      double qz = axisAngle.getZ() * sin;
      double qs = cos;

      QuaternionTools.transformImpl(qx, qy, qz, qs, negateAngle, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the quaternion {@code quaternionOriginal} using {@code axisAngle} and stores the
    * result in {@code quaternionTransformed}.
    * <p>
    * Both {@code quaternionOriginal} and {@code quaternionTransformed} can be the same object for
    * performing in place transformation.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code axisAngle} and {@code quaternionOriginal}.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the quaternion. Not modified.
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      multiplyImpl(axisAngle, false, quaternionOriginal, false, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform of the quaternion {@code quaternionOriginal} using
    * {@code axisAngle} and stores the result in {@code quaternionTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, QuaternionReadOnly, QuaternionBasics)} with an axis-angle
    * that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both {@code quaternionOriginal} and {@code quaternionTransformed} can be the same object for
    * performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the quaternion. Not modified.
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      multiplyImpl(axisAngle, true, quaternionOriginal, false, quaternionTransformed);
   }

   /**
    * Transforms the vector {@code vectorOriginal} using {@code axisAngle} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transformImpl(axisAngle, false, vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform of the vector {@code vectorOriginal} using
    * {@code axisAngle} and stores the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, Vector4DReadOnly, Vector4DBasics)} with an axis-angle
    * that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transformImpl(axisAngle, true, vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector {@code vectorOriginal} using {@code axisAngle} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the tuple. Not modified.
    * @param negateAngle whether to negate the angle of the axis-angle to perform an inverse
    *           transform or not.
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   private static void transformImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      double axisNorm = axisAngle.axisNorm();

      if (axisNorm < EPS)
      {
         vectorTransformed.set(vectorOriginal);
         return;
      }

      double angle = axisAngle.getAngle();

      if (negateAngle)
         angle = -angle;

      double cos = Math.cos(angle);
      double oneMinusCos = 1.0 - cos;
      double sin = Math.sin(angle);

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();

      axisNorm = 1.0 / axisNorm;
      ux *= axisNorm;
      uy *= axisNorm;
      uz *= axisNorm;

      double crossX = uy * vectorOriginal.getZ() - uz * vectorOriginal.getY();
      double crossY = uz * vectorOriginal.getX() - ux * vectorOriginal.getZ();
      double crossZ = ux * vectorOriginal.getY() - uy * vectorOriginal.getX();
      double crossCrossX = uy * crossZ - uz * crossY;
      double crossCrossY = uz * crossX - ux * crossZ;
      double crossCrossZ = ux * crossY - uy * crossX;

      double x = vectorOriginal.getX() + sin * crossX + oneMinusCos * crossCrossX;
      double y = vectorOriginal.getY() + sin * crossY + oneMinusCos * crossCrossY;
      double z = vectorOriginal.getZ() + sin * crossZ + oneMinusCos * crossCrossZ;
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }

   /**
    * Transforms the rotation matrix {@code rotationMatrixOriginal} using {@code axisAngle} and
    * stores the result in {@code rotationMatrixTransformed}.
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(axisAngle) * rotationMatrixOriginal <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code axisAngle} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the rotation matrix. Not modified.
    * @param rotationMatrixOriginal the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void transform(AxisAngleReadOnly axisAngle, RotationMatrixReadOnly rotationMatrixOriginal, RotationMatrix rotationMatrixTransformed)
   {
      multiplyImpl(axisAngle, false, rotationMatrixOriginal, false, rotationMatrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the rotation matrix {@code rotationMatrixOriginal}
    * using {@code axisAngle} and stores the result in {@code rotationMatrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(AxisAngleReadOnly, RotationMatrixReadOnly, RotationMatrix)} with an
    * axis-angle that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(axisAngle)<sup>-1</sup> * rotationMatrixOriginal <br>
    * where R(axisAngle) is the function to convert an axis-angle into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code axisAngle} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the rotation matrix. Not modified.
    * @param rotationMatrixOriginal the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(AxisAngleReadOnly axisAngle, RotationMatrixReadOnly rotationMatrixOriginal, RotationMatrix rotationMatrixTransformed)
   {
      multiplyImpl(axisAngle, true, rotationMatrixOriginal, false, rotationMatrixTransformed);
   }

   /**
    * Performs the multiplication of {@code axisAngle} and {@code quaternion} and stores the result
    * in {@code quaternionToPack}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both quaternions can be the same object for in place operations.
    * </p>
    *
    * @param axisAngle the axis-angle, first term in the multiplication. Not modified.
    * @param negateAngle whether to negate the angle of the axis-angle.
    * @param quaternion the quaternion, second term in the multiplication. Not modified.
    * @param conjugateQuaternion whether to conjugate the quaternion or not.
    * @param quaternionToPack the quaternion in which the result is stores. Modified.
    */
   private static void multiplyImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, QuaternionReadOnly quaternion, boolean conjugateQuaternion,
                                    QuaternionBasics quaternionTransformed)
   {
      double axisNorm = axisAngle.axisNorm();

      if (axisNorm < EPS)
      {
         quaternionTransformed.set(quaternion);
         return;
      }

      double cos = Math.cos(0.5 * axisAngle.getAngle());
      double sin = Math.sin(0.5 * axisAngle.getAngle()) / axisNorm;

      double q1x = axisAngle.getX() * sin;
      double q1y = axisAngle.getY() * sin;
      double q1z = axisAngle.getZ() * sin;
      double q1s = cos;

      double q2x = quaternion.getX();
      double q2y = quaternion.getY();
      double q2z = quaternion.getZ();
      double q2s = quaternion.getS();
      QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, negateAngle, q2x, q2y, q2z, q2s, conjugateQuaternion, quaternionTransformed);
   }

   /**
    * Performs the multiplication of {@code axisAngle} and {@code matrix} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * <b> This method is for internal use only. </b>
    * </p>
    * <p>
    * Both quaternions can be the same object for in place operations.
    * </p>
    *
    * @param axisAngle the axis-angle used to transform the rotation matrix. Not modified.
    * @param negateAngle whether to negate the angle of the axis-angle.
    * @param matrix the rotation matrix to be multiplied. Not modified.
    * @param transposeMatrix whether to conjugate the quaternion or not.
    * @param matrixToPack the rotation matrix in which the result is stores. Modified.
    */
   private static void multiplyImpl(AxisAngleReadOnly axisAngle, boolean negateAngle, RotationMatrixReadOnly matrix, boolean transposeMatrix,
                                    RotationMatrix matrixToPack)
   {
      double axisNorm = axisAngle.axisNorm();

      if (axisNorm < EPS)
      {
         matrixToPack.set(matrix);
         return;
      }

      double cos = Math.cos(0.5 * axisAngle.getAngle());
      double sin = Math.sin(0.5 * axisAngle.getAngle()) / axisNorm;

      double q1x = axisAngle.getX() * sin;
      double q1y = axisAngle.getY() * sin;
      double q1z = axisAngle.getZ() * sin;
      double q1s = cos;

      QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, negateAngle, matrix, transposeMatrix, matrixToPack);
   }
}

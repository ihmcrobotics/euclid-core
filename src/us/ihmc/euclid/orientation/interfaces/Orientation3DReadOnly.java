package us.ihmc.euclid.orientation.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public interface Orientation3DReadOnly
{
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
    *
    * @param rotationVectorToPack the rotation vector in which this orientation is to be stored.
    *           Modified.
    */
   void getRotationVector(Vector3DBasics rotationVectorToPack);

   void getYawPitchRoll(double[] yawPitchRollToPack);

   double getYaw();

   double getPitch();

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

   void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed);

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

   void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed);

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

   void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed);

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

   void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed);
}

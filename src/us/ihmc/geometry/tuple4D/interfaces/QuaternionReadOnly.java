package us.ihmc.geometry.tuple4D.interfaces;

import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.RotationVectorConversion;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.geometry.tuple4D.QuaternionTools;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public interface QuaternionReadOnly<T extends QuaternionReadOnly<T>> extends Tuple4DReadOnly<T>
{
   default boolean isNormalized(double epsilon)
   {
      double normSquared = lengthSquared();
      return !Double.isNaN(normSquared) && Math.abs(normSquared - 1.0) < epsilon;
   }

   default double getAngle()
   {
      double sinHalfTheta = Math.sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());
      return 2.0 * Math.atan2(sinHalfTheta, getS());
   }

   default void get(Vector3DBasics<?> rotationVectorToPack)
   {
      RotationVectorConversion.convertQuaternionToRotationVector(this, rotationVectorToPack);
   }

   default void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(this, yawPitchRollToPack);
   }

   default double getYaw()
   {
      return YawPitchRollConversion.computeYaw(this);
   }

   default double getPitch()
   {
      return YawPitchRollConversion.computePitch(this);
   }

   default double getRoll()
   {
      return YawPitchRollConversion.computeRoll(this);
   }

   default void transform(Tuple3DBasics<?> tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   default void transform(Tuple3DReadOnly<?> tupleOriginal, Tuple3DBasics<?> tupleTransformed)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }
   
   default void transform(Tuple2DBasics<?> tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform, true);
   }

   default void transform(Tuple2DReadOnly<?> tupleOriginal, Tuple2DBasics<?> tupleTransformed)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, true);
   }

   default void transform(Tuple2DBasics<?> tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   default void transform(Tuple2DReadOnly<?> tupleOriginal, Tuple2DBasics<?> tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   default void transform(QuaternionBasics<?> quaternionToTransform)
   {
      transform(quaternionToTransform, quaternionToTransform);
   }

   default void transform(QuaternionReadOnly<?> quaternionOriginal, QuaternionBasics<?> quaternionTransformed)
   {
      QuaternionTools.transform(this, quaternionOriginal, quaternionTransformed);
   }

   default void transform(Vector4DBasics<?> vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   default void transform(Vector4DReadOnly<?> vectorOriginal, Vector4DBasics<?> vectorTransformed)
   {
      QuaternionTools.transform(this, vectorOriginal, vectorTransformed);
   }

   default void transform(Matrix3D matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   default void transform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      QuaternionTools.transform(this, matrixOriginal, matrixTransformed);
   }

   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   default void transform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTransformed)
   {
      QuaternionTools.transform(this, matrixOriginal, matrixTransformed);
   }

   default void inverseTransform(Tuple3DBasics<?> tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   default void inverseTransform(Tuple3DReadOnly<?> tupleOriginal, Tuple3DBasics<?> tupleTransformed)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   default void inverseTransform(QuaternionBasics<?> quaternionToTransform)
   {
      inverseTransform(quaternionToTransform, quaternionToTransform);
   }

   default void inverseTransform(QuaternionReadOnly<?> quaternionOriginal, QuaternionBasics<?> quaternionTransformed)
   {
      QuaternionTools.inverseTransform(this, quaternionOriginal, quaternionTransformed);
   }

   default void inverseTransform(Vector4DBasics<?> vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   default void inverseTransform(Vector4DReadOnly<?> vectorOriginal, Vector4DBasics<?> vectorTransformed)
   {
      QuaternionTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(Matrix3D matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   default void inverseTransform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      QuaternionTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   default void inverseTransform(RotationMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   default void inverseTransform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTransformed)
   {
      QuaternionTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   default void inverseTransform(Tuple2DBasics<?> tupleToTransform)
   {
      inverseTransform(tupleToTransform, true);
   }

   default void inverseTransform(Tuple2DReadOnly<?> tupleOriginal, Tuple2DBasics<?> tupleTransformed)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed, true);
   }

   default void inverseTransform(Tuple2DBasics<?> tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   default void inverseTransform(Tuple2DReadOnly<?> tupleOriginal, Tuple2DBasics<?> tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }
}
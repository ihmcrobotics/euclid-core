package us.ihmc.euclid.tools;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class RotationMatrixToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPS = 1.0e-12;

   @Test
   public void testApplyYawRotation() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      { // test applyYawRotation(double yaw, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         RotationMatrix rotationMatrix = new RotationMatrix();
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         rotationMatrix.setToYawMatrix(yaw);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);

         Tuple3DBasics expectedTuple = new Vector3D();
         Tuple3DBasics actualTuple = new Vector3D();

         rotationMatrix.transform(tupleOriginal, expectedTuple);
         RotationMatrixTools.applyYawRotation(yaw, tupleOriginal, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, EPS);

         actualTuple.set(tupleOriginal);
         RotationMatrixTools.applyYawRotation(yaw, actualTuple, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // test applyYawRotation(double yaw, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
         RotationMatrix rotationMatrix = new RotationMatrix();
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         rotationMatrix.setToYawMatrix(yaw);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.generateRandomPoint2D(random, 10.0);

         Tuple2DBasics expectedTuple = new Vector2D();
         Tuple2DBasics actualTuple = new Vector2D();

         rotationMatrix.transform(tupleOriginal, expectedTuple);
         RotationMatrixTools.applyYawRotation(yaw, tupleOriginal, actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, EPS);

         actualTuple.set(tupleOriginal);
         RotationMatrixTools.applyYawRotation(yaw, actualTuple, actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, EPS);
      }
   }

   @Test
   public void testApplyPitchRotation() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      { // test applyPitchRotation(double pitch, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         RotationMatrix rotationMatrix = new RotationMatrix();
         double pitch = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         rotationMatrix.setToPitchMatrix(pitch);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);

         Tuple3DBasics expectedTuple = new Vector3D();
         Tuple3DBasics actualTuple = new Vector3D();

         rotationMatrix.transform(tupleOriginal, expectedTuple);
         RotationMatrixTools.applyPitchRotation(pitch, tupleOriginal, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, EPS);

         actualTuple.set(tupleOriginal);
         RotationMatrixTools.applyPitchRotation(pitch, actualTuple, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, EPS);
      }
   }

   @Test
   public void testApplyRollRotation() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      { // test applyRollRotation(double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         RotationMatrix rotationMatrix = new RotationMatrix();
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         rotationMatrix.setToRollMatrix(roll);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);

         Tuple3DBasics expectedTuple = new Vector3D();
         Tuple3DBasics actualTuple = new Vector3D();

         rotationMatrix.transform(tupleOriginal, expectedTuple);
         RotationMatrixTools.applyRollRotation(roll, tupleOriginal, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, EPS);

         actualTuple.set(tupleOriginal);
         RotationMatrixTools.applyRollRotation(roll, actualTuple, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, EPS);
      }
   }
}

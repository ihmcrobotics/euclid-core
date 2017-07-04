package us.ihmc.euclid.tools;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

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

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(74232);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix r1 = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix r2 = EuclidCoreRandomTools.generateRandomRotationMatrix(random);

         Quaternion q1 = new Quaternion(r1);
         Quaternion q2 = new Quaternion(r2);

         double alpha = random.nextDouble();
         Quaternion qInterpolated = new Quaternion();
         qInterpolated.interpolate(q1, q2, alpha);
         RotationMatrix expectedMatrix = new RotationMatrix(qInterpolated);

         RotationMatrix actualMatrix = new RotationMatrix();
         RotationMatrixTools.interpolate(r1, r2, alpha, actualMatrix);

         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPS);
      }

      {
         double errorAverage = 0.0;

         for (int i = 0; i < ITERATIONS; i++)
         {// Test with singularities
            AxisAngle axisAngle = new AxisAngle(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0), Math.PI);
            RotationMatrix diff = new RotationMatrix(axisAngle);
            RotationMatrix r1 = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
            RotationMatrix r2 = new RotationMatrix();
            r2.set(r1);
            r2.multiply(diff);

            double alpha = random.nextDouble();

            axisAngle.scaleAngle(alpha);
            diff.set(axisAngle);

            RotationMatrix expectedMatrix = new RotationMatrix();
            expectedMatrix.set(r1);
            expectedMatrix.multiply(diff);

            RotationMatrix actualMatrix = new RotationMatrix();
            RotationMatrixTools.interpolate(r1, r2, alpha, actualMatrix);

            if (!expectedMatrix.epsilonEquals(actualMatrix, EPS))
            {
               axisAngle.scaleAngle(-1.0);
               diff.set(axisAngle);
               expectedMatrix.set(r1);
               expectedMatrix.multiply(diff);

               EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPS);
            }

            for (int row = 0; row < 3; row ++)
               for (int column = 0; column < 3; column ++)
                  errorAverage += Math.abs(expectedMatrix.getElement(row, column) - actualMatrix.getElement(row, column));
         }

         errorAverage /= 9.0 * ITERATIONS;
         assertTrue(errorAverage < 5.0e-5);
      }
   }
}

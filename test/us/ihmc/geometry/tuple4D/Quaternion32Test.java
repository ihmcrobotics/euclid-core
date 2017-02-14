package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.Vector3D;

public class Quaternion32Test extends QuaternionBasicsTest<Quaternion32>
{
   public static final int NUMBER_OF_ITERATIONS = QuaternionTest.NUMBER_OF_ITERATIONS;
   public static final float EPS = (float) 1e-6;

   @Test
   public void testQuaternion32()
   {
      Random random = new Random(613615L);
      Quaternion32 quaternion = new Quaternion32();
      Quaternion32 quaternionCopy;
      Quaternion32 expected = new Quaternion32();

      { // Test Quaternion32()
         expected.setToZero();
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion32(QuaternionBasics other)
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         Quaternion32 quaternion2 = new Quaternion32(quaternion);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternion2, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion32(float[] quaternionArray)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         float[] quaternionArray;
         quaternionArray = new float[] {expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32()};

         quaternion = new Quaternion32(quaternionArray);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion32(RotationMatrix rotationMatrix) about X-axis
         RotationMatrix rotationMatrix, rotationMatrixCopy;
         rotationMatrix = rotationMatrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         quaternion = new Quaternion32(rotationMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion32(VectorBasics rotationVector)
         Vector3D rotationVector, rotationVectorCopy;
         rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

         quaternion = new Quaternion32(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(double x, double y, double z, double s)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion = new Quaternion32(expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32());

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Quaternion32 q = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

      int newHashCode, previousHashCode;
      newHashCode = q.hashCode();
      assertEquals(newHashCode, q.hashCode());

      previousHashCode = q.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float qx = q.getX32();
         float qy = q.getY32();
         float qz = q.getZ32();
         float qs = q.getS32();
         switch (random.nextInt(4))
         {
         case 0:
            qx = random.nextFloat();
            break;
         case 1:
            qy = random.nextFloat();
            break;
         case 2:
            qz = random.nextFloat();
            break;
         case 3:
            qs = random.nextFloat();
            break;
         }
         q.setUnsafe(qx, qy, qz, qs);
         newHashCode = q.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public Quaternion32 createEmptyTuple()
   {
      return new Quaternion32();
   }

   @Override
   public Quaternion32 createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomQuaternion32(random);
   }

   @Override
   public Quaternion32 createTuple(double x, double y, double z, double s)
   {
      Quaternion32 quaternion = new Quaternion32();
      quaternion.setUnsafe(x, y, z, s);
      return quaternion;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-6;
   }
}

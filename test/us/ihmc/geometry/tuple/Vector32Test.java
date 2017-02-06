package us.ihmc.geometry.tuple;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.tuple3D.Tuple32;
import us.ihmc.geometry.tuple3D.Vector32;

public class Vector32Test extends Tuple32Test
{
   @Test
   public void testVector32()
   {
      Random random = new Random(621541L);
      Vector32 vector = new Vector32();

      { // Test Vector32()
         Assert.assertTrue(0 == vector.getX32());
         Assert.assertTrue(0 == vector.getY32());
         Assert.assertTrue(0 == vector.getZ32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector32(float x, float y, float z)
         float newX = random.nextFloat();
         float newY = random.nextFloat();
         float newZ = random.nextFloat();

         vector = new Vector32(newX, newY, newZ);

         Assert.assertTrue(newX == vector.getX32());
         Assert.assertTrue(newY == vector.getY32());
         Assert.assertTrue(newZ == vector.getZ32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector32(float[] vectorArray)
         float[] randomVector32Array = {random.nextFloat(), random.nextFloat(), random.nextFloat()};
         float[] copyRandomVector32Array = new float[3];
         copyRandomVector32Array[0] = randomVector32Array[0];
         copyRandomVector32Array[1] = randomVector32Array[1];
         copyRandomVector32Array[2] = randomVector32Array[2];

         Vector32 vectorArray = new Vector32(randomVector32Array);

         Assert.assertTrue(randomVector32Array[0] == vectorArray.getX32());
         Assert.assertTrue(randomVector32Array[1] == vectorArray.getY32());
         Assert.assertTrue(randomVector32Array[2] == vectorArray.getZ32());

         Assert.assertTrue(copyRandomVector32Array[0] == randomVector32Array[0]);
         Assert.assertTrue(copyRandomVector32Array[1] == randomVector32Array[1]);
         Assert.assertTrue(copyRandomVector32Array[2] == randomVector32Array[2]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector32(TupleBasics tuple)
         Vector32 vector2 = GeometryBasicsRandomTools.generateRandomVector32(random);

         vector = new Vector32(vector2);

         Assert.assertTrue(vector.getX32() == vector2.getX32());
         Assert.assertTrue(vector.getY32() == vector2.getY32());
         Assert.assertTrue(vector.getZ32() == vector2.getZ32());
      }
   }

   @Test
   public void testSet()
   {
      Random random = new Random(65465131L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float newX = random.nextFloat();
         float newY = random.nextFloat();
         float newZ = random.nextFloat();

         Vector32 vector = new Vector32(newX, newY, newZ);

         Vector32 vector2 = new Vector32();
         vector2.set(vector);

         Assert.assertTrue(vector.getX32() == vector2.getX32());
         Assert.assertTrue(vector.getY32() == vector2.getY32());
         Assert.assertTrue(vector.getZ32() == vector2.getZ32());
      }
   }

   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);
      Vector32 vector1 = new Vector32();
      Vector32 vector2 = new Vector32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple(random, vector1);
         GeometryBasicsRandomTools.randomizeTuple(random, vector2);

         float actualAngle = vector1.angle(vector2);
         float dotProduct = vector1.dot(vector2);
         float magnitudeVector321 = vector1.length();
         float magnitudeVector322 = vector2.length();
         float expectedAngle = (float) Math.acos(dotProduct / (magnitudeVector321 * magnitudeVector322));

         Assert.assertTrue(actualAngle == expectedAngle);
      }
   }

   @Test
   public void testCross()
   {
      Random random = new Random(5751684L);
      Vector32 vector1 = new Vector32();
      Vector32 vector2 = new Vector32();
      Vector32 vector3 = new Vector32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple(random, vector1);
         GeometryBasicsRandomTools.randomizeTuple(random, vector2);

         vector3.cross(vector1, vector2);

         float x = vector1.getY32() * vector2.getZ32() - vector1.getZ32() * vector2.getY32();
         float y = vector2.getX32() * vector1.getZ32() - vector2.getZ32() * vector1.getX32();
         float z = vector1.getX32() * vector2.getY32() - vector1.getY32() * vector2.getX32();

         float magnitudeVector1 = vector1.length();
         float magnitudeVector2 = vector2.length();
         float actualMagnitudeVector3 = vector3.length();

         float theta = (float) Math.acos(vector1.dot(vector2) / (magnitudeVector1 * magnitudeVector2));
         float expectedMagnitudeVector3 = magnitudeVector1 * magnitudeVector2 * (float) Math.sin(theta);
         float vector3DotVector1 = vector3.dot(vector1);
         float vector3DotVector2 = vector3.dot(vector2);

         Assert.assertEquals(expectedMagnitudeVector3, actualMagnitudeVector3, 1e-4);
         Assert.assertEquals(vector3DotVector1, 0, 1e-6);
         Assert.assertEquals(vector3DotVector2, 0, 1e-6);

         Assert.assertTrue(vector3.getX32() == x);
         Assert.assertTrue(vector3.getY32() == y);
         Assert.assertTrue(vector3.getZ32() == z);
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(615651L);
      Vector32 vector1 = new Vector32();
      Vector32 vector2 = new Vector32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple(random, vector1);
         GeometryBasicsRandomTools.randomizeTuple(random, vector2);
         float actualDot = vector1.dot(vector2);
         float expectedDot = vector1.getX32() * vector2.getX32() + vector1.getY32() * vector2.getY32() + vector1.getZ32() * vector2.getZ32();

         Assert.assertTrue(actualDot == expectedDot);
      }
   }

   @Test
   public void testLength()
   {
      Random random = new Random(312310L);
      Vector32 vector = new Vector32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple(random, vector);

         float length = vector.length();
         float expectedLengthSquared = vector.getX32() * vector.getX32() + vector.getY32() * vector.getY32() + vector.getZ32() * vector.getZ32();
         float expectedLength = (float) Math.sqrt(expectedLengthSquared);

         Assert.assertTrue(length == expectedLength);
      }
   }

   @Test
   public void testLengthSquared()
   {
      Random random = new Random(312310L);
      Vector32 vector = new Vector32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple(random, vector);

         float lengthSquared = vector.lengthSquared();
         float expectedLengthSquared = vector.getX32() * vector.getX32() + vector.getY32() * vector.getY32() + vector.getZ32() * vector.getZ32();

         Assert.assertTrue(lengthSquared == expectedLengthSquared);
      }
   }

   @Test
   public void testNormalize()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize()
         Vector32 vector = GeometryBasicsRandomTools.generateRandomVector32(random);

         vector.normalize();

         float expectedLength = 1;
         float normalLength = vector.length();
         float difference = Math.abs(expectedLength - normalLength);

         Assert.assertTrue(difference < 1e-6);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize(Vector32 vector)
         Vector32 vector = GeometryBasicsRandomTools.generateRandomVector32(random);

         Vector32 normal = new Vector32();

         normal.setAndNormalize(vector);

         float expectedLength = 1;
         float actualLength = normal.length();
         float difference = Math.abs(expectedLength - actualLength);

         Assert.assertTrue(difference < 1e-6);
      }
   }

   @Ignore
   @Test
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testEpsilonEquals()
   {
      Random random = new Random(621541L);
      Vector32 vector1 = new Vector32();
      Vector32 vector2 = new Vector32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float epsilon = random.nextFloat();

         GeometryBasicsRandomTools.randomizeTuple(random, vector1);

         vector2.setX(vector1.getX32() + 0.1f * epsilon);
         vector2.setY(vector1.getY32() + 0.1f * epsilon);
         vector2.setZ(vector1.getZ32() + 0.1f * epsilon);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() - 0.1f * epsilon);
         vector2.setY(vector1.getY32() - 0.1f * epsilon);
         vector2.setZ(vector1.getZ32() - 0.1f * epsilon);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() - 1.1f * epsilon);
         vector2.setY(vector1.getY32() - 0.1f * epsilon);
         vector2.setZ(vector1.getZ32() - 0.1f * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() - 0.1f * epsilon);
         vector2.setY(vector1.getY32() - 1.1f * epsilon);
         vector2.setZ(vector1.getZ32() - 0.1f * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() - 0.1f * epsilon);
         vector2.setY(vector1.getY32() - 0.1f * epsilon);
         vector2.setZ(vector1.getZ32() - 1.1f * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() + 1.1f * epsilon);
         vector2.setY(vector1.getY32() + 0.1f * epsilon);
         vector2.setZ(vector1.getZ32() + 0.1f * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() + 0.1f * epsilon);
         vector2.setY(vector1.getY32() + 1.1f * epsilon);
         vector2.setZ(vector1.getZ32() + 0.1f * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() + 0.1f * epsilon);
         vector2.setY(vector1.getY32() + 0.1f * epsilon);
         vector2.setZ(vector1.getZ32() + 1.1f * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() + epsilon - (float) 1.0e-7);
         vector2.setY(vector1.getY32() + epsilon - (float) 1.0e-7);
         vector2.setZ(vector1.getZ32() + epsilon - (float) 1.0e-7);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX32() - epsilon + (float) 1.0e-7);
         vector2.setY(vector1.getY32() - epsilon + (float) 1.0e-7);
         vector2.setZ(vector1.getZ32() - epsilon + (float) 1.0e-7);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));
      }
   }

   @Override
   public Tuple32 createEmptyTuple32()
   {
      return new Vector32();
   }
}

package us.ihmc.geometry.tuple2D;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DReadOnly;

public class Vector2D32Test extends Tuple2D32Test<Vector2D32>
{
   @Test
   public void testVector2D32()
   {
      Random random = new Random(621541L);
      Vector2D32 vector = new Vector2D32();

      { // Test Vector2D32()
         Assert.assertTrue(0 == vector.getX32());
         Assert.assertTrue(0 == vector.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D32(float x, float y, float z)
         float newX = random.nextFloat();
         float newY = random.nextFloat();

         vector = new Vector2D32(newX, newY);

         Assert.assertTrue(newX == vector.getX32());
         Assert.assertTrue(newY == vector.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D32(float[] vectorArray)
         float[] randomVector2DArray = {random.nextFloat(), random.nextFloat()};
         float[] copyRandomVector2DArray = new float[2];
         copyRandomVector2DArray[0] = randomVector2DArray[0];
         copyRandomVector2DArray[1] = randomVector2DArray[1];

         Vector2D32 vectorArray = new Vector2D32(randomVector2DArray);

         Assert.assertTrue(randomVector2DArray[0] == vectorArray.getX32());
         Assert.assertTrue(randomVector2DArray[1] == vectorArray.getY32());

         Assert.assertTrue(copyRandomVector2DArray[0] == randomVector2DArray[0]);
         Assert.assertTrue(copyRandomVector2DArray[1] == randomVector2DArray[1]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D32(TupleBasics tuple)
         Vector2D32 vector2 = GeometryBasicsRandomTools.generateRandomVector2D32(random);

         vector = new Vector2D32((Tuple2DReadOnly<?>) vector2);

         Assert.assertTrue(vector.getX32() == vector2.getX32());
         Assert.assertTrue(vector.getY32() == vector2.getY32());
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

         Vector2D32 vector = new Vector2D32(newX, newY);

         Vector2D32 vector2 = new Vector2D32();
         vector2.set(vector);

         Assert.assertTrue(vector.getX32() == vector2.getX32());
         Assert.assertTrue(vector.getY32() == vector2.getY32());
      }
   }

   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);
      Vector2D32 vector1 = new Vector2D32();
      Vector2D32 vector2 = new Vector2D32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple2D(random, vector1);
         GeometryBasicsRandomTools.randomizeTuple2D(random, vector2);

         double actualAngle = vector1.angle(vector2);
         double dotProduct = vector1.dot(vector2);
         double magnitudeVector2D321 = vector1.length();
         double magnitudeVector2D322 = vector2.length();
         double expectedAngle = Math.acos(dotProduct / (magnitudeVector2D321 * magnitudeVector2D322));

         Assert.assertTrue(actualAngle == expectedAngle);
      }
   }

   @Test
   public void testCross()
   {
      Random random = new Random(5751684L);
      Vector2D32 vector1 = new Vector2D32();
      Vector2D32 vector2 = new Vector2D32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple2D(random, vector1);
         GeometryBasicsRandomTools.randomizeTuple2D(random, vector2);

         { // Test cross(Vector2D32Basics other)
            double actualMagnitudeVector3 = vector1.cross(vector2);
            double expectedMagnitudeVector3 = vector1.getX32() * vector2.getY32() - vector1.getY32() * vector2.getX32();

            Assert.assertEquals(expectedMagnitudeVector3, actualMagnitudeVector3, 1e-7);
         }

         { // Test cross(Vector2D32Basics v1, Vector2D32Basics v2)
            double actualMagnitudeVector3 = Vector2DReadOnly.cross(vector1, vector2);
            double expectedMagnitudeVector3 = vector1.getX32() * vector2.getY32() - vector1.getY32() * vector2.getX32();

            Assert.assertEquals(expectedMagnitudeVector3, actualMagnitudeVector3, 1e-7);
         }
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(615651L);
      Vector2D32 vector1 = new Vector2D32();
      Vector2D32 vector2 = new Vector2D32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple2D(random, vector1);
         GeometryBasicsRandomTools.randomizeTuple2D(random, vector2);

         double actualDot = vector1.dot(vector2);
         double expectedDot = vector1.getX32() * vector2.getX32() + vector1.getY32() * vector2.getY32();

         Assert.assertEquals(expectedDot, actualDot, 1e-6);
      }
   }

   @Test
   public void testLength()
   {
      Random random = new Random(312310L);
      Vector2D32 vector = new Vector2D32();

      GeometryBasicsRandomTools.randomizeTuple2D(random, vector);

      double length = vector.length();
      double expectedLengthSquared = vector.getX() * vector.getX() + vector.getY() * vector.getY();
      double expectedLength = Math.sqrt(expectedLengthSquared);

      Assert.assertTrue(length == expectedLength);
   }

   @Test
   public void testLengthSquared()
   {
      Random random = new Random(312310L);
      Vector2D32 vector = new Vector2D32();

      GeometryBasicsRandomTools.randomizeTuple2D(random, vector);

      double lengthSquared = vector.lengthSquared();
      double expectedLengthSquared = vector.getX() * vector.getX() + vector.getY() * vector.getY();

      Assert.assertTrue(lengthSquared == expectedLengthSquared);
   }

   @Test
   public void testNormalize()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize()
         Vector2D32 vector = GeometryBasicsRandomTools.generateRandomVector2D32(random);

         vector.normalize();

         double expectedLength = 1;
         double normalLength = vector.length();
         double difference = Math.abs(expectedLength - normalLength);

         Assert.assertTrue(difference < 1e-6);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize(Vector2D32 vector)
         Vector2D32 vector = GeometryBasicsRandomTools.generateRandomVector2D32(random);

         Vector2D32 normal = new Vector2D32();

         normal.setAndNormalize(vector);

         double expectedLength = 1;
         double actualLength = normal.length();
         double difference = Math.abs(expectedLength - actualLength);

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
      float epsilon = random.nextFloat();

      Vector2D32 v1 = GeometryBasicsRandomTools.generateRandomVector2D32(random);
      Vector2D32 v2 = new Vector2D32(v1);

      assertTrue(v1.epsilonEquals(v2, epsilon));

      for (int index = 0; index < 2; index++)
      {
         v2.set(v1);
         v2.set(index, v1.get(index) + 0.999f * epsilon);
         assertTrue(v1.epsilonEquals(v2, epsilon));

         v2.set(v1);
         v2.set(index, v1.get(index) - 0.999f * epsilon);
         assertTrue(v1.epsilonEquals(v2, epsilon));

         v2.set(v1);
         v2.set(index, v1.get(index) + 1.001f * epsilon);
         assertFalse(v1.epsilonEquals(v2, epsilon));

         v2.set(v1);
         v2.set(index, v1.get(index) - 1.001f * epsilon);
         assertFalse(v1.epsilonEquals(v2, epsilon));
      }
   }

   @Override
   public Vector2D32 createEmptyTuple()
   {
      return new Vector2D32();
   }

   @Override
   public Vector2D32 createTuple(double x, double y)
   {
      return new Vector2D32((float) x, (float) y);
   }

   @Override
   public Vector2D32 createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomVector2D32(random);
   }
}

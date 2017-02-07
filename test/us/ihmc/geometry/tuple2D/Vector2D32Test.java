package us.ihmc.geometry.tuple2D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public class Vector2D32Test extends Vector2DBasicsTest<Vector2D32>
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

         Vector2D32 vectorArray = new Vector2D32(randomVector2DArray);

         Assert.assertTrue(randomVector2DArray[0] == vectorArray.getX32());
         Assert.assertTrue(randomVector2DArray[1] == vectorArray.getY32());
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
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point2D32 point = GeometryBasicsRandomTools.generateRandomPoint2D32(random);

      int newHashCode, previousHashCode;
      newHashCode = point.hashCode();
      assertEquals(newHashCode, point.hashCode());

      previousHashCode = point.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         point.set(i % 2, random.nextFloat());
         newHashCode = point.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-6;
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

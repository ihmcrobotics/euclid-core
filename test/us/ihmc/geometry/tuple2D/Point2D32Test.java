package us.ihmc.geometry.tuple2D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;

public class Point2D32Test extends Point2DBasicsTest<Point2D32>
{
   @Test
   public void testPoint2D32()
   {
      Random random = new Random(621541L);
      Point2D32 point = new Point2D32();

      { // Test Point2D32()
         Assert.assertTrue(0f == point.getX32());
         Assert.assertTrue(0f == point.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D32(float x, float y)
         float newX = random.nextFloat();
         float newY = random.nextFloat();

         point = new Point2D32(newX, newY);

         Assert.assertTrue(newX == point.getX32());
         Assert.assertTrue(newY == point.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D32(float[] pointArray)
         float[] randomPoint2D32Array = {random.nextFloat(), random.nextFloat()};
         float[] copyRandomPoint2D32Array = new float[2];
         copyRandomPoint2D32Array[0] = randomPoint2D32Array[0];
         copyRandomPoint2D32Array[1] = randomPoint2D32Array[1];

         Point2D32 pointArray = new Point2D32(randomPoint2D32Array);

         Assert.assertTrue(randomPoint2D32Array[0] == pointArray.getX32());
         Assert.assertTrue(randomPoint2D32Array[1] == pointArray.getY32());

         Assert.assertTrue(copyRandomPoint2D32Array[0] == randomPoint2D32Array[0]);
         Assert.assertTrue(copyRandomPoint2D32Array[1] == randomPoint2D32Array[1]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D32(TupleBasics tuple)
         Point2D32 point2 = GeometryBasicsRandomTools.generateRandomPoint2D32(random);

         point = new Point2D32(point2);

         Assert.assertTrue(point.getX32() == point2.getX32());
         Assert.assertTrue(point.getY32() == point2.getY32());
      }
   }

   @Override
   public void testSetters() throws Exception
   {
      super.testSetters();

      Random random = new Random(621541L);
      Point2D32 tuple1 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setX(float x)
         float x = random.nextFloat();
         tuple1.setX(x);
         assertEquals(tuple1.getX32(), x, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setY(float y)
         float y = random.nextFloat();
         tuple1.setY(y);
         assertEquals(tuple1.getY32(), y, getEpsilon());
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
   public Point2D32 createEmptyTuple()
   {
      return new Point2D32();
   }

   @Override
   public Point2D32 createTuple(double x, double y)
   {
      return new Point2D32((float) x, (float) y);
   }

   @Override
   public Point2D32 createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomPoint2D32(random);
   }
}

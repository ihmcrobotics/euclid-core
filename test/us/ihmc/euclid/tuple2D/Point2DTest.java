package us.ihmc.euclid.tuple2D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;

public class Point2DTest extends Point2DBasicsTest<Point2D>
{
   @Test
   public void testPoint2D()
   {
      Random random = new Random(621541L);
      Point2D point = new Point2D();

      { // Test Point2D()
         Assert.assertTrue(0 == point.getX());
         Assert.assertTrue(0 == point.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D(double x, double y)
         double newX = random.nextDouble();
         double newY = random.nextDouble();

         point = new Point2D(newX, newY);

         Assert.assertTrue(newX == point.getX());
         Assert.assertTrue(newY == point.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D(double[] pointArray)
         double[] randomPoint2DArray = {random.nextDouble(), random.nextDouble()};
         Point2D pointArray = new Point2D(randomPoint2DArray);

         Assert.assertTrue(randomPoint2DArray[0] == pointArray.getX());
         Assert.assertTrue(randomPoint2DArray[1] == pointArray.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D(TupleBasics tuple)
         Point2D point2 = createRandomTuple(random);
         point = new Point2D(point2);

         Assert.assertTrue(point.getX() == point2.getX());
         Assert.assertTrue(point.getY() == point2.getY());
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point2D tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.setElement(i % 2, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }

   @Override
   public Point2D createEmptyTuple()
   {
      return new Point2D();
   }

   @Override
   public Point2D createTuple(double x, double y)
   {
      return new Point2D(x, y);
   }

   @Override
   public Point2D createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.generateRandomPoint2D(random);
   }
}

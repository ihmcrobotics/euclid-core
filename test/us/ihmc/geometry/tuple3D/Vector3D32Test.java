package us.ihmc.geometry.tuple3D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;

public class Vector3D32Test extends Vector3DBasicsTest<Vector3D32>
{
   @Test
   public void testVector32()
   {
      Random random = new Random(621541L);
      Vector3D32 vector = new Vector3D32();

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

         vector = new Vector3D32(newX, newY, newZ);

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

         Vector3D32 vectorArray = new Vector3D32(randomVector32Array);

         Assert.assertTrue(randomVector32Array[0] == vectorArray.getX32());
         Assert.assertTrue(randomVector32Array[1] == vectorArray.getY32());
         Assert.assertTrue(randomVector32Array[2] == vectorArray.getZ32());

         Assert.assertTrue(copyRandomVector32Array[0] == randomVector32Array[0]);
         Assert.assertTrue(copyRandomVector32Array[1] == randomVector32Array[1]);
         Assert.assertTrue(copyRandomVector32Array[2] == randomVector32Array[2]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector32(TupleBasics tuple)
         Vector3D32 vector2 = GeometryBasicsRandomTools.generateRandomVector3D32(random);

         vector = new Vector3D32(vector2);

         Assert.assertTrue(vector.getX32() == vector2.getX32());
         Assert.assertTrue(vector.getY32() == vector2.getY32());
         Assert.assertTrue(vector.getZ32() == vector2.getZ32());
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Vector3D32 tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.set(i % 3, random.nextFloat());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public Vector3D32 createEmptyTuple()
   {
      return new Vector3D32();
   }

   @Override
   public Vector3D32 createTuple(double x, double y, double z)
   {
      return new Vector3D32((float) x, (float) y, (float) z);
   }

   @Override
   public Vector3D32 createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomVector3D32(random);
   }

   @Override
   public double getEpsilon()
   {
      return 2.0e-7;
   }
}

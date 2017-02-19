package us.ihmc.euclid.tuple4D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public class Vector4DTest extends Vector4DBasicsTest<Vector4D>
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(3453L);

      { // Test empty constructor
         Vector4D vector = new Vector4D();
         assertTrue(vector.getX() == 0.0);
         assertTrue(vector.getY() == 0.0);
         assertTrue(vector.getZ() == 0.0);
         assertTrue(vector.getS() == 0.0);
      }

      { // Test Vector4D(double x, double y, double z, double s)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         Vector4D vector = new Vector4D(x, y, z, s);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test Vector4D(double[] vectorArray)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         double[] vectorArray = {x, y, z, s};
         Vector4D vector = new Vector4D(vectorArray);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test Vector4D(QuaternionReadOnly quaternion)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector4D vector = new Vector4D(quaternion);
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D(Tuple4DReadOnly other)
         Tuple4DReadOnly quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector4D vector = new Vector4D(quaternion);
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D(Vector3DReadOnly vector3D)
         Vector3DReadOnly vector3D = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector4D vector = new Vector4D(vector3D);
         for (int i = 0; i < 3; i++)
            assertTrue(vector.getElement(i) == vector3D.get(i));
         assertTrue(vector.getS() == 0.0);
      }

      { // Test Vector4D(Point3DReadOnly vector3D)
         Point3DReadOnly point3D = EuclidCoreRandomTools.generateRandomPoint3D(random);
         Vector4D vector = new Vector4D(point3D);
         for (int i = 0; i < 3; i++)
            assertTrue(vector.getElement(i) == point3D.get(i));
         assertTrue(vector.getS() == 1.0);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Vector4D vector = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = vector.hashCode();
      assertEquals(newHashCode, vector.hashCode());

      previousHashCode = vector.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector.setElement(i % 4, random.nextDouble());
         newHashCode = vector.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public Vector4D createEmptyTuple()
   {
      return new Vector4D();
   }

   @Override
   public Vector4D createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.generateRandomVector4D(random);
   }

   @Override
   public Vector4D createTuple(double x, double y, double z, double s)
   {
      return new Vector4D(x, y, z, s);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }
}

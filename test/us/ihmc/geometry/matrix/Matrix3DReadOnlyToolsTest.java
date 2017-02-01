package us.ihmc.geometry.matrix;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;

public class Matrix3DReadOnlyToolsTest
{

   @Test
   public void testToString() throws Exception
   {
      Random random = new Random(32456L);

      String stringExpected = "0.0, 1.0, 2.0\n3.0, 4.0, 5.0\n6.0, 7.0, 8.0\n";
      String stringActual = Matrix3DReadOnlyTools.toString(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
      assertEquals(stringExpected, stringActual);

      Matrix3D m = GeometryBasicsRandomTools.generateRandomDiagonalMatrix3D(random);
      stringExpected = Matrix3DReadOnlyTools.toString(m.getM00(), m.getM01(), m.getM02(), m.getM10(), m.getM11(), m.getM12(), m.getM20(), m.getM21(), m.getM22());
      stringActual = Matrix3DReadOnlyTools.toString(m);
      assertEquals(stringExpected, stringActual);
   }
}

package us.ihmc.euclid.testSuite;

import java.awt.Desktop;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import org.pitest.mutationtest.commandline.MutationCoverageReport;

import us.ihmc.euclid.axisAngle.AxisAngle32Test;
import us.ihmc.euclid.axisAngle.AxisAngleTest;
import us.ihmc.euclid.matrix.Matrix3DTest;
import us.ihmc.euclid.matrix.RotationMatrixTest;
import us.ihmc.euclid.matrix.RotationScaleMatrixTest;
import us.ihmc.euclid.rotationConversion.AxisAngleConversionTest;
import us.ihmc.euclid.rotationConversion.CyclingConversionTest;
import us.ihmc.euclid.rotationConversion.QuaternionConversionTest;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversionTest;
import us.ihmc.euclid.rotationConversion.RotationVectorConversionTest;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversionTest;
import us.ihmc.euclid.tools.EuclidCoreTestToolsTest;
import us.ihmc.euclid.tools.EuclidCoreToolsTest;
import us.ihmc.euclid.tools.Matrix3DFeaturesTest;
import us.ihmc.euclid.tools.Matrix3DToolsTest;
import us.ihmc.euclid.tools.QuaternionToolsTest;
import us.ihmc.euclid.tools.RotationMatrixToolsTest;
import us.ihmc.euclid.tools.RotationScaleMatrixToolsTest;
import us.ihmc.euclid.tools.TupleToolsTest;
import us.ihmc.euclid.transform.AffineTransformTest;
import us.ihmc.euclid.transform.QuaternionBasedTransformTest;
import us.ihmc.euclid.transform.RigidBodyTransformTest;
import us.ihmc.euclid.tuple2D.Point2D32Test;
import us.ihmc.euclid.tuple2D.Point2DTest;
import us.ihmc.euclid.tuple2D.Vector2D32Test;
import us.ihmc.euclid.tuple2D.Vector2DTest;
import us.ihmc.euclid.tuple3D.Point3D32Test;
import us.ihmc.euclid.tuple3D.Point3DTest;
import us.ihmc.euclid.tuple3D.Vector3D32Test;
import us.ihmc.euclid.tuple3D.Vector3DTest;
import us.ihmc.euclid.tuple4D.Quaternion32Test;
import us.ihmc.euclid.tuple4D.QuaternionTest;
import us.ihmc.euclid.tuple4D.Vector4D32Test;
import us.ihmc.euclid.tuple4D.Vector4DTest;

@RunWith(Suite.class)
@Suite.SuiteClasses({Point3DTest.class, Point3D32Test.class, Point2DTest.class, Point2D32Test.class, Vector3DTest.class, Vector3D32Test.class,
      Vector2DTest.class, Vector2D32Test.class, Vector4DTest.class, Vector4D32Test.class, AxisAngleTest.class, AxisAngle32Test.class, QuaternionTest.class,
      Quaternion32Test.class, Matrix3DTest.class, RotationMatrixTest.class, RotationScaleMatrixTest.class, RigidBodyTransformTest.class,
      QuaternionBasedTransformTest.class, AffineTransformTest.class,

      // Tools tests
      EuclidCoreToolsTest.class, EuclidCoreTestToolsTest.class, QuaternionToolsTest.class, Matrix3DToolsTest.class, Matrix3DFeaturesTest.class,
      RotationMatrixToolsTest.class, RotationScaleMatrixToolsTest.class, TupleToolsTest.class,

      // Conversion tests
      AxisAngleConversionTest.class, RotationVectorConversionTest.class, QuaternionConversionTest.class, RotationMatrixConversionTest.class,
      YawPitchRollConversionTest.class, CyclingConversionTest.class})

public class EuclidCoreTestSuite
{
   public static void main(String[] args) throws URISyntaxException, IOException
   {
      String targetTests = "us.ihmc.euclid.testSuite.EuclidCoreTestSuite";
      String targetClasses = "us.ihmc.euclid.*";
      doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }

   public static void doPITMutationTestAndOpenResult(String targetTests, String targetClasses)
   {
      String reportDirectoryName = "pit-reports";
      MutationCoverageReport.main(new String[] {"--reportDir", reportDirectoryName, "--targetClasses", targetClasses, "--targetTests", targetTests,
            "--sourceDirs", "src,test"});

      File reportDirectory = new File(reportDirectoryName);
      if (reportDirectory.isDirectory() && reportDirectory.exists())
      {
         String[] list = reportDirectory.list();
         String lastDirectoryName = list[list.length - 1];

         System.out.println("Found last directory " + lastDirectoryName);

         File reportFile = new File(reportDirectory, lastDirectoryName + "/index.html");
         String absolutePath;
         try
         {
            absolutePath = reportFile.getCanonicalPath();

            absolutePath = absolutePath.replace("\\", "/");
            System.out.println("Opening " + "file://" + absolutePath);

            URI uri = new URI("file://" + absolutePath);
            Desktop.getDesktop().browse(uri);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         catch (URISyntaxException e)
         {
            e.printStackTrace();
         }
      }
   }
}
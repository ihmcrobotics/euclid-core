package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Point3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public class Point3D extends Tuple3D<Point3D> implements Serializable, Point3DBasics<Point3D>
{
   private static final long serialVersionUID = -1234830724104344277L;

   public Point3D()
   {
      super();
   }

   public Point3D(double x, double y, double z)
   {
      super(x, y, z);
   }

   public Point3D(double[] pointArray)
   {
      super(pointArray);
   }

   public Point3D(Tuple3DReadOnly<?> other)
   {
      super(other);
   }
}

package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;

public class Vector3D extends Tuple3D<Vector3D> implements Serializable, Vector3DBasics<Vector3D>
{
   private static final long serialVersionUID = -8494204802892437237L;

   public Vector3D()
   {
      super();
   }

   public Vector3D(double x, double y, double z)
   {
      super(x, y, z);
   }

   public Vector3D(double[] vectorArray)
   {
      super(vectorArray);
   }

   public Vector3D(Tuple3DReadOnly<?> other)
   {
      super(other);
   }
}

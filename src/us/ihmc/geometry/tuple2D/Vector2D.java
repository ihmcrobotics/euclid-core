package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;

public class Vector2D extends Tuple2D<Vector2D> implements Serializable, Vector2DBasics<Vector2D>
{
   private static final long serialVersionUID = -1422872858238666884L;

   public Vector2D()
   {
      super();
   }

   public Vector2D(double x, double y)
   {
      super(x, y);
   }

   public Vector2D(double[] vectorArray)
   {
      super(vectorArray);
   }

   public Vector2D(Tuple2DReadOnly<?> other)
   {
      super(other);
   }
}

package us.ihmc.alexMocap.core;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * One marker's measurement in one frame: {@code ^W m_ij}, plus whether the system actually saw it.
 * <p>
 * Mutable and reusable. Instances are owned by a {@link MocapFrame} and live for the length of the
 * session; the 200 Hz loop overwrites them rather than allocating.
 * </p>
 * <p>
 * <b>An invisible marker's position is NaN, not the last known value and not zero.</b> A stale
 * position is the worst of the three: it looks plausible, it registers without complaint, and it
 * drags the recovered pose toward where the marker used to be. NaN propagates visibly.
 * </p>
 */
public class MarkerObservation
{
   private final MarkerId id;
   private final Point3D position = new Point3D(Double.NaN, Double.NaN, Double.NaN);
   private boolean visible = false;

   public MarkerObservation(MarkerId id)
   {
      if (id == null)
         throw new IllegalArgumentException("MarkerId must not be null.");

      this.id = id;
   }

   public MarkerId getId()
   {
      return id;
   }

   /**
    * The measured world position, valid only when {@link #isVisible()}. NaN otherwise.
    * <p>
    * Read-only on purpose: writing through this reference would set a position without setting the
    * visibility flag, leaving an observation that is invisible and yet has coordinates.
    * </p>
    */
   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public boolean isVisible()
   {
      return visible;
   }

   /**
    * Marks this marker seen at the given world position.
    * <p>
    * Non-finite coordinates are rejected. A source reporting a marker as visible at NaN is broken,
    * and accepting it would break the invariant <b>visible ⟺ finite position</b> -- which the CSV
    * log leans on to encode visibility in the coordinates themselves rather than in a redundant
    * flag that can disagree with them.
    * </p>
    */
   public void setVisible(double x, double y, double z)
   {
      if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z))
         throw new IllegalArgumentException("A visible marker needs a finite position; " + id + " was given (" + x + ", " + y + ", " + z + ").");

      position.set(x, y, z);
      visible = true;
   }

   /** @see #setVisible(double, double, double) */
   public void setVisible(Point3DReadOnly position)
   {
      setVisible(position.getX(), position.getY(), position.getZ());
   }

   /** Marks this marker unseen and sets its position to NaN. */
   public void setNotVisible()
   {
      position.setToNaN();
      visible = false;
   }

   public void set(MarkerObservation other)
   {
      if (other.visible)
         setVisible(other.position);
      else
         setNotVisible();
   }

   @Override
   public String toString()
   {
      return id + (visible ? " " + position : " (not visible)");
   }
}

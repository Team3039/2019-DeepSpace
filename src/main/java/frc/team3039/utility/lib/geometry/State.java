package frc.team3039.utility.lib.geometry;

import frc.team3039.utility.CSVWritable;
import frc.team3039.utility.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}

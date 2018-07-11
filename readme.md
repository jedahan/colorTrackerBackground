This tracks a 'keyboard'

Really it just looks for a blob that is roughly keyboard-sized, and floting maybe 10cm above the table.

# installation

Export the OPENFRAMEWORKS home directory

    export OF_HOME=$HOME/of_v0.10*

Link this to $OF_HOME/apps/roomSensors/

    ln -s . $OF_HOME/apps/roomSensors/

Rebuild

    cd $OF_HOME/apps/roomSensors/keyboardTracker
    make

Run

    DISPLAY=:0 make RunRelease

package com.javohirjambulov.rosandroid.model.entities.widgets;

import com.javohirjambulov.rosandroid.ui.general.Position;

/**
 * Entity with positional information to be able to display it
 * in the visualisation view as a stand-alone widget.
 *
 */
public interface IPositionEntity {

    Position getPosition();
    void setPosition(Position position);
}

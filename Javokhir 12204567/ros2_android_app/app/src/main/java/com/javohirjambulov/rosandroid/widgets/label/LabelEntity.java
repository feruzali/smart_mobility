package com.javohirjambulov.rosandroid.widgets.label;

import com.javohirjambulov.rosandroid.model.entities.widgets.SilentWidgetEntity;



public class LabelEntity extends SilentWidgetEntity {

    public String text;
    public int rotation;

    public LabelEntity() {
        this.width = 3;
        this.height = 1;
        this.text = "A label";
        this.rotation = 0;
    }
}

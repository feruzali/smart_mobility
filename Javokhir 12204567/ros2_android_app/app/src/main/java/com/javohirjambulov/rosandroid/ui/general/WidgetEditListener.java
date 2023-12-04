package com.javohirjambulov.rosandroid.ui.general;

import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;



public interface WidgetEditListener {

    void onWidgetEdited(BaseEntity widgetEntity, boolean updateConfig);
}

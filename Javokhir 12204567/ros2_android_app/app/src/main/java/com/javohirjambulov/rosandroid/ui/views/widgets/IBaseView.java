package com.javohirjambulov.rosandroid.ui.views.widgets;

import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;


public interface IBaseView {

    void setWidgetEntity(BaseEntity entity);
    BaseEntity getWidgetEntity();

    boolean sameWidgetEntity(BaseEntity other);
}

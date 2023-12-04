package com.javohirjambulov.rosandroid.ui.views.details;

import android.view.View;

import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;


interface IBaseViewHolder {

    void baseInitView(View view);
    void baseBindEntity(BaseEntity entity);
    void baseUpdateEntity(BaseEntity entity);
}

package com.javohirjambulov.rosandroid.widgets.logger;

import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Spinner;

import com.javohirjambulov.rosandroid.R;
import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.SubscriberWidgetViewHolder;
import com.javohirjambulov.rosandroid.utility.Utils;

import java.util.Collections;
import java.util.List;




public class LoggerDetailVH extends SubscriberWidgetViewHolder {

    private Spinner labelTextRotationSpinner;
    private ArrayAdapter<CharSequence> rotationAdapter;


    @Override
    public void initView(View view) {
        labelTextRotationSpinner = view.findViewById(R.id.loggerTextRotation);

        rotationAdapter = ArrayAdapter.createFromResource(view.getContext(),
                R.array.button_rotation, android.R.layout.simple_spinner_dropdown_item);
        labelTextRotationSpinner.setAdapter(rotationAdapter);
    }

    @Override
    protected void bindEntity(BaseEntity entity) {
        LoggerEntity loggerEntity = (LoggerEntity) entity;
        String degrees = Utils.numberToDegrees(loggerEntity.rotation);

        labelTextRotationSpinner.setSelection(rotationAdapter.getPosition(degrees));
    }

    @Override
    protected void updateEntity(BaseEntity entity) {
        LoggerEntity loggerEntity = (LoggerEntity) entity;
        String degrees = labelTextRotationSpinner.getSelectedItem().toString();

        loggerEntity.rotation = Utils.degreesToNumber(degrees);
    }

    @Override
    public List<String> getTopicTypes() {
        return Collections.singletonList(std_msgs.String._TYPE);
    }
}

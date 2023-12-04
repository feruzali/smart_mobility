package com.javohirjambulov.rosandroid.widgets.button;

import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.Spinner;

import com.javohirjambulov.rosandroid.R;
import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.PublisherWidgetViewHolder;
import com.javohirjambulov.rosandroid.utility.Utils;

import java.util.Collections;
import java.util.List;

import std_msgs.Bool;



public class ButtonDetailVH extends PublisherWidgetViewHolder {

    private EditText textText;
    private Spinner rotationSpinner;
    private ArrayAdapter<CharSequence> rotationAdapter;


    @Override
    public void initView(View view) {
        textText = view.findViewById(R.id.btnTextTypeText);
        rotationSpinner = view.findViewById(R.id.btnTextRotation);

        // Init spinner
        rotationAdapter = ArrayAdapter.createFromResource(view.getContext(),
                R.array.button_rotation, android.R.layout.simple_spinner_dropdown_item);

        rotationSpinner.setAdapter(rotationAdapter);
    }

    @Override
    protected void bindEntity(BaseEntity entity) {
        ButtonEntity buttonEntity = (ButtonEntity) entity;

        textText.setText(buttonEntity.text);
        String degrees = Utils.numberToDegrees(buttonEntity.rotation);
        rotationSpinner.setSelection(rotationAdapter.getPosition(degrees));
    }

    @Override
    protected void updateEntity(BaseEntity entity) {
        ButtonEntity buttonEntity = (ButtonEntity)entity;

        buttonEntity.text = textText.getText().toString();
        String degrees = rotationSpinner.getSelectedItem().toString();
        buttonEntity.rotation = Utils.degreesToNumber(degrees);
    }

    @Override
    public List<String> getTopicTypes() {
        return Collections.singletonList(Bool._TYPE);
    }
}

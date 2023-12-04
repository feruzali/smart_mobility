package com.javohirjambulov.rosandroid.widgets.rqtplot;

import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.TextView;

import com.google.android.material.textfield.TextInputEditText;
import com.javohirjambulov.rosandroid.R;
import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.SubscriberWidgetViewHolder;
import com.javohirjambulov.rosandroid.utility.Utils;

import java.util.ArrayList;
import java.util.List;



public class RqtPlotDetailVH extends SubscriberWidgetViewHolder implements TextView.OnEditorActionListener {

    private static final String TAG = RqtPlotDetailVH.class.getSimpleName();

    private TextInputEditText fieldEditText;


    @Override
    public void initView(View view) {
        fieldEditText = view.findViewById(R.id.fieldEditText);
        fieldEditText.setOnEditorActionListener(this);
    }

    @Override
    protected void bindEntity(BaseEntity entity) {
        RqtPlotEntity plotEntity = (RqtPlotEntity) entity;
        fieldEditText.setText(plotEntity.fieldPath);
    }

    @Override
    protected void updateEntity(BaseEntity entity) {
        RqtPlotEntity plotEntity = (RqtPlotEntity) entity;

        if (fieldEditText.getText() == null)
            return;

        plotEntity.fieldPath = fieldEditText.getText().toString().trim();

    }

    @Override
    public List<String> getTopicTypes() {
        return new ArrayList<>();
    }


    @Override
    public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
        switch (actionId){
            case EditorInfo.IME_ACTION_DONE:
            case EditorInfo.IME_ACTION_NEXT:
            case EditorInfo.IME_ACTION_PREVIOUS:
                Utils.hideSoftKeyboard(v);
                v.clearFocus();
                this.forceWidgetUpdate();
                return true;
        }

        return false;
    }
}

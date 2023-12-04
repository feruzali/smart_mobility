package com.javohirjambulov.rosandroid.widgets.viz2d;

import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.TextView;

import com.google.android.material.textfield.TextInputEditText;
import com.javohirjambulov.rosandroid.R;
import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.WidgetGroupViewHolder;
import com.javohirjambulov.rosandroid.utility.Utils;



public class Viz2DDetailVH  extends WidgetGroupViewHolder implements TextView.OnEditorActionListener {

    private TextInputEditText frameEditText;


    @Override
    protected void initView(View parentView) {
        // Initialize Topic Edittext
        frameEditText = parentView.findViewById(R.id.frame_edit_text);
        frameEditText.setOnEditorActionListener(this);
    }

    @Override
    protected void bindEntity(BaseEntity entity) {
        String frame = ((Viz2DEntity) entity).frame;
        frameEditText.setText(frame);
    }

    @Override
    protected void updateEntity(BaseEntity entity) {
        Viz2DEntity viz2d = (Viz2DEntity) entity;
        viz2d.frame = frameEditText.getText().toString();
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

package com.javohirjambulov.rosandroid.widgets.debug;

import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.EditText;
import android.widget.TextView;

import com.javohirjambulov.rosandroid.R;
import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.SubscriberWidgetViewHolder;
import com.javohirjambulov.rosandroid.utility.Utils;

import java.util.ArrayList;
import java.util.List;



public class DebugDetailVH extends SubscriberWidgetViewHolder implements TextView.OnEditorActionListener {

    protected EditText messageNumberEdittext;


    @Override
    protected void initView(View parentView) {
        messageNumberEdittext = parentView.findViewById(R.id.messageNumberEdittext);
        messageNumberEdittext.setOnEditorActionListener(this);
    }

    @Override
    protected void bindEntity(BaseEntity widget) {
        DebugEntity entity = (DebugEntity) widget;
        messageNumberEdittext.setText(String.valueOf(entity.numberMessages));

    }

    @Override
    protected void updateEntity(BaseEntity widget) {
        DebugEntity entity = (DebugEntity) widget;
        entity.numberMessages = Integer.parseInt(messageNumberEdittext.getText().toString());
    }


    @Override
    public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
        switch (actionId){
            case EditorInfo.IME_ACTION_DONE:
            case EditorInfo.IME_ACTION_NEXT:
            case EditorInfo.IME_ACTION_PREVIOUS:
                Utils.hideSoftKeyboard(itemView);
                itemView.requestFocus();
                return true;
        }

        return false;
    }

    @Override
    public List<String> getTopicTypes() {
        return new ArrayList<>();
    }

}
package com.javohirjambulov.rosandroid.widgets.battery;

import android.view.View;

import com.google.android.material.switchmaterial.SwitchMaterial;
import com.javohirjambulov.rosandroid.R;
import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.SubscriberWidgetViewHolder;

import java.util.Collections;
import java.util.List;



public class BatteryDetailVH extends SubscriberWidgetViewHolder {

    SwitchMaterial voltageSwitch;
    boolean forceSetChecked;

    @Override
    public void initView(View view) {
        voltageSwitch = view.findViewById(R.id.voltageSwitch);
        voltageSwitch.setOnCheckedChangeListener((compoundButton, b) -> {
            if (!forceSetChecked) forceWidgetUpdate();
        });
    }

    @Override
    protected void bindEntity(BaseEntity entity) {
        BatteryEntity batteryEntity = (BatteryEntity) entity;

        forceSetChecked = true;
        voltageSwitch.setChecked(batteryEntity.displayVoltage);
        forceSetChecked = false;
    }

    @Override
    protected void updateEntity(BaseEntity entity) {
        BatteryEntity batteryEntity = (BatteryEntity)entity;
        batteryEntity.displayVoltage = voltageSwitch.isChecked();
    }

    @Override
    public List<String> getTopicTypes() {
        return Collections.singletonList(sensor_msgs.BatteryState._TYPE);
    }
}

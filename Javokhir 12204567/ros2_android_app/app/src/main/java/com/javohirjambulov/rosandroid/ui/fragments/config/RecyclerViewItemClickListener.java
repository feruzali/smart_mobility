package com.javohirjambulov.rosandroid.ui.fragments.config;

import android.view.View;

import androidx.recyclerview.widget.RecyclerView;



public interface RecyclerViewItemClickListener {

    void onClick(RecyclerView parent, View view, int position);

    //void onLongClick(View view, int position);
}

package com.javohirjambulov.rosandroid.model.entities;

import androidx.room.Entity;
import androidx.room.PrimaryKey;



@Entity(tableName = "config_table")
public class ConfigEntity {

    @PrimaryKey(autoGenerate = true)
    public long id;

    public long creationTime;
    public long lastUsed;
    public String name = "DefaultName";
    public boolean isFavourite;
}
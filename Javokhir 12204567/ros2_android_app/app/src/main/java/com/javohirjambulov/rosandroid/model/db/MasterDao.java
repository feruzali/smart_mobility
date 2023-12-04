package com.javohirjambulov.rosandroid.model.db;

import androidx.lifecycle.LiveData;
import androidx.room.Dao;
import androidx.room.Query;

import com.javohirjambulov.rosandroid.model.entities.MasterEntity;



@Dao
public abstract class MasterDao implements BaseDao<MasterEntity>{

    @Query("SELECT * FROM master_table WHERE configId = :configId LIMIT 1")
    abstract LiveData<MasterEntity> getMaster(long configId);

    @Query("DELETE FROM master_table WHERE configId = :configId")
    abstract void delete(long configId);

    @Query("DELETE FROM master_table")
    abstract void deleteAll();
}

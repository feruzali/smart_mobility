package com.javohirjambulov.rosandroid.ui.fragments.details;

import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.AsyncListDiffer;
import androidx.recyclerview.widget.DiffUtil;
import androidx.recyclerview.widget.RecyclerView;

import com.javohirjambulov.rosandroid.BuildConfig;
import com.javohirjambulov.rosandroid.R;
import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.general.WidgetChangeListener;
import com.javohirjambulov.rosandroid.ui.views.details.BaseDetailSubscriberVH;
import com.javohirjambulov.rosandroid.ui.views.details.BaseDetailViewHolder;
import com.javohirjambulov.rosandroid.utility.Constants;
import com.javohirjambulov.rosandroid.utility.Utils;
import com.javohirjambulov.rosandroid.viewmodel.DetailsViewModel;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;



public class WidgetDetailListAdapter extends RecyclerView.Adapter<BaseDetailViewHolder<BaseEntity>>
        implements WidgetChangeListener {

    public static String TAG = WidgetDetailListAdapter.class.getSimpleName();

    private WidgetChangeListener widgetChangeListener;
    private final AsyncListDiffer<BaseEntity> mDiffer;
    private final ArrayList<Class<? extends BaseDetailViewHolder<BaseEntity>>> types;
    private final DetailsViewModel mViewModel;


    public WidgetDetailListAdapter(DetailsViewModel viewModel) {
        this.mViewModel = viewModel;
        mDiffer = new AsyncListDiffer<>(this, diffCallback);
        types = new ArrayList<>();
    }


    @NonNull
    @Override
    public BaseDetailViewHolder<BaseEntity> onCreateViewHolder(@NonNull ViewGroup parent, int itemType) {
        try {
            Class<? extends BaseDetailViewHolder<BaseEntity>> viewHolderClazz = types.get(itemType);
            Constructor<? extends BaseDetailViewHolder<BaseEntity>> cons =
                    viewHolderClazz.getConstructor(View.class, WidgetChangeListener.class);

            LayoutInflater inflator = LayoutInflater.from(parent.getContext());
            View itemView = inflator.inflate(R.layout.widget_detail_base, parent, false);

            BaseDetailViewHolder<BaseEntity> viewHolder = cons.newInstance(itemView, this);
            viewHolder.setViewModel(mViewModel);

            return viewHolder;

        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }


    @Override
    public void onBindViewHolder(@NonNull BaseDetailViewHolder<BaseEntity> holder, int position) {
        BaseEntity widget = this.getItem(position);

        // Get layout id
        String layoutStr = String.format(Constants.DETAIL_LAYOUT_FORMAT, widget.type.toLowerCase());
        int detailContentLayout = Utils.getResId(layoutStr, R.layout.class);

        // Inflate layout
        LayoutInflater inflator = LayoutInflater.from(holder.itemView.getContext());
        View inflatedView = inflator.inflate(detailContentLayout, null);

        holder.detailContend.removeView(holder.detailContend.getChildAt(1));
        holder.detailContend.addView(inflatedView);

        // Bind to widget
        holder.init();
        holder.bind(widget.copy());
    }

    @Override
    public int getItemViewType(int position) {
        BaseEntity entity = this.getItem(position);

        String classPath = BuildConfig.APPLICATION_ID
                + String.format(Constants.VIEWHOLDER_FORMAT, entity.type.toLowerCase(), entity.type);

        try {
            Class<?> clazzObject = Class.forName(classPath);

            if (clazzObject.getSuperclass() != BaseDetailViewHolder.class
                    && clazzObject.getSuperclass() != BaseDetailSubscriberVH.class) {
                return -1;

            } else if (types.contains(clazzObject)) {
                return types.indexOf(clazzObject);

            } else {
                types.add((Class<? extends BaseDetailViewHolder<BaseEntity>>) clazzObject);
                return types.size() - 1;
            }

        } catch (ClassNotFoundException e) {
            return -1;
        }

    }

    @Override
    public int getItemCount() {
        return mDiffer.getCurrentList().size();
    }

    @Override
    public void onWidgetDetailsChanged(BaseEntity widget) {
        if (widgetChangeListener != null) {
            this.widgetChangeListener.onWidgetDetailsChanged(widget);
        }
    }

    public BaseEntity getItem(int position) {
        return mDiffer.getCurrentList().get(position);
    }

    public void setWidgets(List<BaseEntity> newWidgets) {
        for (BaseEntity entity: newWidgets) {
            Log.i(TAG, "New Widget: " + entity);
        }
        mDiffer.submitList(newWidgets);
    }

    public void setChangeListener(WidgetChangeListener widgetChangeListener) {
        this.widgetChangeListener = widgetChangeListener;
    }


    private final DiffUtil.ItemCallback<BaseEntity> diffCallback = new DiffUtil.ItemCallback<BaseEntity>() {
        @Override
        public boolean areItemsTheSame(BaseEntity oldItem, BaseEntity newItem) {
            return oldItem.id == newItem.id;
        }

        @Override
        public boolean areContentsTheSame(BaseEntity oldItem, @NonNull BaseEntity newItem) {
            return oldItem.equals(newItem);
        }
    };
}
package com.example.android.maps_bluetooth_1;

import android.app.Activity;
import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.view.Window;

import java.io.Serializable;

public class Main_page extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
       // this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.activity_main_page);

    }

    public void on_click(View view) {

        Intent intent = new Intent(this, mainActivity.class);
        startActivity(intent);

    }
}

package com.example.android.maps_bluetooth_1;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Parcelable;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.view.Window;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Set;

public class mainActivity extends Activity {

    //Private members
    private BluetoothAdapter BA;
    private Set<BluetoothDevice> pairedDevices;
    private BluetoothDevice mDevice;
    private BluetoothSocket socket = null;


    @Override
    protected void onCreate(Bundle savedInstanceState) {


        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.activity_main);
        BA = BluetoothAdapter.getDefaultAdapter();
        Button butt = (Button)findViewById(R.id.show_connected);
        butt.setEnabled(false);


    }




    //This function will switch on the bluetooth device
    public void sw_on_bt(View view) {
        Button butt = (Button)findViewById(R.id.show_connected);
            butt.setEnabled(true);
            if (!BA.isEnabled()) {
                Intent turnOn = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(turnOn, 0);
                Toast.makeText(getApplicationContext(), "Turned on", Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(getApplicationContext(), "Already on", Toast.LENGTH_SHORT).show();
            }


    }


    //This function will list all the devices on the list view
    public void list_connected_bt(View view) {

        ArrayList list = new ArrayList();
        pairedDevices = BA.getBondedDevices();

        ListView lv = (ListView)findViewById(R.id.bt_list);
        pairedDevices = BA.getBondedDevices();
        for(BluetoothDevice bt : pairedDevices) {
            list.add(bt.getName());
            if(bt.getName().equals("HC-05")) {
                mDevice = bt;
            }
        }
        Toast.makeText(getApplicationContext(), "Showing Paired Devices",Toast.LENGTH_SHORT).show();

        final ArrayAdapter adapter = new  ArrayAdapter(this,android.R.layout.simple_list_item_1, list);
        lv.setAdapter(adapter);
        lv.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                String bt_name = parent.getItemAtPosition(position).toString();
                if(bt_name.equals("HC-05")) {
                    Intent intent = new Intent(getApplicationContext(), Send_RecvActivity.class);
                    intent.putExtra("Bt_name",(Parcelable) mDevice);

                    startActivity(intent);
                    Toast.makeText(getApplicationContext(), "You selected the right one to communicate", Toast.LENGTH_SHORT);
                }
                else
                    Toast.makeText(getApplicationContext(),"Sorry select another one or checkt the connectivity",Toast.LENGTH_SHORT);
            }
        });




    }
}

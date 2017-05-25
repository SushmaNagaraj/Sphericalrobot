package com.example.android.maps_bluetooth_1;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Handler;
import android.os.Message;
import android.os.Parcelable;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.view.Window;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.maps.model.LatLng;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.UUID;

import static android.R.attr.start;

public class Send_RecvActivity extends Activity {


    TextView tv;

    private String recv_str = "";
    private BluetoothAdapter BA;
    private Set<BluetoothDevice> pairedDevices;
    private BluetoothDevice mDevice;
    private BluetoothSocket socket = null;
    ConnectedThread mConnectedThread;
    ConnectThread mConnectThread;
    Button butt, butt1, butt2, butt3 ;



    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.activity_send__recv);
        BA = BluetoothAdapter.getDefaultAdapter();
        Intent intent = getIntent();
        mDevice = intent.getExtras().getParcelable("Bt_name");
        mConnectThread = new ConnectThread(mDevice);
        mConnectThread.start();
        tv = (TextView)findViewById(R.id.data_disp);
        butt = (Button)findViewById(R.id.map_btn);
        butt.setEnabled(false);
        butt1 = (Button)findViewById(R.id.recv_data);
        butt1.setEnabled(false);
        butt2 = (Button)findViewById(R.id.split_data);
        butt2.setEnabled(false);

    }

    public void btn_start(View view) throws InterruptedException {

        String test = mDevice.getName();
        Toast.makeText(this,test,Toast.LENGTH_SHORT).show();
        System.out.println("Device is :"+test);
        //For testing without Bluetooth device comment out the below part
//

        String start = "1";
        mConnectedThread.write(start.getBytes());
        butt1.setEnabled(true);
    }

    private List<LatLng> location= new ArrayList<LatLng>();

    public void btn_recv(View view) {

        String stop = "0";
        mConnectedThread.write(stop.getBytes());
        butt2.setEnabled(true);
        //For debuggin purpose

       // recv_str = "37.329852, -121.904997,37.329521,-121.905198,37.329393,  -121.904965,37.329344, -121.904965";

        if(recv_str == "") {


            Toast.makeText(getApplicationContext(), "Till the text view is filled with data please press receiver button or check your bluetooth module", Toast.LENGTH_SHORT).show();

        }
       /* else {
            Toast.makeText(getApplicationContext(),recv_str,Toast.LENGTH_SHORT).show();

            int str_itr = 0;
            int cnt = 0;    //contains the number of

            for (int i = 0; i < recv_str.length(); i++) {
                if (recv_str.charAt(str_itr) == ',')
                    cnt++;
                str_itr++;
            }
            cnt++;
            double[] latitude = new double[cnt];
            double[] longitude = new double[cnt];

            String[] inputNumbers = recv_str.split(",");

            for (int i = 0; i < cnt; i = i + 2) {
                latitude[i] = Double.parseDouble(inputNumbers[i]);
            }

            for (int i = 1; i < cnt; i = i + 2) {
                longitude[i] = Double.parseDouble(inputNumbers[i]);
            }


            for (int i = 0; i < cnt; i = i + 2) {
                LatLng loc = new LatLng(latitude[i], longitude[i + 1]);
                location.add(loc);
            }
            tv.setText(recv_str);
            butt.setEnabled(true);

        }

*/


    }

    public void map_data(View view) {


        Intent intent = new Intent(this, MapsActivity.class);
        System.out.println("size while sending"+location.size());
        intent.putExtra("latlong", (Serializable) location);
        startActivity(intent);


    }

    public void split_data(View view) {

        int str_itr = 0;
        int cnt = 0;    //contains the number of

        for (int i = 0; i < recv_str.length()-1; i++) {
            if (recv_str.charAt(str_itr) == ',')
                cnt++;
            str_itr++;
        }
        StringBuilder sb = new StringBuilder(recv_str);
        sb.deleteCharAt(recv_str.length()-1);
        recv_str = sb.toString();
        cnt++;
        double[] latitude = new double[cnt];
        double[] longitude = new double[cnt];

        String[] inputNumbers = recv_str.split(",");

        for (int i = 0; i < cnt; i = i + 2) {
            latitude[i] = Double.parseDouble(inputNumbers[i]);
        }

        for (int i = 1; i < cnt; i = i + 2) {
            longitude[i] = Double.parseDouble(inputNumbers[i]);
        }


        for (int i = 0; i < cnt; i = i + 2) {
            LatLng loc = new LatLng(latitude[i], longitude[i + 1]);
            location.add(loc);
        }
        tv.setText(recv_str);
        butt.setEnabled(true);



    }

    public void demo_data(View view) {
        String stop = "F";
        mConnectedThread.write(stop.getBytes());

    }



    //Thread creation for inner thread of main class

    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final BluetoothDevice mmDevice;
        private final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
        public ConnectThread(BluetoothDevice device) {
            BluetoothSocket tmp = null;
            mmDevice = device;

            try {
                tmp = device.createRfcommSocketToServiceRecord(MY_UUID);
            } catch (IOException e) { }
            mmSocket = tmp;
            socket = mmSocket;
        }
        public void run() {
            BA.cancelDiscovery();

            try {
                mmSocket.connect();
                mConnectedThread = new ConnectedThread(mmSocket);
                mConnectedThread.start();

            } catch (IOException connectException) {
                try {
                    mmSocket.close();
                } catch (IOException closeException) { }



                return;
            }
        }

        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) { }
        }
    }



    int i =1;
    Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            byte[] writeBuf = (byte[]) msg.obj;
            int begin = (int)msg.arg1;
            int end = (int)msg.arg2;
            switch(msg.what) {
                case 1:
                    String writeMessage = new String(writeBuf);
                    writeMessage = writeMessage.substring(begin, end);
                   // Toast.makeText(getApplicationContext(), writeMessage,Toast.LENGTH_SHORT).show();
                     recv_str =recv_str + writeMessage;
                    tv.setText(recv_str);
                    System.out.println("The message is "+recv_str);
                    break;
            }
        }
    };


    //Used for sending and receiving the UART communication

    private class ConnectedThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectedThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) { }
            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }
        public void run() {
            byte[] buffer = new byte[1024];
            int begin = 0;
            int bytes = 0;
            while (true) {

                try {
                    bytes += mmInStream.read(buffer, bytes, buffer.length - bytes);
                    for(int i = begin; i < bytes; i++) {
                        if(buffer[i] == "L".getBytes()[0]) {
                            mHandler.obtainMessage(1, begin, i, buffer).sendToTarget();
                            begin = i + 1;
                            if(i == bytes - 1) {
                                bytes = 0;
                                begin = 0;
                            }
                        }
                    }
                } catch (IOException e) {
                    break;
                }
            }
        }
        public void write(byte[] bytes) {
            try {
                mmOutStream.write(bytes);
            } catch (IOException e) { }
        }
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) { }
        }
    }

}

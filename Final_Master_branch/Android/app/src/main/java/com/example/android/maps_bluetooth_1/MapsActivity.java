package com.example.android.maps_bluetooth_1;

import android.content.Intent;
import android.graphics.Color;
import android.location.Location;
import android.os.SystemClock;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.view.View;
import android.view.Window;

import com.google.android.gms.maps.CameraUpdate;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.PolylineOptions;

import java.util.ArrayList;
import java.util.List;

import static android.R.attr.start;
import static com.example.android.maps_bluetooth_1.R.id.map;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback {

    private GoogleMap mMap;

    private List<LatLng> location = new ArrayList<LatLng>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.activity_maps);
        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(map);
        mapFragment.getMapAsync(this);

        Intent intent = getIntent();
        location = intent.getParcelableArrayListExtra("latlong");

        System.out.println("size is" + location.size());

    }


    /**
     * Manipulates the map once available.
     * This callback is triggered when the map is ready to be used.
     * This is where we can add markers or lines, add listeners or move the camera. In this case,
     * we just add a marker near Sydney, Australia.
     * If Google Play services is not installed on the device, the user will be prompted to install
     * it inside the SupportMapFragment. This method will only be triggered once the user has
     * installed Google Play services and returned to the app.
     */
    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;

/*        // Add a marker in Sydney and move the camera
        LatLng sydney = new LatLng(-34, 151);
        mMap.addMarker(new MarkerOptions().position(sydney).title("Marker in Sydney"));
        mMap.moveCamera(CameraUpdateFactory.newLatLng(sydney));*/


        for (int i = 0; i < location.size(); i++) {
            mMap.addMarker(new MarkerOptions()
                    .position(location.get(i)));
            if (i < location.size() - 1)
                mMap.addPolyline(new PolylineOptions().add(location.get(i), location.get(i + 1)).width(5).color(Color.BLUE));
        }


//        mMap.addPolyline(new PolylineOptions().add(location.get(location.size()-1),location.get(0)).width(5).color(Color.BLUE));
//       mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(location.get(0),18));

        //Animation making



    }


    public void map_show(View view) {

        CameraPosition cameraPosition =
                new CameraPosition.Builder()
                        .target(location.get(0))
                        .bearing(45)
                        .tilt(90)
                        .zoom(18)
                        .build();

        mMap.animateCamera(CameraUpdateFactory.newCameraPosition(cameraPosition));


        mMap.animateCamera(
                CameraUpdateFactory.newCameraPosition(cameraPosition),
                10,
                new GoogleMap.CancelableCallback() {

                    @Override
                    public void onFinish() {


                    }

                    @Override
                    public void onCancel() {
                    }
                }
        );




    }
}

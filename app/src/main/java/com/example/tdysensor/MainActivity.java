package com.example.tdysensor;

import androidx.appcompat.app.AppCompatActivity;

import android.hardware.Sensor;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.Switch;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    private EditText thresholdEt;
    private EditText intervalEt;
    private Button thresholdBtn;
    private Button intervalBtn;
    private Switch gravitySw;
    private Switch accelerometerSw;
    private Switch linearAccelerometerSw;
    private Switch gyroscopeSw;
    private Switch rotationVectorSw;
    private TextView gravityTv;
    private TextView accelerometerTv;
    private TextView linearAccelerometerTv;
    private TextView gyroscopeTv;
    private TextView rotationVectorTv;
    private SensorUtil sensorUtil;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initView();
        initData();
        initListener();

    }

    private void initView() {
        thresholdEt = findViewById(R.id.et_threshold);
        intervalEt = findViewById(R.id.et_interval);
        thresholdBtn = findViewById(R.id.btn_threshold);
        intervalBtn = findViewById(R.id.btn_interval);

        gravitySw = findViewById(R.id.sw_gravity);
        accelerometerSw = findViewById(R.id.sw_accelerometer);
        linearAccelerometerSw = findViewById(R.id.sw_linear_accelerometer);
        gyroscopeSw = findViewById(R.id.sw_gyroscope);
        rotationVectorSw = findViewById(R.id.sw_rotationVector);

        gravityTv = findViewById(R.id.tv_gravity_value);
        accelerometerTv = findViewById(R.id.tv_accelerometer_value);
        linearAccelerometerTv = findViewById(R.id.tv_linear_accelerometer_value);
        gyroscopeTv = findViewById(R.id.tv_gyroscope_value);
        rotationVectorTv = findViewById(R.id.tv_rotationVector_value);
    }

    private void initListener() {
        thresholdBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                sensorUtil.setThreshold(thresholdEt.getText().toString());
            }
        });
        intervalBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                sensorUtil.setInterval(intervalEt.getText().toString());
            }
        });
        gravitySw.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean isChecked) {
                if (isChecked) {
                    sensorUtil.setSensorState(true, Sensor.TYPE_GRAVITY);
                } else {
                    sensorUtil.setSensorState(false, Sensor.TYPE_GRAVITY);
//                    gravityTv.setText("X???= \nY???= \nZ???=");
                }
            }
        });
        accelerometerSw.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean isChecked) {
                if (isChecked) {
                    sensorUtil.setSensorState(true, Sensor.TYPE_ACCELEROMETER);
                } else {
                    sensorUtil.setSensorState(false, Sensor.TYPE_ACCELEROMETER);
//                    accelerometerTv.setText("X???= \nY???= \nZ???=");
                }
            }
        });

        linearAccelerometerSw.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean isChecked) {
                if (isChecked) {
                    sensorUtil.setSensorState(true, Sensor.TYPE_LINEAR_ACCELERATION);
                } else {
                    sensorUtil.setSensorState(false, Sensor.TYPE_LINEAR_ACCELERATION);
//                    linearAccelerometerTv.setText("X???= \nY???= \nZ???=");
                }
            }
        });
        gyroscopeSw.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean isChecked) {
                if (isChecked) {
                    sensorUtil.setSensorState(true, Sensor.TYPE_GYROSCOPE);
                } else {
                    sensorUtil.setSensorState(false, Sensor.TYPE_GYROSCOPE);
//                    gyroscopeTv.setText("X???= \nY???= \nZ???=");
                }
            }
        });
        rotationVectorSw.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean isChecked) {
                if (isChecked) {
                    sensorUtil.setSensorState(true, Sensor.TYPE_ROTATION_VECTOR);
                } else {
                    sensorUtil.setSensorState(false, Sensor.TYPE_ROTATION_VECTOR);
//                    rotationVectorTv.setText("X???= \nY???= \nZ???=");
                }
            }
        });

        sensorUtil.setSensorValueListener(new SensorUtil.OnSensorValueListener() {
            @Override
            public void onSenorValue(String value, int type) {
                switch (type) {
                    case Sensor.TYPE_GRAVITY: //?????????????????????
                        gravityTv.setText(value);
                        break;
                    case Sensor.TYPE_ACCELEROMETER: //??????????????????
                        MyLogUtil.e("??????????????????",value);
                        accelerometerTv.setText(value);
                        break;
                    case Sensor.TYPE_LINEAR_ACCELERATION: //????????????????????????
                        linearAccelerometerTv.setText(value);
                        break;
                    case Sensor.TYPE_GYROSCOPE: //???????????????
                        gyroscopeTv.setText(value);
                        break;
                    case Sensor.TYPE_ROTATION_VECTOR: //???????????????????????????
                        rotationVectorTv.setText(value);
                        break;
                }
            }
        });
    }

    private void initData() {
        sensorUtil = new SensorUtil(this);

    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorUtil.onPause();
    }
}
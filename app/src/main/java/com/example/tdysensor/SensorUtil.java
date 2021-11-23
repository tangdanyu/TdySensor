package com.example.tdysensor;

import static android.util.Half.EPSILON;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;


public class SensorUtil implements SensorEventListener {
    private String TAG = getClass().getSimpleName();
    private SensorManager sensorManager;
    private Sensor gravitySensor;//重力传感器
    private Sensor accelerometerSensor;//加速度传感器
    private Sensor linearAccelerometerSensor;//线性加速度传感器
    private Sensor gyroscopeSensor;//陀螺仪
    private Sensor rotationVectorSensor;//旋转矢量传感器

    private long gravityLastTime;//重力传感器上次检测的时间
    private long accelerometerLastTime;//加速度上次检测的时间
    private long linearAccelerometerLastTime;// 线性加速度上次检测的时间
    private long gyroscopeLastTime;//陀螺仪上次检测的时间
    private long rotationVectorLastTime;//旋转矢量传感器上次检测的时间


    private float gravityValues[] = new float[3];
    private float lastGravityValues[] = new float[3];

    private float accelerometerValues[] = new float[3];
    private float lastAccelerometerValues[] = new float[3];

    private float accelerometerGravityValues[] = new float[3];
    private float accelerometerLinearValues[] = new float[3];
    private float lastAccelerometerLinearValues[] = new float[3];

    private float linearAccelerationValues[] = new float[3];
    private float lastLinearAccelerationValues[] = new float[3];

    private float gyroscopeValues[] = new float[3];
    private float lastGyroscopeValues[] = new float[3];
    private float deltaRotationVector[] = new float[4];

    private float rotationVectorValues[] = new float[3];
    private float lastRotationVectorValues[] = new float[3];

    // 检测的时间间隔 单位毫秒
    private int interval = 0;
    //摇晃检测阈值，决定了对摇晃的敏感程度，越小越敏感。
    private double threshold = 0;

    //重力加速度常量
    public static final float STANDARD_GRAVITY = 9.78F;

    //加速度传感器是否包含重力
    private boolean isIncludeGravity = true;

    //陀螺仪传感器
    private float timestamp;

    // 将纳秒转化为秒
    private static final float NS2S = 1.0f / 1000000000.0f;


    private boolean isOnlyZ = false;


    private OnSensorValueListener sensorValueListener;

    //使用重力传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
    //使用线性加速度计 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
    //使用旋转矢量传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    //使用有效运动传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_SIGNIFICANT_MOTION);
    //使用计步器传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_COUNTER);
    //使用步测器传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR);
    //使用加速度计 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    //使用陀螺仪 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
    //使用未经校准的陀螺仪 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
    //使用游戏旋转矢量传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
    //使用地磁旋转矢量传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR);
    //使用地磁场传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    //使用未经校准的磁力计 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED);
    //使用近程传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
    //使用环境温度传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_AMBIENT_TEMPERATURE);
    //使用光传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_LIGHT);
    //使用环境压力传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
    //使用环境湿度传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_RELATIVE_HUMIDITY);
    //使用设备温度传感器 sensor = sensorManager.getDefaultSensor(Sensor.TYPE_TEMPERATURE);
    //SensorManager.SENSOR_DELAY_FASTEST = 4000
    //SensorManager.SENSOR_DELAY_NORMAL = 10000
    //SensorManager.SENSOR_DELAY_GAME = 10000
    //SensorManager.SENSOR_DELAY_UI = 10000
    public SensorUtil(Context mContext) {
        sensorManager = (SensorManager) mContext.getSystemService(Context.SENSOR_SERVICE);
        setSenorType(Sensor.TYPE_GRAVITY);
        setSenorType(Sensor.TYPE_ACCELEROMETER);
        setSenorType(Sensor.TYPE_LINEAR_ACCELERATION);
        setSenorType(Sensor.TYPE_GYROSCOPE);
        setSenorType(Sensor.TYPE_ROTATION_VECTOR);
    }

    public interface OnSensorValueListener {
        void onSenorValue(String value, int type);
    }

    public void setSensorValueListener(OnSensorValueListener onSensorValueListener) {
        sensorValueListener = onSensorValueListener;
    }

    public void setThreshold(String threshold) {
        if (!threshold.isEmpty()) {
            this.threshold = Double.parseDouble(threshold);
            MyLogUtil.e(TAG, "阈值=" + threshold);
        } else {
            this.threshold = 0;
        }

    }

    public void setInterval(String interval) {
        if (!interval.isEmpty()) {
            this.interval = Integer.parseInt(interval);
            MyLogUtil.e(TAG, "时间间隔=" + interval);
        } else {
            this.interval = 0;
        }
    }

    public boolean setSenorType(int type) {
        if (sensorManager.getDefaultSensor(type) != null) {
            switch (type) {
                case Sensor.TYPE_GRAVITY: //使用重力传感器
                    gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
                    break;
                case Sensor.TYPE_ACCELEROMETER: //使用加速度计
                    accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
                    break;
                case Sensor.TYPE_LINEAR_ACCELERATION: //使用线性加速度计
                    linearAccelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
                    break;
                case Sensor.TYPE_GYROSCOPE: //使用陀螺仪
                    gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
                    break;
                case Sensor.TYPE_ROTATION_VECTOR: //使用旋转矢量传感器
                    rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
                    break;
            }
            return true;
        } else {
            MyLogUtil.e(TAG, "没有该传感器" + type);
            return false;
        }
    }

    public void setSensorState(boolean isOpen, int type) {
        switch (type) {
            case Sensor.TYPE_GRAVITY: //使用重力传感器
                if (isOpen) {
                    sensorManager.registerListener(this, gravitySensor, SensorManager.SENSOR_DELAY_NORMAL);
                } else {
                    sensorManager.unregisterListener(this, gravitySensor);
                }
                break;
            case Sensor.TYPE_ACCELEROMETER: //使用加速度计
                if (isOpen) {
                    sensorManager.registerListener(this, accelerometerSensor, SensorManager.SENSOR_DELAY_NORMAL);
                } else {
                    sensorManager.unregisterListener(this, accelerometerSensor);
                }
                break;
            case Sensor.TYPE_LINEAR_ACCELERATION: //使用线性加速度计
                if (isOpen) {
                    sensorManager.registerListener(this, linearAccelerometerSensor, SensorManager.SENSOR_DELAY_NORMAL);
                } else {
                    sensorManager.unregisterListener(this, linearAccelerometerSensor);
                }
                break;
            case Sensor.TYPE_GYROSCOPE: //使用陀螺仪
                if (isOpen) {
                    sensorManager.registerListener(this, gyroscopeSensor, SensorManager.SENSOR_DELAY_NORMAL);
                } else {
                    sensorManager.unregisterListener(this, gyroscopeSensor);
                }
                break;
            case Sensor.TYPE_ROTATION_VECTOR: //使用旋转矢量传感器
                if (isOpen) {
                    sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_NORMAL);
                } else {
                    sensorManager.unregisterListener(this, rotationVectorSensor);
                }
                break;
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        Sensor sensor = event.sensor;
        if (sensor.getType() == Sensor.TYPE_GRAVITY) {//重力传感器
            //1、当x轴的值接近重力加速度时，说明设备的左边朝下。
            //2、当x轴的值接近负的g值时，说明设备的右边朝下。
            //3、当y轴的值接近g值时，说明设备的下边超下（与上图一样）。
            //4、当y轴的值接近负的g值时，说明设备的上边朝下（倒置）。
            //5、当z轴的值接近g值时，说明设备的屏幕朝上。
            //6、当z轴的值接近负的g值时，说明设备屏幕朝下。
            handleGravitySensor(event);
        } else if (sensor.getType() == Sensor.TYPE_ACCELEROMETER) {//加速度计
            //左右移动加速度 X=values[0]
            //前后移动加速度 Y=values[1]
            //垂直方向加速度 Z=values[2]
            //当设备平放在桌子上并从左侧向右推动时，x 加速度值为正。
            //当设备平放在桌子上时，z轴加速度值为+9.81，对应于设备的加速度（0 m/s^2）减去重力（-9.81 m/s^2）。
            //当设备平放在桌子上并以A m/s^2的加速度被推向天空时，加速度值等于A+9.81，对应于设备的加速度（+A m/s^2 ) 减去重力 (-9.81 m/s^2)。
            handleAccelerometerSensor(event);
        } else if (sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {//线性加速度计
            handleLinearAccelerometerSensor(event);
        } else if (sensor.getType() == Sensor.TYPE_GYROSCOPE) {   //陀螺仪
            //该设备绕X轴旋转的角速度 X=values[0]
            //该设备绕Y轴旋转的角速度 Y=values[1]
            //该设备绕Z轴旋转的角速度 Z=values[2]
            //逆时针方向旋转为正
            //坐标系与加速度传感器使用的坐标系相同
            handleGyroscopeSensor(event);
        } else if (sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {//使用旋转矢量传感器
            //将旋转向量转换为4x4矩阵。矩阵
            //OpenGL将其解释为
            //旋转向量，这就是我们想要的。
            handleRotationVector(event);
        }
    }

    //重力处理函数
    private void handleGravitySensor(SensorEvent event) {

        long currentUpdateTime = System.currentTimeMillis();
        long timeInterval = currentUpdateTime - gravityLastTime;
        if (interval > 0 && timeInterval < interval)
            return;
        gravityLastTime = currentUpdateTime;
        // 获得x,y,z坐标
        gravityValues[0] = event.values[0];
        gravityValues[1] = event.values[1];
        gravityValues[2] = event.values[2];

        // 获得x,y,z的变化值
        float deltaX = gravityValues[0] - lastGravityValues[0];
        float deltaY = gravityValues[1] - lastGravityValues[1];
        float deltaZ = gravityValues[2] - lastGravityValues[2];

        // 将现在的坐标变成last坐标
        lastGravityValues[0] = gravityValues[0];
        lastGravityValues[1] = gravityValues[1];
        lastGravityValues[2] = gravityValues[2];

//            MyLogUtil.e(TAG, "加速度传感器 " +
//                    " \nX轴加速度 " + event.values[0] +
//                    " \nY轴加速度 " + event.values[1] +
//                    " \nZ轴加速度 " + event.values[2]);
        String value = "取值范围=" + event.sensor.getMaximumRange() +
                "\nX轴=" + event.values[0] +
                "\nY轴=" + event.values[1] +
                "\nZ轴=" + event.values[2];
        double diff = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) / timeInterval * 10000;
        if (threshold > 0 && diff > threshold) {
            value += " 超过阈值";
            MyLogUtil.e(TAG, "重力传感器 手机在晃动");
        } else {
            MyLogUtil.e(TAG, "重力传感器 手机没有晃动");
        }
        sensorValueListener.onSenorValue(value, event.sensor.getType());

        if (gravityValues[0] > STANDARD_GRAVITY) {
            MyLogUtil.e(TAG, "重力传感器 重力指向设备左边");
        } else if (gravityValues[0] < -STANDARD_GRAVITY) {
            MyLogUtil.e(TAG, "重力传感器 重力指向设备右边");
        } else if (gravityValues[1] > STANDARD_GRAVITY) {
            MyLogUtil.e(TAG, "重力传感器 重力指向设备下边");
        } else if (gravityValues[1] < -STANDARD_GRAVITY) {
            MyLogUtil.e(TAG, "重力传感器 重力指向设备上边");
        } else if (gravityValues[2] > STANDARD_GRAVITY) {
            MyLogUtil.e(TAG, "重力传感器 屏幕朝上");
        } else if (gravityValues[2] < -STANDARD_GRAVITY) {
            MyLogUtil.e(TAG, "重力传感器 屏幕朝下");
        }


    }

    //加速度处理函数
    private void handleAccelerometerSensor(SensorEvent event) {
        long currentUpdateTime = System.currentTimeMillis();
        long timeInterval = currentUpdateTime - accelerometerLastTime;
        if (interval > 0 && timeInterval < interval)
            return;
        accelerometerLastTime = currentUpdateTime;
        if (isIncludeGravity) {//包含重力影响
            // 获得x,y,z坐标
            accelerometerValues[0] = event.values[0];
            accelerometerValues[1] = event.values[1];
            accelerometerValues[2] = event.values[2];

            // 获得x,y,z的变化值
            float deltaX = accelerometerValues[0] - lastAccelerometerValues[0];
            float deltaY = accelerometerValues[1] - lastAccelerometerValues[1];
            float deltaZ = accelerometerValues[2] - lastAccelerometerValues[2];

            // 将现在的坐标变成last坐标
            lastAccelerometerValues[0] = accelerometerValues[0];
            lastAccelerometerValues[1] = accelerometerValues[1];
            lastAccelerometerValues[2] = accelerometerValues[2];

//            MyLogUtil.e(TAG, "加速度传感器 " +
//                    " \nX轴加速度 " + event.values[0] +
//                    " \nY轴加速度 " + event.values[1] +
//                    " \nZ轴加速度 " + event.values[2]);
            String value = "取值范围=" + event.sensor.getMaximumRange() +
                    "\nX轴=" + event.values[0] +
                    "\nY轴=" + event.values[1] +
                    "\nZ轴=" + event.values[2];
            double diff = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) / timeInterval * 10000;
            if (threshold > 0 && diff > threshold) {
                value += " 超过阈值";
                MyLogUtil.e(TAG, "加速度传感器 手机在晃动");
            } else {
                MyLogUtil.e(TAG, "加速度传感器 手机没有晃动");
            }
            sensorValueListener.onSenorValue(value, event.sensor.getType());
        } else {//过滤重力影响
            // alpha is calculated as t / (t + dT)
            // with t, the low-pass filter's time-constant
            // and dT, the event delivery rate
            //  alpha 由 t / (t + dT)得到
            //  t为滤波器时间常量，为传感器单次采样时间
            //  dT为采样频率
            final float alpha = 0.8f;

            // 用低通滤波器隔离重力。
            accelerometerGravityValues[0] = alpha * accelerometerGravityValues[0] + (1 - alpha) * event.values[0];
            accelerometerGravityValues[1] = alpha * accelerometerGravityValues[1] + (1 - alpha) * event.values[1];
            accelerometerGravityValues[2] = alpha * accelerometerGravityValues[2] + (1 - alpha) * event.values[2];

            // 用高通滤波器剔除重力干扰
            accelerometerLinearValues[0] = event.values[0] - accelerometerGravityValues[0];
            accelerometerLinearValues[1] = event.values[1] - accelerometerGravityValues[1];
            accelerometerLinearValues[2] = event.values[2] - accelerometerGravityValues[2];

            accelerometerLinearValues[0] = (float) (Math.round(accelerometerLinearValues[0] * 1000)) / 1000;
            accelerometerLinearValues[1] = (float) (Math.round(accelerometerLinearValues[1] * 1000)) / 1000;
            accelerometerLinearValues[2] = (float) (Math.round(accelerometerLinearValues[2] * 1000)) / 1000;

            float lineX = accelerometerLinearValues[0] - lastAccelerometerLinearValues[0];
            float lineY = accelerometerLinearValues[1] - lastAccelerometerLinearValues[1];
            float lineZ = accelerometerLinearValues[2] - lastAccelerometerLinearValues[2];

            lastAccelerometerLinearValues[0] = accelerometerLinearValues[0];
            lastAccelerometerLinearValues[1] = accelerometerLinearValues[1];
            lastAccelerometerLinearValues[2] = accelerometerLinearValues[2];

//            MyLogUtil.e(TAG, "加速度传感器去除重力影响 " +
//                    " \nX轴加速度 " + event.values[0] +
//                    " \nY轴加速度 " + event.values[1] +
//                    " \nZ轴加速度 " + event.values[2]);
            double value = Math.sqrt(lineX * lineX + lineY * lineY + lineZ * lineZ) / timeInterval * 10000;
            if (value > threshold) {
                MyLogUtil.e(TAG, "加速度传感器去除重力影响 手机在晃动");
            } else {
                MyLogUtil.e(TAG, "加速度传感器去除重力影响 手机没有晃动");
            }
        }
    }

    //线性加速度处理函数
    private void handleLinearAccelerometerSensor(SensorEvent event) {
        long currentUpdateTime = System.currentTimeMillis();
        long timeInterval = currentUpdateTime - linearAccelerometerLastTime;
        if (interval > 0 && timeInterval < interval)
            return;
        linearAccelerometerLastTime = currentUpdateTime;
        linearAccelerationValues[0] = event.values[0];
        linearAccelerationValues[1] = event.values[1];
        linearAccelerationValues[2] = event.values[2];
        // 获得x,y,z的变化值
        float deltaX = linearAccelerationValues[0] - lastLinearAccelerationValues[0];
        float deltaY = linearAccelerationValues[1] - lastLinearAccelerationValues[1];
        float deltaZ = linearAccelerationValues[2] - lastLinearAccelerationValues[2];

        // 将现在的坐标变成last坐标
        lastLinearAccelerationValues[0] = linearAccelerationValues[0];
        lastLinearAccelerationValues[1] = linearAccelerationValues[1];
        lastLinearAccelerationValues[2] = linearAccelerationValues[2];

//            MyLogUtil.e(TAG, "加速度传感器 " +
//                    " \nX轴加速度 " + event.values[0] +
//                    " \nY轴加速度 " + event.values[1] +
//                    " \nZ轴加速度 " + event.values[2]);
        String value = "取值范围=" + event.sensor.getMaximumRange() +
                "\nX轴=" + event.values[0] +
                "\nY轴=" + event.values[1] +
                "\nZ轴=" + event.values[2];
        double diff = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) / timeInterval * 10000;
        if (threshold > 0 && diff > threshold) {
            value += " 超过阈值";
            MyLogUtil.e(TAG, "线性加速度传感器 手机在晃动");
        } else {
            MyLogUtil.e(TAG, "线性加速度传感器 手机没有晃动");
        }
        sensorValueListener.onSenorValue(value, event.sensor.getType());
        MyLogUtil.e(TAG, "加速度传感器线性  X轴加速度=" + event.values[0] + " Y轴加速度=" + event.values[1] + " Z轴加速度=" + event.values[2]);
    }

    //陀螺仪处理函数
    private void handleGyroscopeSensor(SensorEvent event) {
        if (isOnlyZ) {
            //仅对沿Z轴的旋转感兴趣
            if (event.values[2] > 0.5f) {
                MyLogUtil.e(TAG, "陀螺仪传感器 绕Z轴逆时针旋转的角度 大于45度");
            } else if (event.values[2] < -0.5f) {
                MyLogUtil.e(TAG, "陀螺仪传感器 绕Z轴顺时针旋转的角度 大于45度");
            }
        } else {
            // 根据陀螺仪采样数据计算出此次时间间隔的偏移量后，它将与当前旋转向量相乘。
            if (timestamp != 0) {
                //纳秒转换成秒
                final float dT = (event.timestamp - timestamp) * NS2S;
                long currentUpdateTime = System.currentTimeMillis();
                long timeInterval = currentUpdateTime - gyroscopeLastTime;
                if (interval > 0 && timeInterval < interval)
                    return;
                gyroscopeLastTime = currentUpdateTime;
                // 未规格化的旋转向量坐标值，。
                gyroscopeValues[0] = event.values[0];
                gyroscopeValues[1] = event.values[1];
                gyroscopeValues[2] = event.values[2];


                // 获得x,y,z的变化值
                float deltaX = gyroscopeValues[0] - lastGyroscopeValues[0];
                float deltaY = gyroscopeValues[1] - lastGyroscopeValues[1];
                float deltaZ = gyroscopeValues[2] - lastGyroscopeValues[2];

                // 将现在的坐标变成last坐标
                lastGyroscopeValues[0] = gyroscopeValues[0];
                lastGyroscopeValues[1] = gyroscopeValues[1];
                lastGyroscopeValues[2] = gyroscopeValues[2];

//            MyLogUtil.e(TAG, "加速度传感器 " +
//                    " \nX轴加速度 " + event.values[0] +
//                    " \nY轴加速度 " + event.values[1] +
//                    " \nZ轴加速度 " + event.values[2]);
                String value = "取值范围=" + event.sensor.getMaximumRange() +
                        "\nX轴=" + event.values[0] +
                        "\nY轴=" + event.values[1] +
                        "\nZ轴=" + event.values[2];
                double diff = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) / timeInterval * 10000;
                if (threshold > 0 && diff > threshold) {
                    value += " 超过阈值";
                    MyLogUtil.e(TAG, "陀螺仪传感器 手机在晃动");
                } else {
                    MyLogUtil.e(TAG, "陀螺仪传感器 手机没有晃动");
                }
                sensorValueListener.onSenorValue(value, event.sensor.getType());

                // 计算角速度 开根号
                float omegaMagnitude = (float) Math.sqrt(gyroscopeValues[0] * gyroscopeValues[0] + gyroscopeValues[1] * gyroscopeValues[1] + gyroscopeValues[2] * gyroscopeValues[2]);

                // 如果旋转向量偏移值足够大，可以获得坐标值，则规格化旋转向量
                // (也就是说，EPSILON 为计算偏移量的起步值。小于该值的偏移视为误差，不予计算。)
                if (omegaMagnitude > EPSILON) {
                    gyroscopeValues[0] /= omegaMagnitude;
                    gyroscopeValues[1] /= omegaMagnitude;
                    gyroscopeValues[2] /= omegaMagnitude;
                }

                // 为了得到此次取样间隔的旋转偏移量，需要把围绕坐标轴旋转的角速度与时间间隔合并表示。
                // 在转换为旋转矩阵之前，我们要把围绕坐标轴旋转的角度表示为四元组。
                //通过时间步长将该轴与角速度进行积分，以便在时间步长内从该样本获得增量旋转。
                // 我们将在将其转换为旋转矩阵之前，将增量旋转的轴角度表示转换为四元数。
                float thetaOverTwo = omegaMagnitude * dT / 2.0f;
                float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
                float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
                deltaRotationVector[0] = sinThetaOverTwo * gyroscopeValues[0];
                deltaRotationVector[1] = sinThetaOverTwo * gyroscopeValues[1];
                deltaRotationVector[2] = sinThetaOverTwo * gyroscopeValues[2];
                deltaRotationVector[3] = cosThetaOverTwo;
            }
            timestamp = event.timestamp;
            float[] deltaRotationMatrix = new float[9];
            SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
            // 为了得到旋转后的向量，用户代码应该把我们计算出来的偏移量与当前向量叠加。
            // 用户代码应该将我们计算的增量旋转与当前旋转连接起来，以获得更新的旋转。
            // rotationCurrent = rotationCurrent * deltaRotationMatrix;

            // Get the fused orienatation 获得融合定向
            float[] orientations = new float[3];
            SensorManager.getOrientation(deltaRotationMatrix, orientations);
            MyLogUtil.e(TAG, "陀螺仪传感器 绕X轴旋转的角度 " + orientations[0] + " 绕Y轴旋转的角度 " + orientations[1] + " 绕Z轴旋转的角度 " + orientations[2]);
            gyroscopeValues[0] = (float) (Math.round(event.values[0] + deltaRotationVector[0] * 1000)) / 1000;
            gyroscopeValues[1] = (float) (Math.round(event.values[1] + deltaRotationVector[1] * 1000)) / 1000;
            gyroscopeValues[2] = (float) (Math.round(event.values[2] + deltaRotationVector[2] * 1000)) / 1000;

            if (gyroscopeValues[0] > 45) {
                MyLogUtil.e(TAG, "陀螺仪传感器 绕X轴旋转的角度 大于45度");
            } else if (gyroscopeValues[1] > 45) {
                MyLogUtil.e(TAG, "陀螺仪传感器 绕Y轴旋转的角度 大于45度");
            } else if (gyroscopeValues[2] > 45) {
                MyLogUtil.e(TAG, "陀螺仪传感器 绕Z轴旋转的角度 大于45度");
            }
            MyLogUtil.e(TAG, "陀螺仪传感器 绕X轴旋转的角速度 " + gyroscopeValues[0] + " 绕Y轴旋转的角速度 " + gyroscopeValues[1] + " 绕Z轴旋转的角速度 " + gyroscopeValues[2]);

     /*   gyroscope[0]=(float)(Math.round(event.values[0]*1000))/1000;
        gyroscope[1]=(float)(Math.round(event.values[1]*1000))/1000;
        gyroscope[2]=(float)(Math.round(event.values[2]*1000))/1000;*/

        }


    }

    //旋转矢量处理函数
    private void handleRotationVector(SensorEvent event) {
        long currentUpdateTime = System.currentTimeMillis();
        long timeInterval = currentUpdateTime - rotationVectorLastTime;
        if (interval > 0 && timeInterval < interval)
            return;
        rotationVectorLastTime = currentUpdateTime;
        rotationVectorValues[0] = event.values[0];
        rotationVectorValues[1] = event.values[1];
        rotationVectorValues[2] = event.values[2];


        // 获得x,y,z的变化值
        float deltaX = rotationVectorValues[0] - lastRotationVectorValues[0];
        float deltaY = rotationVectorValues[1] - lastRotationVectorValues[1];
        float deltaZ = rotationVectorValues[2] - lastRotationVectorValues[2];

        // 将现在的坐标变成last坐标
        lastRotationVectorValues[0] = rotationVectorValues[0];
        lastRotationVectorValues[1] = rotationVectorValues[1];
        lastRotationVectorValues[2] = rotationVectorValues[2];

//            MyLogUtil.e(TAG, "加速度传感器 " +
//                    " \nX轴加速度 " + event.values[0] +
//                    " \nY轴加速度 " + event.values[1] +
//                    " \nZ轴加速度 " + event.values[2]);
        String value = "取值范围=" + event.sensor.getMaximumRange() +
                "\nX轴=" + event.values[0] +
                "\nY轴=" + event.values[1] +
                "\nZ轴=" + event.values[2];
        double diff = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) / timeInterval * 10000;
        if (threshold > 0 && diff > threshold) {
            value += " 超过阈值";
            MyLogUtil.e(TAG, "旋转矢量传感器 手机在晃动");
        } else {
            MyLogUtil.e(TAG, "旋转矢量传感器 手机没有晃动");
        }
        sensorValueListener.onSenorValue(value, event.sensor.getType());

        float[] rotationMatrix = new float[16];
        SensorManager.getRotationMatrixFromVector(
                rotationMatrix, event.values);
        // Remap coordinate system
//            float[] remappedRotationMatrix = new float[16];
//            SensorManager.remapCoordinateSystem(rotationMatrix,
//                    SensorManager.AXIS_X,
//                    SensorManager.AXIS_Z,
//                    remappedRotationMatrix);
        // Convert to orientations
        float[] orientations = new float[3];
        SensorManager.getOrientation(rotationMatrix, orientations);
        for (int i = 0; i < 3; i++) {
            orientations[i] = (float) (Math.toDegrees(orientations[i]));
            String str;
            if (i == 0) {
                str = " X轴 ";
            } else if (i == 1) {
                str = " Y轴 ";
            } else {
                str = " Z轴 ";
            }
            if (orientations[i] > 45) {
                MyLogUtil.e(TAG, "旋转矢量传感器 " + str + i + "顺时针倾斜大于45度" + orientations[i]);
            } else if (orientations[i] < -45) {
                MyLogUtil.e(TAG, "旋转矢量传感器 " + str + i + "逆时针倾斜大于45度" + orientations[i]);
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void onResume() {
//        sensorManager.registerListener(this, gravitySensor, SensorManager.SENSOR_DELAY_NORMAL);
//        sensorManager.registerListener(this, accelerometerSensor, SensorManager.SENSOR_DELAY_NORMAL);
//        sensorManager.registerListener(this, linearAccelerometerSensor, SensorManager.SENSOR_DELAY_NORMAL);
//        sensorManager.registerListener(this, gyroscopeSensor, SensorManager.SENSOR_DELAY_NORMAL);
//        sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_NORMAL);
    }

    public void onPause() {
        sensorManager.unregisterListener(this);
    }
}

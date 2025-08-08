# PID Controller Improvements

## Branch: improve-pid-stability

### สาเหตุของปัญหา (จากการวิเคราะห์ข้อมูล export.csv)

จากข้อมูลการทดสอบ PM ตั้งแต่ 2025-08-07 09:33:28 จนถึง 2025-08-07 16:09:46 พบว่า:

1. **ค่า PM ไม่เสถียร**: เริ่มจาก 0 µg/m³ เพิ่มขึ้นเป็น 48.1 µg/m³ อย่างต่อเนื่อง
2. **ไม่มี Convergence**: ระบบไม่สามารถเข้าสู่ค่าเป้าหมายและคงที่ได้
3. **Oscillation**: มีการเปลี่ยนแปลงค่าที่ไม่สม่ำเสมอ

### การปรับปรุงที่ทำ

#### 1. **ปรับ PID Parameters**
```cpp
// เดิม
float Kp = 0.2;   // Proportional gain
float Ki = 0.01;  // Integral gain  
float Kd = 0.2;   // Derivative gain

// ใหม่ (เพื่อความเสถียร)
float Kp = 0.15;  // ลดลงเพื่อป้องกัน overshooting
float Ki = 0.005; // ลดลงเพื่อป้องกัน integral windup
float Kd = 0.25;  // เพิ่มขึ้นเพื่อการ damping ที่ดีขึ้น
```

#### 2. **เพิ่ม Time-based PID Calculation**
- เพิ่มการติดตาม `dt` (time difference) สำหรับการคำนวณที่แม่นยำ
- ป้องกันการหาร 0 ด้วย minimum time step
- Reset PID state เมื่อเข้าถึงเป้าหมาย

#### 3. **ปรับปรุง Anti-windup Protection**
```cpp
// เพิ่มค่า integral limits
if (target > 20) {
    integralMax = 10.0f;  // เพิ่มจาก 5.0f
    integralMin = -10.0f;
} else {
    integralMax = 8.0f;   // เพิ่มจาก 5.0f  
    integralMin = -8.0f;
}
```

#### 4. **เพิ่ม Derivative Filtering**
```cpp
// Low-pass filter สำหรับ derivative term เพื่อลด noise
static float filtered_derivative = 0;
float alpha = 0.1;
filtered_derivative = alpha * derivative + (1 - alpha) * filtered_derivative;
```

#### 5. **เพิ่ม Deadband Control**
```cpp
// 5% deadband ใกล้เป้าหมายเพื่อลด oscillation
float errorPercent = abs(current_error) / target;
if (errorPercent < 0.05) {
    fanSpeed *= (errorPercent / 0.05);
}
```

#### 6. **เพิ่ม Rate Limiting**
```cpp
// จำกัดอัตราการเปลี่ยนแปลงของ fan speed
float maxChangeRate = 5.0f; // Maximum 5% change per second
float maxChange = maxChangeRate * dt;
```

#### 7. **ปรับ Fan Speed Range**
```cpp
// ลด minimum fan speed เพื่อการควบคุมที่ละเอียดขึ้น
if (target > 20) {
    minFanSpeed = 25.0f; // ลดจาก 30.0f
} else {
    minFanSpeed = 20.0f; // ลดจาก 30.0f
}
```

### ฟังก์ชันใหม่

#### `resetPID()`
```cpp
void Calculator::resetPID() {
    current_error = 0;
    previous_error = 0;
    integral = 0;
    derivative = 0;
    lastTime = 0;
}
```

### วิธีใช้งาน

1. **เรียก `resetPID()`** เมื่อเปลี่ยนเป้าหมายใหม่
2. **Monitor การทำงาน** ผ่าน Serial output
3. **ปรับ parameters** ตามต้องการ:
   - `Kp`: ลดถ้า overshooting, เพิ่มถ้า response ช้า
   - `Ki`: ลดถ้า oscillation, เพิ่มถ้า steady-state error
   - `Kd`: เพิ่มเพื่อ damping ที่ดีขึ้น

### Expected Results

1. **ลด Overshooting**: การเปลี่ยนแปลงที่นุ่มนวลขึ้น
2. **เสถียรภาพดีขึ้น**: ลด oscillation ใกล้เป้าหมาย
3. **การตอบสนองที่ดีขึ้น**: ระยะเวลา settling time ที่เหมาะสม
4. **ลด Noise**: การกรอง derivative term

### การทดสอบ

สำหรับการทดสอบ สามารถปรับค่าเหล่านี้ในไฟล์ `main.cpp`:

```cpp
// สำหรับการทดสอบ - เปลี่ยนเป็นค่าน้อยลง
#define AUTO_START_DELAY_SECONDS 10      // 10 วินาที
#define CYCLE_INTERVAL_SECONDS 30        // 30 วินาที
```

### Notes

- การปรับปรุงนี้เน้นความเสถียรมากกว่าความเร็วในการตอบสนอง
- สามารถปรับแต่ง parameters เพิ่มเติมตามผลการทดสอบจริง
- ควรทดสอบกับเป้าหมาย PM หลายระดับ (5, 10, 20, 40 µg/m³)

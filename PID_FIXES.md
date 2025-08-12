# การแก้ไขสมการ PID Controller

## ปัญหาที่พบและการแก้ไข

### 1. การคำนวณ Error ที่ผิด
**ปัญหาเดิม:**
```cpp
current_error = target - current;
```

**การแก้ไข:**
```cpp
current_error = current - target;
```

**เหตุผล:** สำหรับระบบ PM2.5 controller ที่ต้องการลดค่าฝุ่น ค่า error ควรเป็นบวกเมื่อค่าปัจจุบันสูงกว่าเป้าหมาย เพื่อให้พัดลมทำงานแรงขึ้น

### 2. การคำนวณ Derivative Term
**ปัญหาเดิม:**
```cpp
derivative = (current_error - previous_error) / dt;
```

**การแก้ไข:**
```cpp
derivative = -(current_error - previous_error) / dt;
```

**เหตุผล:** เครื่องหมายลบทำให้ derivative term ช่วยลดการแกว่งของระบบได้ถูกต้อง

### 3. การจัดการเมื่อถึงเป้าหมาย
**ปัญหาเดิม:**
```cpp
if (current >= target) {
    return 0.0f; // ปิดพัดลมทันที
}
```

**การแก้ไข:**
```cpp
if (current_error <= 0) {
    float minSpeedPercent = (ROOM_AIR_LEAK_M3H / FAN_MAX_AIR_FLOW_M3H) * 100;
    integral *= 0.5; // ลดค่า integral แบบค่อยเป็นค่อยไป
    return minSpeedPercent; // รักษาความเร็วขั้นต่ำ
}
```

**เหตุผล:** การปิดพัดลมทันทีอาจทำให้ค่า PM2.5 กลับมาสูงขึ้นเร็ว ควรรักษาการหมุนเวียนอากาศขั้นต่ำ

### 4. การปรับปรุง Output Scaling
**ปัญหาเดิม:**
```cpp
float outputScale = 1.0f / (Kp * target * 0.5f);
```

**การแก้ไข:**
```cpp
float maxExpectedError = target * 2.0f;
float normalizedOutput = pidOutput / maxExpectedError;
```

**เหตุผล:** การ scale ตาม expected error ทำให้ได้ผลลึพธ์ที่สม่ำเสมอและคาดการณ์ได้

### 5. การปรับปรุง PID Gains
**ค่าเดิม:**
- Kp = 0.2, Ki = 0.01, Kd = 0.2

**ค่าใหม่:**
- Kp = 0.3 (เพิ่มการตอบสนอง)
- Ki = 0.005 (ลดเพื่อป้องกัน integral windup)
- Kd = 0.1 (ลดเพื่อลดความไวต่อ noise)

### 6. การปรับปรุง Rate Limiting
**การปรับปรุง:**
- เพิ่ม maxChangeRate จาก 5% เป็น 10% ต่อวินาที
- ปรับ initialization ของ lastFanSpeed ให้เริ่มที่ minFanSpeed
- ปรับ logic ของ deadband ให้เหมาะสมกว่า

## ผลลัพธ์ที่คาดหวัง

1. **การตอบสนองที่ดีขึ้น:** ระบบจะตอบสนองต่อการเปลี่ยนแปลงของ PM2.5 ได้เร็วขึ้น
2. **ความเสถียรที่ดีขึ้น:** ลดการแกว่งของความเร็วพัดลม
3. **ประหยัดพลังงาน:** ไม่ปิดพัดลมทันทีแต่รักษาการหมุนเวียนอากาศขั้นต่ำ
4. **ลด Overshoot:** การปรับ gains และ scaling ช่วยลดการเกินเป้าหมาย

## การทดสอบแนะนำ

1. ทดสอบการตอบสนองเมื่อมีการเปลี่ยนแปลง PM2.5 อย่างรวดเร็ว
2. ทดสอบความเสถียรเมื่อค่า PM2.5 อยู่ใกล้เป้าหมาย
3. ทดสอบการประหยัดพลังงานในระยะยาว
4. Monitor integral windup และ derivative kick

## หมายเหตุ

การแก้ไขนี้อิงตามหลักการ PID controller แบบคลาสสิก แต่ปรับให้เหมาะกับระบบควบคุม PM2.5 โดยเฉพาะ อาจต้องปรับ fine-tune ค่า gains เพิ่มเติมตามผลการทดสอบจริง

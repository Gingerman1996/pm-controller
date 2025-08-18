# PM2.5 Exponential Rise Control System

## การเปลี่ยนแปลงจากระบบเดิม

ระบบได้รับการปรับปรุงจากการควบคุมแบบขั้นบันได (5→10→25→35→50 μg/m³) เป็นการควบคุมแบบ **Exponential Rise** ที่ทำงานใน 3 ขั้นตอน:

1. **Exponential Rise**: เพิ่มขึ้นอย่างต่อเนื่องและราบรื่นจาก **0 ถึง 50 μg/m³** ในระยะเวลา **2 ชั่วโมง**
2. **Maintenance Phase**: รักษาระดับ **50 μg/m³** เป็นเวลา **20 นาที**  
3. **Natural Decay**: ปิดระบบให้ PM ลดลงตามธรรมชาติ

## หลักการทำงาน

### 1. Setpoint Shaping with Exponential Rise
ใช้วิธี **First-order Filter** กับ setpoint เพื่อสร้างการเพิ่มขึ้นแบบ exponential:

```
Target(t) = Max_Target × (1 - e^(-t/τ))
```

โดยที่:
- `Max_Target` = 50 μg/m³
- `τ` = 2400 วินาที (40 นาที) - time constant
- `t` = เวลาที่ผ่านไป (วินาที)

### 2. Filtered Setpoint Control
PID Controller จะติดตาม **filtered setpoint** แทนการติดตาม raw target:

```cpp
filtered_target += alpha * (exponential_target - filtered_target)
error = filtered_target - current_pm
```

## การตั้งค่าระบบ

### ค่าพารามิเตอร์หลัก
```cpp
#define EXPONENTIAL_RISE_DURATION_SECONDS 7200  // 2 ชั่วโมง
#define MAINTENANCE_DURATION_SECONDS 1200       // 20 นาที
const float MAX_PM_TARGET = 50.0                // เป้าหมายสูงสุด
```

### ค่า PID ที่ปรับแล้ว
- **Kp = 0.4** - Proportional gain สำหรับการผลิต PM
- **Ki = 0.008** - Integral gain สำหรับความแม่นยำ
- **Kd = 0.05** - Derivative gain ที่ลดลงเพื่อลด noise
- **Alpha = 0.1** - Filter coefficient สำหรับ setpoint shaping

## คำสั่งการใช้งาน

### Serial Commands
```
start  - เริ่มต้นระบบ exponential rise
stop   - หยุดระบบและตั้งพัดลมเป็น 0
status - แสดงสถานะปัจจุบันของระบบ
help   - แสดงคำสั่งที่ใช้ได้
```

### การทำงานอัตโนมัติ
- ระบบจะเริ่มทำงานอัตโนมัติหลังจาก boot 60 วินาที
- **Phase 1**: ใช้เวลา 2 ชั่วโมงในการเพิ่ม PM จาก 0 ถึง 50 μg/m³
- **Phase 2**: รักษาระดับ 50 μg/m³ เป็นเวลา 20 นาที
- **Phase 3**: ปิดระบบให้ PM ลดลงตามธรรมชาติ

## ข้อดีของระบบใหม่

### 1. การเพิ่มขึ้นที่ราบรื่น
- ไม่มีการเปลี่ยนแปลงแบบกระโดด (step change)
- การเพิ่มขึ้นแบบ exponential เหมือนธรรมชาติ
- เหมาะสำหรับการทดสอบเซ็นเซอร์ PM

### 2. ควบคุมได้ดีกว่า
- ป้องกัน overshoot และ oscillation
- การตอบสนองที่เสถียรและทำนายได้
- ลด noise ในระบบ

### 3. ยืดหยุ่นในการปรับแต่ง
- เปลี่ยนระยะเวลาได้ง่าย (แค่ปรับ `EXPONENTIAL_RISE_DURATION_SECONDS`)
- เปลี่ยนเป้าหมายสูงสุดได้ (ปรับ `MAX_PM_TARGET`)
- ปรับความนุ่มนวลได้ (ปรับ `alpha`)

## การทดสอบระบบ

### สำหรับการทดสอบระยะสั้น
แก้ไขในไฟล์ `main.cpp`:
```cpp
// เปลี่ยนจาก 2 ชั่วโมง เป็น 5 นาที
#define EXPONENTIAL_RISE_DURATION_SECONDS 300  // 5 minutes for testing
#define MAINTENANCE_DURATION_SECONDS 60        // 1 minute for testing
```

### การติดตามความคืบหน้า
ระบบจะ log ความคืบหน้าทุก 30 วินาที:
```
Exponential rise progress: 12.5/50.0 μg/m³ (25.0% complete)
```

## Mathematical Model

### Exponential Rise Function
```
PM(t) = 50 × (1 - e^(-t/2400))
```

### Time to reach percentage of target
- **63.2%** ของเป้าหมาย: 40 นาที (1τ)
- **86.5%** ของเป้าหมาย: 80 นาที (2τ)  
- **95.0%** ของเป้าหมาย: 120 นาที (3τ)

### Complete System Timeline
- **0-120 นาที**: Exponential rise จาก 0→50 μg/m³
- **120-140 นาที**: Maintenance phase ที่ 50 μg/m³  
- **140+ นาที**: Natural decay (ระบบปิด)

### Progress at specific times
- **30 นาที**: ~24 μg/m³ (48%)
- **60 นาที**: ~37 μg/m³ (74%)
- **90 นาที**: ~43 μg/m³ (86%)
- **120 นาที**: ~47.5 μg/m³ (95%)

## การแก้ไขปัญหา

### ถ้าระบบเพิ่มขึ้นเร็วเกินไป
ลดค่า `alpha` ใน Calculator.cpp:
```cpp
alpha = 0.05; // ลดจาก 0.1 เพื่อให้นุ่มนวลขึ้น
```

### ถ้าระบบเพิ่มขึ้นช้าเกินไป
เพิ่มค่า `alpha`:
```cpp
alpha = 0.15; // เพิ่มจาก 0.1 เพื่อให้เร็วขึ้น
```

### ถ้ามี oscillation
ลดค่า Kp และ Kd:
```cpp
Kp = 0.3;  // ลดจาก 0.4
Kd = 0.03; // ลดจาก 0.05
```

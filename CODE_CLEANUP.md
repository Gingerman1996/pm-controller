# การลบโค้ดที่ไม่จำเป็นออกจาก PM Controller

## ฟังก์ชันที่ลบออก

### 1. `getFanRunningInterval()`
```cpp
// ลบออกแล้ว - ไม่ได้ใช้ใน main.cpp
float Calculator::getFanRunningInterval(float current, uint16_t target);
```
**เหตุผล:** ใช้เฉพาะ `getFanRunningIntervalV2()` แทน

### 2. `controlFanRPM()`
```cpp
// ลบออกแล้ว - ไม่ได้ใช้ใน main.cpp
float Calculator::controlFanRPM(double targetPM25FlowRate, double inletPMConcentration);
```
**เหตุผล:** ฟังก์ชันนี้ไม่ได้ถูกเรียกใช้ในระบบปัจจุบัน

### 3. `resetPID()`
```cpp
// ลบออกแล้ว - ไม่ได้ใช้ใน main.cpp
void Calculator::resetPID();
```
**เหตุผล:** ไม่มีการเรียกใช้งานในโค้ดหลัก

## Include Headers ที่ลบออก

### จากไฟล์ Calculator.h:
```cpp
// ลบออกแล้ว
#include <array>     // ไม่ได้ใช้
#include <iostream>  // ไม่ได้ใช้ (ไม่มี console output)
#include <numeric>   // ไม่ได้ใช้
```

**เก็บไว้:**
```cpp
#include <cmath>     // ใช้สำหรับ M_PI, pow(), log(), abs()
```

## ฟังก์ชันที่เก็บไว้ (ยังใช้อยู่)

### 1. `getFanRunningIntervalV2()`
- **ใช้ใน:** การคำนวณ duration สำหรับการทำงานของ generator
- **ตำแหน่งใน main.cpp:** line 417

### 2. `getFanRunSpeed()`
- **ใช้ใน:** การควบคุม PID สำหรับความเร็ว generator  
- **ตำแหน่งใน main.cpp:** line 416

### 3. `scaleDutyCycle()`
- **ใช้ใน:** การแปลง percentage เป็น PWM duty cycle
- **ตำแหน่งใน main.cpp:** line 423

### 4. `calculateInletConcentration()`
- **ใช้ใน:** การคำนวณ inlet concentration
- **ตำแหน่งใน main.cpp:** line 390

### 5. `convertPercentageToRPM()`
- **ใช้ใน:** การแปลง percentage เป็น RPM และการแสดงผล
- **ตำแหน่งใน main.cpp:** line 408 (commented), line 560

### 6. `calculateAirFlowRate()`
- **ใช้ใน:** ฟังก์ชัน `calculateInletConcentration()` เป็น helper function
- **จำเป็น:** ต้องเก็บไว้เพื่อการคำนวณ airflow

## ผลลัพธ์การลบโค้ด

### ประโยชน์:
1. **ลดขนาดไฟล์:** ลดโค้ดที่ไม่จำเป็นประมาณ 50+ บรรทัด
2. **ลด Flash Memory Usage:** ลดการใช้ memory ใน ESP32
3. **โค้ดสะอาดขึ้น:** ง่ายต่อการบำรุงรักษาและเข้าใจ
4. **ลด Compilation Time:** compile เร็วขึ้นเล็กน้อย

### สถิติก่อนและหลัง:
**ก่อนลบ:**
- ฟังก์ชันใน Calculator: 9 ฟังก์ชัน
- Include headers: 4 headers

**หลังลบ:**
- ฟังก์ชันใน Calculator: 6 ฟังก์ชัน  
- Include headers: 1 header
- **ลดลง:** 33% ของฟังก์ชันและ 75% ของ includes

### การทดสอบ:
✅ **Build สำเร็จ:** ไม่มี compilation errors  
✅ **Functionality:** ฟังก์ชันหลักยังคงทำงานได้ตามปกติ

## คำแนะนำ

1. **การ Refactor ต่อ:** อาจพิจารณาลบตัวแปรที่ไม่ได้ใช้ใน main.cpp
2. **การทดสอบ:** ทดสอบการทำงานของระบบให้ครบทุกฟังก์ชัน
3. **Documentation:** อัปเดต documentation ให้สอดคล้องกับโค้ดปัจจุบัน

## หมายเหตุ

การลบโค้ดนี้ไม่ส่งผลกระทบต่อฟังก์ชันการทำงานหลักของระบบ PM generator เนื่องจากลบเฉพาะส่วนที่ไม่ได้ใช้งานจริงเท่านั้น

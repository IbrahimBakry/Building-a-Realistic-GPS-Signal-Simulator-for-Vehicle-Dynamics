## 🚗✨ Building a Realistic GPS Signal Simulator for Vehicle Dynamics  

### 🌐 The Task
*Client needed a physics-based GPS simulator* that generates realistic NMEA-0183 messages (GGA/VTG) for a moving vehicle. Key constraints:  
- **Steering angle**: -600° to +600°  
- **Speed**: 0–30 km/h  
- **Wheelbase**: 2 meters  
- **Update frequency**: 10 Hz (100 ms)  
- **Start coordinates**: 53.262778°N, 50.372778°E  
*Critical requirement*: At constant steering angle/speed, the vehicle **must trace a perfect circle**.  

---

### ⚙️ Core Solution: Physics & Math in Harmony  
I engineered a **dynamic bicycle model** with tire slip dynamics and coordinate transformations:  

![car free body model](https://github.com/user-attachments/assets/6647df14-4995-4749-ba00-44dafe371b1e)

#### 🔄 **Vehicle Dynamics Model**  
*Derived from Newton-Euler equations:*  
```math
\begin{aligned}
&m(\dot{U} - \omega_z V) = F_x \\
&m(\dot{V} + \omega_z U) = F_y \\
&I_z \dot{\omega_z} = M_z
\end{aligned}
```  
Where:  
- `U`, `V` = Longitudinal/lateral velocities  
- `ω_z` = Yaw rate  
- `F_x`, `F_y`, `M_z` = Forces/moment from **tire slip dynamics**  

**Tire forces** were modeled linearly:  
```math
F_{yi} = C_{Fy} \cdot \alpha_i \quad \text{(lateral force)}  
```  
With **slip angles**:  
```math
\alpha_{front} = \delta - \frac{V + a\omega_z}{U}, \quad \alpha_{rear} = -\frac{V - b\omega_z}{U}
```  

![slip angle of the forward wheels](https://github.com/user-attachments/assets/68ce3b51-fb23-4359-9b96-3bbe27cca221)
![slip angle of the backward wheels](https://github.com/user-attachments/assets/4c877af7-98f4-4cfc-a365-96a43eddd94c)

#### 🌍 **Coordinate Transformation Pipeline**  
1. **Body → Local ENU**: Rotation by yaw angle `θ`  
2. **ENU → ECEF**: Accounting for Earth’s curvature and rotation  
3. **ECEF → Geodetic (Lat/Lon)**: Using ellipsoidal Earth model:  
   ```math
   \lambda = \text{atan2}(y,x), \quad \phi = \text{atan2}\left(z, p(1 - e^2 \frac{N}{N+h})\right)
   ```  

![algorithm of solving the model and modeling](https://github.com/user-attachments/assets/fcbff389-b90c-4050-afca-3da3454651a3)

---

### 📡 Generating NMEA-0183 Messages  
Implemented **real-time protocol formatting** at 10 Hz:  

#### 🔷 **GGA Message** (Position)  
`$GNGGA,<time>,<lat>,N,<lon>,E,1,,,,,,*<checksum>`  
- **Lat/Lon formatting**: Converted to `DDMM.MMMMMM`  
  *Example*: `5315.792486` = 53° 15.792486'  

#### 🔷 **VTG Message** (Speed/Course)  
`$GNVTG,<course>,T,,,<speed_knots>,N,<speed_kmph>,K,A*<checksum>`  
- **Course**: True north heading from yaw  
- **Speed**: Converted to knots + km/h  

---

### 🧪 Validation & Results  
**Test 1: Straight Motion** (δ=0°, 20 km/h)  
✅ Vehicle moved due north with <0.1° heading drift  
```nmea
$GPGGA,000006.80,5315.792486,N,05022.366680,E,,,,0.0,M,0.0,M,,,*41  
$GPVTG,0.0,T,,,,13.5,N,,25.0,K,A*40
```  

![linear moving](https://github.com/user-attachments/assets/d4dcd34e-d2d6-4618-bacf-7252b5ca1262)

**Test 2: Circular Motion** (δ=100°, 20 km/h)  
✅ Traced a perfect circle with **constant radius**:  
```nmea
$GPGGA,000032.00,5315.779169,N,05022.382997,E,,,,0.0,M,0.0,M,,,*4C  
$GPVTG,77.0,T,,,,10.8,N,,20.0,K,A*7B
```  

![circular moving](https://github.com/user-attachments/assets/3dcbd958-87a1-4a75-af0c-e0970793f9cc)

---

### 💡 Key Insights  
- **Realism > Simplification**: Linear tire models sufficed for low speeds (<30 km/h) but would need Pacejka models for higher speeds.  
- **Earth Rotation Matters**: Ignoring Coriolis forces caused 0.3% positional drift in early tests.  
- **Edge Handling**: Capped steering angles to ±600° and implemented slip-angle saturation.  

This project blends **applied physics**, **geodesy**, and **real-time systems** to solve a practical automotive testing need—proving that math truly drives the world! 🧮🌎  

*(MATLAB source code available in repository)*
